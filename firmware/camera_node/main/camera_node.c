// SPDX-License-Identifier: GPL-3.0-only
// Camera GPIO facts: Espressif arduino-esp32 3.3.7
// CameraWebServer/camera_pins.h, CAMERA_MODEL_AI_THINKER. This is an opt-in
// candidate profile, not board detection.
#include <inttypes.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char kTag[] = "rover_camera";
#define MAX_JPEG_BYTES (256 * 1024)
static const int64_t kMaxFrameAgeUs = 2000000;
static const char kBoundary[] = "prag-rover-frame";
static SemaphoreHandle_t frame_mutex = NULL;
static uint8_t *latest_jpeg = NULL;
static size_t latest_size = 0;
static uint64_t latest_frame_id = 0;
static int64_t latest_captured_us = 0;
static char boot_id[33] = {0};
static atomic_bool connected = ATOMIC_VAR_INIT(false);

typedef struct {
    int64_t deadline_us;
} SendBudget;

static int bounded_send(httpd_handle_t server, int socket, const char *data, size_t size,
                        int flags) {
    SendBudget *budget = httpd_sess_get_ctx(server, socket);
    if (!budget || esp_timer_get_time() >= budget->deadline_us) {
        return HTTPD_SOCK_ERR_TIMEOUT;
    }
    // Socket send timeout is two seconds. Checking each partial send also bounds
    // slow trickle readers across httpd_send_all's repeated calls. This is not a
    // scheduler or hard-real-time guarantee.
    const int result = send(socket, data, size, flags);
    return result < 0 ? HTTPD_SOCK_ERR_FAIL : result;
}

static bool set_send_budget(httpd_req_t *request, int seconds) {
    SendBudget *budget = request->sess_ctx;
    if (!budget) {
        budget = malloc(sizeof(SendBudget));
        if (!budget) {
            return false;
        }
        request->sess_ctx = budget;
        request->free_ctx = free;
    }
    budget->deadline_us = esp_timer_get_time() + (int64_t)(seconds) * 1000000;
    return httpd_sess_set_send_override(request->handle, httpd_req_to_sockfd(request),
                                        bounded_send) == ESP_OK;
}

// One request owns this copy, never a camera-driver buffer. In particular a
// slow stream client cannot hold the acquisition mutex or starve /capture.
typedef struct {
    uint8_t *data;
    size_t size;
    uint64_t frame_id;
    int64_t captured_us;
} FrameCopy;

static void frame_copy_init(FrameCopy *frame) {
    *frame = (FrameCopy){0};
    frame->data = heap_caps_malloc(MAX_JPEG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
static void frame_copy_release(FrameCopy *frame) {
    heap_caps_free(frame->data);
    frame->data = NULL;
}
static bool frame_copy_refresh(FrameCopy *frame) {
    if (!frame->data || xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return false;
    }
    const int64_t now = esp_timer_get_time();
    const bool valid =
        latest_size > 0 && latest_captured_us <= now && now - latest_captured_us <= kMaxFrameAgeUs;
    if (valid) {
        frame->size = latest_size;
        frame->frame_id = latest_frame_id;
        frame->captured_us = latest_captured_us;
        memcpy(frame->data, latest_jpeg, frame->size);
    }
    xSemaphoreGive(frame_mutex);
    return valid;
}

static bool authorized(httpd_req_t *request) {
    enum { kMaxHeader = 264 };
    char actual[kMaxHeader] = {0};
    char expected[kMaxHeader] = {0};
    const size_t header_size = httpd_req_get_hdr_value_len(request, "Authorization");
    const int expected_size =
        snprintf(expected, sizeof(expected), "Bearer %s", CONFIG_ROVER_CAMERA_BEARER_TOKEN);
    if (header_size == 0 || header_size >= sizeof(actual) || expected_size <= 0 ||
        (size_t)(expected_size) != header_size ||
        httpd_req_get_hdr_value_str(request, "Authorization", actual, sizeof(actual)) != ESP_OK) {
        return false;
    }
    // Do not log either value. Compare all bytes for equal-length tokens.
    unsigned difference = 0;
    for (size_t i = 0; i < header_size; ++i) {
        difference |= (unsigned char)(actual[i]) ^ (unsigned char)(expected[i]);
    }
    return difference == 0;
}

static esp_err_t reject_unauthorized(httpd_req_t *request) {
    httpd_resp_set_status(request, "401 Unauthorized");
    httpd_resp_set_hdr(request, "WWW-Authenticate", "Bearer");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, "authorization required\n");
}

static esp_err_t unavailable(httpd_req_t *request) {
    httpd_resp_set_status(request, "503 Service Unavailable");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, "fresh camera frame unavailable\n");
}

static esp_err_t capture_handler(httpd_req_t *request) {
    if (!set_send_budget(request, 10)) {
        return ESP_FAIL;
    }
    if (!authorized(request)) {
        return reject_unauthorized(request);
    }
    esp_err_t result;
    FrameCopy frame;
    frame_copy_init(&frame);
    if (!atomic_load(&connected) || !frame_copy_refresh(&frame)) {
        result = unavailable(request);
        goto cleanup;
    }
    char frame_id[24];
    char captured_us[24];
    snprintf(frame_id, sizeof(frame_id), "%" PRIu64, frame.frame_id);
    snprintf(captured_us, sizeof(captured_us), "%" PRId64, frame.captured_us);
    httpd_resp_set_type(request, "image/jpeg");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "X-Rover-Frame-Id", frame_id);
    httpd_resp_set_hdr(request, "X-Rover-Captured-Us", captured_us);
    httpd_resp_set_hdr(request, "X-Rover-Boot-Id", boot_id);
    result = httpd_resp_send(request, (const char *)frame.data, frame.size);
cleanup:
    frame_copy_release(&frame);
    return result;
}

static esp_err_t stream_handler(httpd_req_t *request) {
    if (!set_send_budget(request, CONFIG_ROVER_CAMERA_STREAM_SECONDS + 2)) {
        return ESP_FAIL;
    }
    if (!authorized(request)) {
        return reject_unauthorized(request);
    }
    esp_err_t result = ESP_OK;
    FrameCopy frame;
    frame_copy_init(&frame);
    if (!atomic_load(&connected) || !frame_copy_refresh(&frame)) {
        result = unavailable(request);
        goto cleanup;
    }
    httpd_resp_set_type(request, "multipart/x-mixed-replace; boundary=prag-rover-frame");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "X-Rover-Boot-Id", boot_id);
    const int64_t finish =
        esp_timer_get_time() + (int64_t)(CONFIG_ROVER_CAMERA_STREAM_SECONDS) * 1000000;
    uint64_t sent_id = 0;
    while (atomic_load(&connected) && esp_timer_get_time() < finish) {
        if (!frame_copy_refresh(&frame)) {
            result = ESP_FAIL;
            goto cleanup; // HTTPD closes the session; release our private copy.
        }
        if (frame.frame_id != sent_id) {
            char header[240];
            const int length = snprintf(
                header, sizeof(header),
                "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n"
                "X-Rover-Frame-Id: %" PRIu64 "\r\nX-Rover-Captured-Us: %" PRId64 "\r\n\r\n",
                kBoundary, frame.size, frame.frame_id, frame.captured_us);
            if (length <= 0 || (size_t)(length) >= sizeof(header)) {
                result = ESP_FAIL;
                goto cleanup;
            }
            result = httpd_resp_send_chunk(request, header, length);
            if (result == ESP_OK) {
                result = httpd_resp_send_chunk(request, (const char *)frame.data, frame.size);
            }
            if (result == ESP_OK) {
                result = httpd_resp_send_chunk(request, "\r\n", 2);
            }
            if (result != ESP_OK) {
                goto cleanup;
            }
            sent_id = frame.frame_id;
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / CONFIG_ROVER_CAMERA_STREAM_FPS));
    }
    result = httpd_resp_send_chunk(request, "--prag-rover-frame--\r\n", HTTPD_RESP_USE_STRLEN);
    if (result == ESP_OK) {
        result = httpd_resp_send_chunk(request, NULL, 0);
    }
cleanup:
    frame_copy_release(&frame);
    return result;
}

static void acquisition_task(void *argument) {
    (void)argument;
    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame) {
            if (frame->format == PIXFORMAT_JPEG && frame->len > 0 && frame->len <= MAX_JPEG_BYTES &&
                xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(250)) == pdTRUE) {
                memcpy(latest_jpeg, frame->buf, frame->len);
                latest_size = frame->len;
                latest_captured_us =
                    (int64_t)(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec;
                ++latest_frame_id;
                xSemaphoreGive(frame_mutex);
            }
            // Every non-null driver buffer is returned on every path.
            esp_camera_fb_return(frame);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static esp_err_t initialize_camera(void) {
#ifndef CONFIG_SPIRAM
    ESP_LOGE(kTag, "PSRAM disabled; confirm board profile and enable PSRAM first");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!esp_psram_is_initialized() || esp_psram_get_size() < 2 * 1024 * 1024) {
        ESP_LOGE(kTag, "At least 2 MiB initialized PSRAM is required; camera disabled");
        return ESP_ERR_NO_MEM;
    }
    camera_config_t camera = {0};
    camera.pin_pwdn = 32;
    camera.pin_reset = -1;
    camera.pin_xclk = 0;
    camera.pin_sccb_sda = 26;
    camera.pin_sccb_scl = 27;
    camera.pin_d7 = 35;
    camera.pin_d6 = 34;
    camera.pin_d5 = 39;
    camera.pin_d4 = 36;
    camera.pin_d3 = 21;
    camera.pin_d2 = 19;
    camera.pin_d1 = 18;
    camera.pin_d0 = 5;
    camera.pin_vsync = 25;
    camera.pin_href = 23;
    camera.pin_pclk = 22;
    camera.xclk_freq_hz = 20000000;
    camera.ledc_timer = LEDC_TIMER_0;
    camera.ledc_channel = LEDC_CHANNEL_0;
    camera.pixel_format = PIXFORMAT_JPEG;
    camera.frame_size = FRAMESIZE_VGA;
    camera.jpeg_quality = CONFIG_ROVER_CAMERA_JPEG_QUALITY;
    camera.fb_count = 2;
    camera.fb_location = CAMERA_FB_IN_PSRAM;
    camera.grab_mode = CAMERA_GRAB_LATEST;
    const esp_err_t result = esp_camera_init(&camera);
    if (result != ESP_OK) {
        return result;
    }
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor || sensor->id.PID != OV2640_PID) {
        esp_camera_deinit();
        ESP_LOGE(kTag, "Confirmed profile requires OV2640; camera disabled");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
#endif
}

static void wifi_event(void *argument, esp_event_base_t base, int32_t id, void *data) {
    (void)argument;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        atomic_store(&connected, false);
        // Camera networking only: no actuator authority, no remembered motion.
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        atomic_store(&connected, true);
        ESP_LOGI(kTag, "Wi-Fi station connected; snapshot and stream ports %d / %d",
                 CONFIG_ROVER_CAMERA_HTTP_PORT, CONFIG_ROVER_CAMERA_STREAM_PORT);
    }
}

static esp_err_t initialize_wifi(void) {
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Do not erase user storage automatically. A deliberate provisioning step
        // can recover NVS after preserving anything valuable on the real board.
        return result;
    }
    ESP_RETURN_ON_ERROR(result, kTag, "NVS initialization failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), kTag, "Network initialization failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), kTag, "Event loop failed");
    if (!esp_netif_create_default_wifi_sta()) {
        return ESP_ERR_NO_MEM;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), kTag, "Wi-Fi initialization failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), kTag, "Wi-Fi storage failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL),
                        kTag, "Wi-Fi event registration failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL),
                        kTag, "IP event registration failed");
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, CONFIG_ROVER_CAMERA_WIFI_SSID, strlen(CONFIG_ROVER_CAMERA_WIFI_SSID));
    memcpy(config.sta.password, CONFIG_ROVER_CAMERA_WIFI_PASSWORD,
           strlen(CONFIG_ROVER_CAMERA_WIFI_PASSWORD));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), kTag, "Wi-Fi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), kTag, "Wi-Fi config failed");
    return esp_wifi_start();
}

static esp_err_t start_http(void) {
    httpd_handle_t snapshot_server = NULL;
    httpd_handle_t stream_server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_ROVER_CAMERA_HTTP_PORT;
    config.ctrl_port = 32768;
    config.max_open_sockets = 2;
    config.max_uri_handlers = 1;
    config.stack_size = 8192;
    config.recv_wait_timeout = 2;
    config.send_wait_timeout = 2;
    config.lru_purge_enable = true;
    ESP_RETURN_ON_ERROR(httpd_start(&snapshot_server, &config), kTag, "Snapshot server failed");
    httpd_uri_t capture = {0};
    capture.uri = "/capture";
    capture.method = HTTP_GET;
    capture.handler = capture_handler;
    esp_err_t result = httpd_register_uri_handler(snapshot_server, &capture);
    if (result != ESP_OK) {
        httpd_stop(snapshot_server);
        return result;
    }
    config.server_port = CONFIG_ROVER_CAMERA_STREAM_PORT;
    config.ctrl_port = 32769; // Each HTTPD instance needs its own control socket.
    config.max_open_sockets = 1;
    result = httpd_start(&stream_server, &config);
    if (result != ESP_OK) {
        httpd_stop(snapshot_server);
        return result;
    }
    httpd_uri_t stream = {0};
    stream.uri = "/stream";
    stream.method = HTTP_GET;
    stream.handler = stream_handler;
    result = httpd_register_uri_handler(stream_server, &stream);
    if (result != ESP_OK) {
        httpd_stop(stream_server);
        httpd_stop(snapshot_server);
    }
    return result;
}

static bool valid_configuration(void) {
#ifndef CONFIG_ROVER_CAMERA_PROFILE_CONFIRMED
    ESP_LOGE(kTag, "Camera disabled: verify board/profile and opt in through menuconfig");
    return false;
#endif
    const size_t ssid_size = strlen(CONFIG_ROVER_CAMERA_WIFI_SSID);
    const size_t password_size = strlen(CONFIG_ROVER_CAMERA_WIFI_PASSWORD);
    const size_t token_size = strlen(CONFIG_ROVER_CAMERA_BEARER_TOKEN);
    if (ssid_size == 0 || ssid_size > 32 || password_size < 8 || password_size > 63 ||
        token_size < 24 || token_size > 240 ||
        CONFIG_ROVER_CAMERA_HTTP_PORT == CONFIG_ROVER_CAMERA_STREAM_PORT) {
        ESP_LOGE(kTag, "Invalid Wi-Fi/token/port configuration; camera disabled");
        return false;
    }
    for (size_t i = 0; i < token_size; ++i) {
        const unsigned char ch = (unsigned char)(CONFIG_ROVER_CAMERA_BEARER_TOKEN[i]);
        if (ch <= 32 || ch >= 127) {
            ESP_LOGE(kTag, "Bearer token must contain visible ASCII without spaces");
            return false;
        }
    }
    return true;
}

void app_main(void) {
    if (!valid_configuration()) {
        return; // No GPIO or network initialization before explicit confirmation.
    }
    ESP_ERROR_CHECK(initialize_camera());
    latest_jpeg = heap_caps_malloc(MAX_JPEG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    frame_mutex = xSemaphoreCreateMutex();
    if (!latest_jpeg || !frame_mutex) {
        if (frame_mutex) {
            vSemaphoreDelete(frame_mutex);
        }
        heap_caps_free(latest_jpeg);
        esp_camera_deinit();
        ESP_LOGE(kTag, "Frame cache allocation failed; camera disabled");
        return;
    }
    uint32_t random[4];
    esp_fill_random(random, sizeof(random));
    snprintf(boot_id, sizeof(boot_id), "%08" PRIx32 "%08" PRIx32 "%08" PRIx32 "%08" PRIx32,
             random[0], random[1], random[2], random[3]);
    ESP_ERROR_CHECK(initialize_wifi());
    if (xTaskCreate(acquisition_task, "camera_acquire", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(kTag, "Acquisition task allocation failed");
        esp_restart();
        return;
    }
    ESP_ERROR_CHECK(start_http());
}
