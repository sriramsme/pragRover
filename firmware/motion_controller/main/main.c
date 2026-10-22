// SPDX-License-Identifier: GPL-3.0-only
#include "cJSON.h"
#include "control.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hardware.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "session.h"
#include "wire.h"
#include <errno.h>
#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char tag[] = "prag_motion";
static RoverController controller;
static SemaphoreHandle_t controller_mutex;
static RoverHardware hardware;
static bool hardware_ready = false;
static atomic_bool stop_requested = ATOMIC_VAR_INIT(false);
/* Keep the shared-state lock away from parsing, allocation and socket I/O. */
static void lock_controller(void) {
    xSemaphoreTake(controller_mutex, portMAX_DELAY);
}
static void unlock_controller(void) {
    xSemaphoreGive(controller_mutex);
}
#ifdef CONFIG_ROVER_WIRING_CONFIRMED
static RoverPins pins(void) {
    return (RoverPins){
        CONFIG_ROVER_LEFT_IN1,        CONFIG_ROVER_LEFT_IN2,       CONFIG_ROVER_RIGHT_IN1,
        CONFIG_ROVER_RIGHT_IN2,       CONFIG_ROVER_SLEEP_PIN,      CONFIG_ROVER_LEFT_ENCODER_A,
        CONFIG_ROVER_RIGHT_ENCODER_A, CONFIG_ROVER_BUMPER_LEFT,    CONFIG_ROVER_BUMPER_RIGHT,
        CONFIG_ROVER_LEFT_ENCODER_B,  CONFIG_ROVER_RIGHT_ENCODER_B};
}
#endif
static double setting(const char *text) {
    char *end = NULL;
    const double value = strtod(text, &end);
    return end && end != text && *end == '\0' ? value : NAN;
}
static RoverConfig load_config(void) {
    RoverConfig c = rover_config_defaults();
#ifdef CONFIG_ROVER_WIRING_CONFIRMED
    RoverPins selected_pins = pins();
    c.wiring_confirmed = rover_pins_valid(&selected_pins);
#endif
    c.wheel_radius_m = setting(CONFIG_ROVER_WHEEL_RADIUS_M);
    c.track_m = setting(CONFIG_ROVER_TRACK_M);
    c.counts_per_revolution = setting(CONFIG_ROVER_COUNTS_PER_REVOLUTION);
    c.kp = setting(CONFIG_ROVER_KP);
    c.ki = setting(CONFIG_ROVER_KI);
    c.feedforward = setting(CONFIG_ROVER_FEEDFORWARD);
    c.max_speed_mps = setting(CONFIG_ROVER_MAX_SPEED_MPS);
    c.max_acceleration_mps2 = setting(CONFIG_ROVER_MAX_ACCELERATION_MPS2);
    c.max_duty = setting(CONFIG_ROVER_MAX_DUTY);
    c.left_quadrature = CONFIG_ROVER_LEFT_ENCODER_B >= 0;
    c.right_quadrature = CONFIG_ROVER_RIGHT_ENCODER_B >= 0;
    return c;
}
static void disconnect_controller(void) {
    atomic_store(&stop_requested, true);
    lock_controller();
    rover_controller_disconnect(&controller);
    unlock_controller();
}
static void control_task(void *argument) {
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        const int64_t now = esp_timer_get_time();
        const bool bumper = hardware_ready && rover_hardware_bumper(&hardware);
        int64_t left = 0, right = 0;
        const bool encoder_error =
            hardware_ready && rover_hardware_edges(&hardware, &left, &right) != ESP_OK;
        RoverOutput output = {0};
        {
            lock_controller();
            if (encoder_error) {
                rover_controller_trip(&controller, ROVER_FAULT_HARDWARE);
            }
            output = rover_controller_tick(&controller, now, left, right, bumper);
            unlock_controller();
        }
        if (atomic_exchange(&stop_requested, false)) {
            output = (RoverOutput){0};
        }
        if (hardware_ready && rover_hardware_apply(&hardware, output, now) != ESP_OK) {
            rover_hardware_disable(&hardware);
            lock_controller();
            rover_controller_trip(&controller, ROVER_FAULT_HARDWARE);
            unlock_controller();
        }
        // This task never parses JSON, accesses sockets, allocates request data,
        // or waits for AI/host responses. It is still not a certified stop circuit.
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}
static void wifi_event(void *argument, esp_event_base_t base, int32_t event, void *data) {
    (void)argument;
    (void)data;
    if (base == WIFI_EVENT && event == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    if (base == WIFI_EVENT && event == WIFI_EVENT_STA_DISCONNECTED) {
        disconnect_controller();
        ESP_LOGW(tag, "Wi-Fi disconnected; motion fault latched. Reboot to reconnect.");
    }
}
static bool start_wifi(void) {
    if (strlen(CONFIG_ROVER_WIFI_SSID) == 0 || strlen(CONFIG_ROVER_SHARED_SECRET) < 16) {
        ESP_LOGW(tag, "Network disabled: configure SSID and >=16-character shared secret.");
        return false;
    }
    if (strlen(CONFIG_ROVER_WIFI_SSID) > 32 || strlen(CONFIG_ROVER_WIFI_PASSWORD) > 63 ||
        strlen(CONFIG_ROVER_SHARED_SECRET) > 128) {
        ESP_LOGE(tag, "Network configuration invalid");
        return false;
    }
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    wifi_config_t wifi = {0};
    memcpy(wifi.sta.ssid, CONFIG_ROVER_WIFI_SSID, strlen(CONFIG_ROVER_WIFI_SSID));
    memcpy(wifi.sta.password, CONFIG_ROVER_WIFI_PASSWORD, strlen(CONFIG_ROVER_WIFI_PASSWORD));
    wifi.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    return true;
}
static void random_hex(char output[33]) {
    unsigned char bytes[16];
    esp_fill_random(bytes, sizeof(bytes));
    const char digits[] = "0123456789abcdef";
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        output[2 * i] = digits[bytes[i] >> 4];
        output[2 * i + 1] = digits[bytes[i] & 15];
    }
    output[32] = '\0';
}
static bool constant_equal(const char *a, const char *b) {
    if (!a || !b) {
        return false;
    }
    const size_t al = strlen(a), bl = strlen(b);
    unsigned difference = (unsigned)(al ^ bl);
    for (size_t i = 0; i < al; ++i) {
        difference |= (unsigned char)(a[i]) ^ (unsigned char)(i < bl ? b[i] : 0);
    }
    return difference == 0;
}
static cJSON *telemetry(void) {
    RoverSnapshot s;
    RoverConfig c;
    {
        lock_controller();
        s = controller.snapshot;
        c = controller.config;
        unlock_controller();
    }
    cJSON *out = cJSON_CreateObject();
    cJSON_AddStringToObject(out, "state", rover_state_name(s.state));
    if (s.fault == ROVER_FAULT_NONE) {
        cJSON_AddNullToObject(out, "fault");
    } else {
        cJSON_AddStringToObject(out, "fault", rover_fault_name(s.fault));
    }
    cJSON_AddBoolToObject(out, "bumper", s.bumper);
    cJSON_AddBoolToObject(out, "simulated", false);
    cJSON_AddBoolToObject(out, "wiring_confirmed", c.wiring_confirmed);
    cJSON_AddBoolToObject(out, "left_quadrature", c.left_quadrature);
    cJSON_AddBoolToObject(out, "right_quadrature", c.right_quadrature);
#define ADD_NUMBER(member) cJSON_AddNumberToObject(out, #member, (double)(s.member))
    ADD_NUMBER(monotonic_us);
    ADD_NUMBER(left_mps);
    ADD_NUMBER(right_mps);
    ADD_NUMBER(left_distance_m);
    ADD_NUMBER(right_distance_m);
    ADD_NUMBER(left_counts);
    ADD_NUMBER(right_counts);
    ADD_NUMBER(raw_left_counts);
    ADD_NUMBER(raw_right_counts);
    ADD_NUMBER(x_m);
    ADD_NUMBER(y_m);
    ADD_NUMBER(heading_rad);
    ADD_NUMBER(last_motion_sequence);
    ADD_NUMBER(left_duty);
    ADD_NUMBER(right_duty);
    ADD_NUMBER(left_target_mps);
    ADD_NUMBER(right_target_mps);
    ADD_NUMBER(max_tick_interval_us);
#undef ADD_NUMBER
    cJSON_AddNumberToObject(out, "wheel_radius_m", c.wheel_radius_m);
    cJSON_AddNumberToObject(out, "track_m", c.track_m);
    cJSON_AddNumberToObject(out, "counts_per_revolution", c.counts_per_revolution);
    cJSON_AddNumberToObject(out, "max_speed_mps", c.max_speed_mps);
    return out;
}
static bool send_json(int socket, cJSON *object) {
    char *encoded = cJSON_PrintUnformatted(object);
    cJSON_Delete(object);
    if (!encoded) {
        return false;
    }
    const size_t length = strlen(encoded);
    size_t sent = 0;
    const int64_t deadline = esp_timer_get_time() + 1000000;
    bool success = false;
    while (sent < length) {
        if (esp_timer_get_time() >= deadline) {
            goto cleanup;
        }
        const int written = send(socket, encoded + sent, length - sent, 0);
        if (written <= 0) {
            goto cleanup;
        }
        sent += (size_t)written;
    }
    success = esp_timer_get_time() < deadline && send(socket, "\n", 1, 0) == 1;
cleanup:
    cJSON_free(encoded);
    return success;
}
// One-byte reads avoid losing pipelined bytes. There is a 1 s absolute line
// deadline after the first byte, so a slow byte trickle cannot hold the parser.
// Authenticated idle connections remain open; drive leases still expire locally.
static bool read_line(int socket, char line[1025], size_t *length, bool allow_idle) {
    *length = 0;
    int64_t deadline = esp_timer_get_time() + 1000000;
    while (*length <= 1024) {
        if ((!allow_idle || *length != 0) && esp_timer_get_time() >= deadline) {
            return false;
        }
        char ch;
        const int received = recv(socket, &ch, 1, 0);
        if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        if (received != 1) {
            return false;
        }
        if (*length == 0) {
            deadline = esp_timer_get_time() + 1000000;
        }
        if (ch == '\n') {
            line[*length] = '\0';
            return *length != 0;
        }
        if (ch == '\0' || *length == 1024) {
            return false;
        }
        line[(*length)++] = ch;
    }
    return false;
}
static void serve_owner(int socket) {
    char line[1025], session[33] = {0}, ticket[33] = {0};
    size_t length = 0;
    RoverSessionGate gate;
    rover_session_reset(&gate);
    bool authenticated = false;
    while (read_line(socket, line, &length, authenticated)) {
        RoverRequest request;
        const char *error = rover_parse_request(line, length, &request);
        if (!error && !authenticated) {
            if (request.op != ROVER_OP_HELLO ||
                !constant_equal(request.token, CONFIG_ROVER_SHARED_SECRET)) {
                error = "authentication_failed";
            } else {
                {
                    lock_controller();
                    rover_controller_connect(&controller);
                    unlock_controller();
                }
                atomic_store(&stop_requested, true);
                random_hex(session);
                rover_session_reset(&gate);
                authenticated = true;
            }
        } else if (!error) {
            const bool status = request.op == ROVER_OP_STATUS, stop = request.op == ROVER_OP_STOP;
            if (!constant_equal(request.session, session)) {
                error = "session_rejected";
            } else {
                error =
                    rover_session_authorize(&gate, request.sequence, esp_timer_get_time(),
                                            constant_equal(request.ticket, ticket), stop, status);
            }
            if (!error) {
                if (status) { /* Snapshot only. */
                } else if (stop) {
                    atomic_store(&stop_requested, true);
                    lock_controller();
                    rover_controller_stop(&controller);
                    unlock_controller();
                } else if (request.op == ROVER_OP_ARM) {
                    lock_controller();
                    error = rover_controller_arm(&controller, esp_timer_get_time());
                    unlock_controller();
                } else if (request.op == ROVER_OP_RESET) {
                    lock_controller();
                    error = rover_controller_reset(&controller, esp_timer_get_time());
                    unlock_controller();
                } else if (request.op == ROVER_OP_DRIVE) {
                    lock_controller();
                    error = rover_controller_drive(&controller, request.left_mps, request.right_mps,
                                                   request.lease_ms, request.sequence,
                                                   esp_timer_get_time());
                    unlock_controller();
                } else {
                    error = "unknown_operation";
                }
            }
        }
        cJSON *response = cJSON_CreateObject();
        cJSON_AddNumberToObject(response, "version", 1);
        cJSON_AddBoolToObject(response, "ok", !error);
        if (error) {
            cJSON_AddStringToObject(response, "error", error);
        }
        if (authenticated) {
            random_hex(ticket);
            rover_session_issue(&gate, esp_timer_get_time());
            cJSON_AddStringToObject(response, "session", session);
            cJSON_AddStringToObject(response, "ticket", ticket);
            cJSON_AddNumberToObject(response, "ticket_ttl_ms", 250);
            cJSON_AddItemToObject(response, "telemetry", telemetry());
        }
        if (!send_json(socket, response) || !authenticated) {
            break;
        }
    }
    if (authenticated) {
        disconnect_controller();
    }
}
static void network_task(void *argument) {
    (void)argument;
    const int server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server < 0) {
        ESP_LOGE(tag, "socket failed");
        vTaskDelete(NULL);
        return;
    }
    int reuse = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(CONFIG_ROVER_TCP_PORT);
    if (bind(server, (struct sockaddr *)&address, sizeof(address)) || listen(server, 1)) {
        close(server);
        ESP_LOGE(tag, "bind/listen failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(tag, "Motion TCP server port %d (local LAN, no TLS)", CONFIG_ROVER_TCP_PORT);
    for (;;) {
        const int client = accept(server, NULL, NULL);
        if (client < 0) {
            continue;
        }
        struct timeval timeout = {0, 200000};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        int no_delay = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
        serve_owner(client);
        shutdown(client, SHUT_RDWR);
        close(client);
    }
}
void app_main(void) {
    RoverConfig c = load_config();
    controller_mutex = xSemaphoreCreateMutex();
    if (!controller_mutex) {
        ESP_LOGE(tag, "mutex allocation failed");
        return;
    }
    rover_controller_init(&controller, c);
#ifdef CONFIG_ROVER_WIRING_CONFIRMED
    if (!c.wiring_confirmed) {
        rover_controller_trip(&controller, ROVER_FAULT_CONFIGURATION);
        ESP_LOGE(tag, "Configuration invalid; no peripherals initialized");
    } else {
        if (rover_hardware_initialize(&hardware, pins()) == ESP_OK) {
            hardware_ready = true;
            if (!rover_config_calibrated(&c)) {
                ESP_LOGW(tag, "Inputs available for calibration; arming disabled until "
                              "calibration/gains are valid");
            }
        } else {
            rover_hardware_disable(&hardware);
            rover_controller_trip(&controller, ROVER_FAULT_HARDWARE);
        }
    }
#else
    ESP_LOGW(tag, "Boardless mode: GPIO/PWM/PCNT untouched; arming is disabled");
#endif
    if (xTaskCreatePinnedToCore(control_task, "wheel_control", 4096, NULL, 12, NULL, 1) != pdPASS) {
        rover_hardware_disable(&hardware);
        ESP_LOGE(tag, "control task allocation failed");
        return;
    }
    if (start_wifi() &&
        xTaskCreatePinnedToCore(network_task, "motion_tcp", 8192, NULL, 4, NULL, 0) != pdPASS) {
        disconnect_controller();
        ESP_LOGE(tag, "network task allocation failed");
    }
}
