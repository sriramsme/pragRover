// SPDX-License-Identifier: GPL-3.0-only
#include "hardware.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include <math.h>

bool rover_pins_valid(const RoverPins *pins) {
    int values[] = {pins->left_in1,     pins->left_in2, pins->right_in1, pins->right_in2,
                    pins->sleep,        pins->left_a,   pins->right_a,   pins->bumper_left,
                    pins->bumper_right, pins->left_b,   pins->right_b};
    unsigned count = 9;
    if (pins->left_b >= 0) {
        values[count++] = pins->left_b;
    }
    if (pins->right_b >= 0) {
        values[count++] = pins->right_b;
    }
    return rover_valid_pins(values, count, 5);
}
static esp_err_t encoder(int a, int b, pcnt_unit_handle_t *unit) {
    pcnt_unit_config_t cfg = {0};
    cfg.low_limit = -30000;
    cfg.high_limit = 30000;
    cfg.flags.accum_count = true;
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&cfg, unit), "encoder", "new unit");
    pcnt_glitch_filter_config_t filter = {0};
    filter.max_glitch_ns = CONFIG_ROVER_ENCODER_FILTER_NS;
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(*unit, &filter), "encoder", "filter");
    pcnt_chan_config_t channel = {0};
    channel.edge_gpio_num = a;
    channel.level_gpio_num = b;
    pcnt_channel_handle_t ca = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit, &channel, &ca), "encoder", "channel A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(ca, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     b < 0 ? PCNT_CHANNEL_EDGE_ACTION_HOLD
                                                           : PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                        "encoder", "edge A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(ca, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      b < 0 ? PCNT_CHANNEL_LEVEL_ACTION_KEEP
                                                            : PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        "encoder", "level A");
    if (b >= 0) {
        channel.edge_gpio_num = b;
        channel.level_gpio_num = a;
        pcnt_channel_handle_t cb = NULL;
        ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit, &channel, &cb), "encoder", "channel B");
        ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(cb, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                         PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                            "encoder", "edge B");
        ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(cb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                          PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                            "encoder", "level B");
    }
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(*unit, -30000), "encoder", "low overflow");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(*unit, 30000), "encoder", "high overflow");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(*unit), "encoder", "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(*unit), "encoder", "clear");
    return pcnt_unit_start(*unit);
}
esp_err_t rover_hardware_initialize(RoverHardware *hardware, RoverPins pins) {
    if (!rover_pins_valid(&pins)) {
        return ESP_ERR_INVALID_ARG;
    }
    hardware->pins = pins;
    // Set output latch low before making nSLEEP an output. External pull-down is
    // required to hold disabled while boot ROM/reset leaves pins high impedance.
    ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)(pins.sleep), 0), "motor", "sleep latch");
    gpio_config_t sleep = {0};
    sleep.pin_bit_mask = 1ULL << pins.sleep;
    sleep.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&sleep), "motor", "sleep output");
    hardware->configured = true;
    rover_hardware_disable(hardware);
    const int bumper_pins[] = {pins.bumper_left, pins.bumper_right};
    for (unsigned i = 0; i < 2; ++i) {
        const int pin = bumper_pins[i];
        gpio_config_t input = {0};
        input.pin_bit_mask = 1ULL << pin;
        input.mode = GPIO_MODE_INPUT;
        input.pull_up_en = pin < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&input), "bumper", "input");
    }
    ledc_timer_config_t timer = {0};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 20000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), "motor", "PWM timer");
    const int output_pins[] = {pins.left_in1, pins.left_in2, pins.right_in1, pins.right_in2};
    for (int i = 0; i < 4; ++i) {
        ledc_channel_config_t channel = {0};
        channel.gpio_num = output_pins[i];
        channel.speed_mode = LEDC_LOW_SPEED_MODE;
        channel.channel = (ledc_channel_t)(i);
        channel.timer_sel = LEDC_TIMER_0;
        channel.intr_type = LEDC_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), "motor", "PWM channel");
    }
    ESP_RETURN_ON_ERROR(encoder(pins.left_a, pins.left_b, &hardware->left), "encoder", "left");
    return encoder(pins.right_a, pins.right_b, &hardware->right);
}
bool rover_hardware_bumper(const RoverHardware *hardware) {
    // NC contacts to ground: pressed OR disconnected reads high. External 3.3 V
    // pull-ups are recommended; input-only GPIO34-39 REQUIRE external pull-ups.
    return hardware->configured && (gpio_get_level((gpio_num_t)(hardware->pins.bumper_left)) ||
                                    gpio_get_level((gpio_num_t)(hardware->pins.bumper_right)));
}
static int64_t count_delta(uint32_t now, uint32_t before) {
    const uint32_t difference = now - before;
    return difference <= 0x7fffffffU ? (int64_t)difference : (int64_t)difference - 0x100000000LL;
}
esp_err_t rover_hardware_edges(RoverHardware *hardware, int64_t *left, int64_t *right) {
    int l = 0, r = 0;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(hardware->left, &l), "encoder", "left count");
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(hardware->right, &r), "encoder", "right count");
    // Wrap-aware difference: extend accumulated 32-bit readings without clearing
    // the live counter (which would lose edges between read and clear).
    const uint32_t lu = (uint32_t)(l), ru = (uint32_t)(r);
    *left = count_delta(lu, hardware->previous_left);
    *right = count_delta(ru, hardware->previous_right);
    hardware->previous_left = lu;
    hardware->previous_right = ru;
#ifdef CONFIG_ROVER_INVERT_LEFT_ENCODER
    if (hardware->pins.left_b >= 0) {
        *left = -*left;
    }
#endif
#ifdef CONFIG_ROVER_INVERT_RIGHT_ENCODER
    if (hardware->pins.right_b >= 0) {
        *right = -*right;
    }
#endif
    return ESP_OK;
}
void rover_hardware_disable(RoverHardware *hardware) {
    if (hardware->configured) {
        gpio_set_level((gpio_num_t)(hardware->pins.sleep), 0);
    }
    hardware->awake = false;
}
esp_err_t rover_hardware_apply(RoverHardware *hardware, RoverOutput output, int64_t now) {
    if (!hardware->configured) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!output.enable || !isfinite(output.left) || !isfinite(output.right)) {
        rover_hardware_disable(hardware);
        // nSLEEP low gates outputs even if a later PWM driver call fails.
        for (int i = 0; i < 4; ++i) {
            const ledc_channel_t ch = (ledc_channel_t)(i);
            ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0), "motor", "zero duty");
            ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch), "motor", "zero update");
        }
        return ESP_OK;
    }
    if (!hardware->awake) {
        ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)(hardware->pins.sleep), 1), "motor", "wake");
        hardware->awake = true;
        hardware->awake_at = now;
        return ESP_OK;
    }
    if (now - hardware->awake_at < 1000) {
        return ESP_OK; // DRV8833 wake delay, without blocking control.
    }
    double values[] = {output.left, output.right};
#ifdef CONFIG_ROVER_INVERT_LEFT_MOTOR
    values[0] = -values[0];
#endif
#ifdef CONFIG_ROVER_INVERT_RIGHT_MOTOR
    values[1] = -values[1];
#endif
    for (int wheel = 0; wheel < 2; ++wheel) {
        for (int input = 0; input < 2; ++input) {
            const ledc_channel_t channel = (ledc_channel_t)(wheel * 2 + input);
            ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0), "motor", "coast");
            ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel), "motor",
                                "coast update");
        }
        const ledc_channel_t channel = (ledc_channel_t)(wheel * 2 + (values[wheel] < 0 ? 1 : 0));
        const uint32_t duty = (uint32_t)round(fmin(fabs(values[wheel]), 1.0) * 1023);
        ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty), "motor", "drive");
        ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel), "motor",
                            "drive update");
    }
    return ESP_OK;
}
