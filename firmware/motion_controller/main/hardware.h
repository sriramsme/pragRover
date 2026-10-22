// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "control.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int left_in1, left_in2, right_in1, right_in2, sleep;
    int left_a, right_a, bumper_left, bumper_right, left_b, right_b;
} RoverPins;
typedef struct {
    RoverPins pins;
    pcnt_unit_handle_t left, right;
    uint32_t previous_left, previous_right;
    bool configured, awake;
    int64_t awake_at;
} RoverHardware;

bool rover_pins_valid(const RoverPins *pins);
esp_err_t rover_hardware_initialize(RoverHardware *hardware, RoverPins pins);
bool rover_hardware_bumper(const RoverHardware *hardware);
esp_err_t rover_hardware_edges(RoverHardware *hardware, int64_t *left, int64_t *right);
esp_err_t rover_hardware_apply(RoverHardware *hardware, RoverOutput output, int64_t now);
void rover_hardware_disable(RoverHardware *hardware);
