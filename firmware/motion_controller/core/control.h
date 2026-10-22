// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { ROVER_DISARMED, ROVER_READY, ROVER_MOVING, ROVER_FAULT } RoverState;
typedef enum {
    ROVER_FAULT_NONE,
    ROVER_FAULT_BUMPER,
    ROVER_FAULT_COMMAND_EXPIRED,
    ROVER_FAULT_LINK_LOST,
    ROVER_FAULT_LOOP_OVERRUN,
    ROVER_FAULT_CONFIGURATION,
    ROVER_FAULT_HARDWARE
} RoverFault;

typedef struct {
    bool wiring_confirmed;
    double wheel_radius_m, track_m, counts_per_revolution;
    double kp, ki, feedforward;
    double max_speed_mps, max_acceleration_mps2, max_duty;
    int64_t max_tick_gap_us;
    bool left_quadrature, right_quadrature;
} RoverConfig;

typedef struct {
    RoverState state;
    RoverFault fault;
    bool bumper;
    int64_t monotonic_us;
    double left_mps, right_mps, left_distance_m, right_distance_m;
    int64_t left_counts, right_counts, raw_left_counts, raw_right_counts;
    double x_m, y_m, heading_rad, left_duty, right_duty;
    double left_target_mps, right_target_mps;
    int64_t last_motion_sequence, max_tick_interval_us;
} RoverSnapshot;

typedef struct {
    double left, right;
    bool enable;
} RoverOutput;
typedef struct {
    double ramp, integral;
    int direction;
} RoverWheel;

/* Caller owns this state. The ESP32 host protects shared access with a mutex.
 * No allocation or peripheral access happens in this portable control core. */
typedef struct {
    RoverConfig config;
    RoverSnapshot snapshot;
    RoverWheel left, right;
    bool connected, sampled;
    int64_t deadline_us;
    double requested_left, requested_right;
} RoverController;

RoverConfig rover_config_defaults(void);
bool rover_config_calibrated(const RoverConfig *config);
const char *rover_state_name(RoverState state);
const char *rover_fault_name(RoverFault fault);
void rover_controller_init(RoverController *controller, RoverConfig config);
void rover_controller_connect(RoverController *controller);
void rover_controller_disconnect(RoverController *controller);
const char *rover_controller_arm(RoverController *controller, int64_t now);
const char *rover_controller_reset(RoverController *controller, int64_t now);
const char *rover_controller_drive(RoverController *controller, double left, double right,
                                   int lease_ms, int64_t sequence, int64_t now);
void rover_controller_stop(RoverController *controller);
void rover_controller_trip(RoverController *controller, RoverFault fault);
/* Single-channel edges infer direction from the last nonzero drive output.
 * Quadrature edges already carry direction. */
RoverOutput rover_controller_tick(RoverController *controller, int64_t now, int64_t left_edges,
                                  int64_t right_edges, bool bumper);
bool rover_valid_pins(const int *pins, unsigned count, unsigned output_count);
