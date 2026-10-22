// SPDX-License-Identifier: GPL-3.0-only
#include "control.h"
#include <math.h>
#include <string.h>

static double clamp(double value, double low, double high) {
    return value < low ? low : value > high ? high : value;
}
static const double pi = 3.14159265358979323846;
static int sign(double x) {
    return (x > 0) - (x < 0);
}
static bool positive(double x) {
    return isfinite(x) && x > 0;
}
const char *rover_state_name(RoverState s) {
    switch (s) {
    case ROVER_DISARMED:
        return "disarmed";
    case ROVER_READY:
        return "ready";
    case ROVER_MOVING:
        return "moving";
    case ROVER_FAULT:
        return "fault";
    }
    return "fault";
}
const char *rover_fault_name(RoverFault f) {
    switch (f) {
    case ROVER_FAULT_NONE:
        return NULL;
    case ROVER_FAULT_BUMPER:
        return "bumper";
    case ROVER_FAULT_COMMAND_EXPIRED:
        return "command_expired";
    case ROVER_FAULT_LINK_LOST:
        return "link_lost";
    case ROVER_FAULT_LOOP_OVERRUN:
        return "loop_overrun";
    case ROVER_FAULT_CONFIGURATION:
        return "configuration";
    case ROVER_FAULT_HARDWARE:
        return "hardware";
    }
    return "hardware";
}
bool rover_config_calibrated(const RoverConfig *config) {
    return positive(config->wheel_radius_m) && config->wheel_radius_m <= 1 &&
           positive(config->track_m) && config->track_m <= 10 &&
           positive(config->counts_per_revolution) && config->counts_per_revolution <= 1e9 &&
           config->left_quadrature == config->right_quadrature && positive(config->kp) &&
           config->kp <= 1000 && isfinite(config->ki) && config->ki >= 0 && config->ki <= 1000 &&
           isfinite(config->feedforward) && config->feedforward >= 0 &&
           config->feedforward <= 1000 && positive(config->max_speed_mps) &&
           config->max_speed_mps <= 2 && positive(config->max_acceleration_mps2) &&
           config->max_acceleration_mps2 <= 5 && positive(config->max_duty) &&
           config->max_duty <= 1 && config->max_tick_gap_us >= 10000;
}
bool rover_valid_pins(const int *pins, unsigned count, unsigned output_count) {
    const int outputs[] = {4, 13, 14, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
    for (unsigned i = 0; i < count; ++i) {
        bool valid = false;
        for (size_t j = 0; j < sizeof(outputs) / sizeof(outputs[0]); ++j) {
            if (pins[i] == outputs[j]) {
                valid = true;
            }
        }
        if (i >= output_count &&
            (pins[i] == 34 || pins[i] == 35 || pins[i] == 36 || pins[i] == 39)) {
            valid = true;
        }
        if (!valid) {
            return false;
        }
        for (unsigned j = 0; j < i; ++j) {
            if (pins[i] == pins[j]) {
                return false;
            }
        }
    }
    return true;
}
RoverConfig rover_config_defaults(void) {
    return (RoverConfig){.max_speed_mps = 0.2,
                         .max_acceleration_mps2 = 0.3,
                         .max_duty = 0.5,
                         .max_tick_gap_us = 30000};
}
void rover_controller_init(RoverController *controller, RoverConfig config) {
    *controller = (RoverController){0};
    controller->config = config;
    controller->snapshot.last_motion_sequence = -1;
}
static void zero(RoverController *controller) {
    controller->requested_left = controller->requested_right = 0;
    controller->left.ramp = controller->right.ramp = controller->left.integral =
        controller->right.integral = 0;
    controller->snapshot.left_duty = controller->snapshot.right_duty = 0;
    controller->snapshot.left_target_mps = controller->snapshot.right_target_mps = 0;
    controller->deadline_us = 0;
    // Keep last direction to account for coasting. It is still only an inference.
}
void rover_controller_stop(RoverController *controller) {
    zero(controller);
    if (controller->snapshot.fault == ROVER_FAULT_NONE) {
        controller->snapshot.state = ROVER_DISARMED;
    }
}
void rover_controller_trip(RoverController *controller, RoverFault f) {
    zero(controller);
    if (controller->snapshot.fault == ROVER_FAULT_NONE) {
        controller->snapshot.fault = f;
    }
    controller->snapshot.state = ROVER_FAULT;
}
void rover_controller_connect(RoverController *controller) {
    controller->connected = true;
    rover_controller_stop(controller);
    controller->snapshot.last_motion_sequence = -1;
}
void rover_controller_disconnect(RoverController *controller) {
    controller->connected = false;
    rover_controller_trip(controller, ROVER_FAULT_LINK_LOST);
}
static const char *permits_arming(const RoverController *controller, int64_t now) {
    if (!controller->config.wiring_confirmed) {
        return "wiring_unconfirmed";
    }
    if (!rover_config_calibrated(&controller->config)) {
        return "calibration_invalid";
    }
    if (!controller->connected) {
        return "link_lost";
    }
    if (!controller->sampled || now < controller->snapshot.monotonic_us ||
        now - controller->snapshot.monotonic_us > controller->config.max_tick_gap_us) {
        return "sensors_stale";
    }
    if (controller->snapshot.bumper) {
        return "bumper";
    }
    return NULL;
}
const char *rover_controller_arm(RoverController *controller, int64_t now) {
    const char *error = permits_arming(controller, now);
    if (error) {
        return error;
    }
    if (controller->snapshot.fault != ROVER_FAULT_NONE) {
        return "fault_latched";
    }
    if (controller->snapshot.state != ROVER_DISARMED) {
        return "already_armed";
    }
    zero(controller);
    controller->snapshot.state = ROVER_READY;
    return NULL;
}
const char *rover_controller_reset(RoverController *controller, int64_t now) {
    if (controller->snapshot.fault == ROVER_FAULT_HARDWARE ||
        controller->snapshot.fault == ROVER_FAULT_CONFIGURATION) {
        return "restart_required";
    }
    const char *error = permits_arming(controller, now);
    if (error) {
        return error;
    }
    zero(controller);
    controller->snapshot.fault = ROVER_FAULT_NONE;
    controller->snapshot.state = ROVER_DISARMED;
    return NULL;
}
const char *rover_controller_drive(RoverController *controller, double left, double right,
                                   int lease, int64_t sequence, int64_t now) {
    if (!isfinite(left) || !isfinite(right) || fabs(left) > controller->config.max_speed_mps ||
        fabs(right) > controller->config.max_speed_mps || lease < 1 || lease > 300) {
        return "invalid_drive";
    }
    const char *error = permits_arming(controller, now);
    if (error) {
        return error;
    }
    if (controller->snapshot.state != ROVER_READY && controller->snapshot.state != ROVER_MOVING) {
        return "not_armed";
    }
    if (controller->deadline_us && now >= controller->deadline_us) {
        rover_controller_trip(controller, ROVER_FAULT_COMMAND_EXPIRED);
        return "command_expired";
    }
    if (left == 0 && right == 0) {
        zero(controller);
        controller->snapshot.state = ROVER_READY;
    } else {
        controller->requested_left = left;
        controller->requested_right = right;
        controller->deadline_us = now + (int64_t)(lease) * 1000;
        controller->snapshot.state = ROVER_MOVING;
    }
    controller->snapshot.last_motion_sequence = sequence;
    return NULL;
}
static double wheel_control(RoverController *controller, RoverWheel *wheel, double requested,
                            double measured, double dt) {
    const double step = controller->config.max_acceleration_mps2 * dt;
    wheel->ramp += clamp(requested - wheel->ramp, -step, step);
    if (fabs(wheel->ramp) < 1e-9) {
        wheel->ramp = 0;
        wheel->integral = 0;
        return 0;
    }
    const double error = wheel->ramp - measured;
    const double proposed = wheel->integral + controller->config.ki * error * dt;
    // Do not command active reversal merely to correct overspeed. A zero input
    // coasts. This also avoids claiming single-channel direction sensing.
    const double low = wheel->ramp > 0 ? 0 : -controller->config.max_duty;
    const double high = wheel->ramp > 0 ? controller->config.max_duty : 0;
    const double base =
        controller->config.feedforward * wheel->ramp + controller->config.kp * error;
    const double raw = base + proposed;
    if ((raw >= low && raw <= high) || (raw > high && error < 0) || (raw < low && error > 0)) {
        wheel->integral =
            clamp(proposed, -controller->config.max_duty, controller->config.max_duty);
    }
    const double duty = clamp(base + wheel->integral, low, high);
    if (duty != 0) {
        wheel->direction = sign(duty);
    }
    return duty;
}
RoverOutput rover_controller_tick(RoverController *controller, int64_t now, int64_t left_edges,
                                  int64_t right_edges, bool bumper) {
    const int64_t elapsed = controller->sampled ? now - controller->snapshot.monotonic_us : 0;
    const bool time_bad =
        controller->sampled && (elapsed <= 0 || elapsed > controller->config.max_tick_gap_us);
    controller->snapshot.bumper = bumper;
    controller->snapshot.monotonic_us = now;
    controller->sampled = true;
    if (elapsed > controller->snapshot.max_tick_interval_us) {
        controller->snapshot.max_tick_interval_us = elapsed;
    }
    // Interlocks precede feedback/control and cannot be cancelled by a new drive.
    if (bumper) {
        rover_controller_trip(controller, ROVER_FAULT_BUMPER);
    }
    if (time_bad) {
        rover_controller_trip(controller, ROVER_FAULT_LOOP_OVERRUN);
    }
    if (controller->snapshot.state == ROVER_MOVING &&
        (!controller->connected || now >= controller->deadline_us)) {
        rover_controller_trip(controller, controller->connected ? ROVER_FAULT_COMMAND_EXPIRED
                                                                : ROVER_FAULT_LINK_LOST);
    }
    controller->snapshot.raw_left_counts += left_edges;
    controller->snapshot.raw_right_counts += right_edges;
    const int64_t dlc =
        controller->config.left_quadrature ? left_edges : left_edges * controller->left.direction;
    const int64_t drc = controller->config.right_quadrature
                            ? right_edges
                            : right_edges * controller->right.direction;
    controller->snapshot.left_counts += dlc;
    controller->snapshot.right_counts += drc;
    if (rover_config_calibrated(&controller->config)) {
        const double metres_per_count =
            2 * pi * controller->config.wheel_radius_m / controller->config.counts_per_revolution;
        const double dl = dlc * metres_per_count, dr = drc * metres_per_count;
        controller->snapshot.left_distance_m += dl;
        controller->snapshot.right_distance_m += dr;
        const double dh = (dr - dl) / controller->config.track_m;
        const double distance = (dl + dr) / 2;
        // Exact differential-drive arc integration, stable at small angles.
        const double scale = fabs(dh) < 1e-9 ? 1 : sin(dh / 2) / (dh / 2);
        controller->snapshot.x_m +=
            distance * scale * cos(controller->snapshot.heading_rad + dh / 2);
        controller->snapshot.y_m +=
            distance * scale * sin(controller->snapshot.heading_rad + dh / 2);
        controller->snapshot.heading_rad = remainder(controller->snapshot.heading_rad + dh, 2 * pi);
        if (elapsed > 0) {
            controller->snapshot.left_mps = dl / (elapsed / 1e6);
            controller->snapshot.right_mps = dr / (elapsed / 1e6);
        }
    }
    if (controller->snapshot.state != ROVER_MOVING || elapsed <= 0 ||
        !rover_config_calibrated(&controller->config)) {
        return (RoverOutput){0};
    }
    controller->snapshot.left_duty =
        wheel_control(controller, &controller->left, controller->requested_left,
                      controller->snapshot.left_mps, elapsed / 1e6);
    controller->snapshot.right_duty =
        wheel_control(controller, &controller->right, controller->requested_right,
                      controller->snapshot.right_mps, elapsed / 1e6);
    controller->snapshot.left_target_mps = controller->left.ramp;
    controller->snapshot.right_target_mps = controller->right.ramp;
    return (RoverOutput){controller->snapshot.left_duty, controller->snapshot.right_duty, true};
}
