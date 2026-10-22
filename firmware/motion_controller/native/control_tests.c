// SPDX-License-Identifier: GPL-3.0-only
#include "control.h"
#include "session.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static RoverConfig calibration(void) {
    // Synthetic geometry/gains for tests only, never a physical calibration.
    RoverConfig c = rover_config_defaults();
    c.wiring_confirmed = true;
    c.wheel_radius_m = 0.03;
    c.track_m = 0.12;
    c.counts_per_revolution = 100;
    c.kp = 2;
    c.ki = 1;
    c.feedforward = 1;
    c.max_speed_mps = 0.3;
    c.max_acceleration_mps2 = 1;
    c.max_duty = 0.5;
    c.left_quadrature = c.right_quadrature = true;
    return c;
}
static void assert_close(double actual, double expected) {
    assert(fabs(actual - expected) < 1e-9);
}
static void ready(RoverController *c) {
    rover_controller_connect(c);
    rover_controller_tick(c, 10000, 0, 0, false);
    assert(!rover_controller_arm(c, 10000));
}
static void startup_and_faults(void) {
    RoverController boardless;
    rover_controller_init(&boardless, rover_config_defaults());
    rover_controller_connect(&boardless);
    rover_controller_tick(&boardless, 10000, 0, 0, false);
    assert(rover_controller_arm(&boardless, 10000));
    assert(!rover_controller_tick(&boardless, 20000, 1, 2, false).enable);
    assert(boardless.snapshot.raw_left_counts == 1 && boardless.snapshot.raw_right_counts == 2);
    assert(boardless.snapshot.left_counts == 0);
    RoverConfig cfg = calibration();
    cfg.kp = NAN;
    assert(!rover_config_calibrated(&cfg));
    cfg = calibration();
    cfg.counts_per_revolution = 0;
    assert(!rover_config_calibrated(&cfg));
    RoverController c;
    rover_controller_init(&c, calibration());
    ready(&c);
    assert(!rover_controller_drive(&c, 0.2, 0.2, 100, 1, 10000));
    assert(rover_controller_tick(&c, 20000, 0, 0, false).enable);
    assert(!rover_controller_tick(&c, 30000, 0, 0, true).enable);
    assert(c.snapshot.fault == ROVER_FAULT_BUMPER);
    assert(rover_controller_reset(&c, 30000));
    rover_controller_tick(&c, 40000, 0, 0, false);
    assert(!rover_controller_reset(&c, 40000));
    assert(c.snapshot.state == ROVER_DISARMED);
    assert(rover_controller_drive(&c, 0.1, 0.1, 100, 2, 40000));
    assert(!rover_controller_arm(&c, 40000));
    assert(!rover_controller_drive(&c, 0.2, 0.2, 100, 3, 40000));
    rover_controller_disconnect(&c);
    assert(!rover_controller_tick(&c, 50000, 0, 0, false).enable);
    assert(c.snapshot.fault == ROVER_FAULT_LINK_LOST);
    rover_controller_connect(&c);
    assert(rover_controller_arm(&c, 50000));
    assert(!rover_controller_reset(&c, 50000));
    assert(!rover_controller_arm(&c, 50000));
    assert(rover_controller_arm(&c, 50000)); // Repeated arm never clears/restarts motion.
    rover_controller_trip(&c, ROVER_FAULT_HARDWARE);
    assert(rover_controller_reset(&c, 50000));
}
static void deadlines_and_zero(void) {
    RoverController c;
    rover_controller_init(&c, calibration());
    ready(&c);
    assert(rover_controller_drive(&c, 0.2, 0.2, 301, 1, 10000));
    assert(rover_controller_drive(&c, INFINITY, 0.1, 100, 1, 10000));
    assert(!rover_controller_drive(&c, 0.2, 0.2, 30, 1, 10000));
    rover_controller_tick(&c, 20000, 0, 0, false);
    rover_controller_tick(&c, 30000, 0, 0, false);
    assert(!rover_controller_tick(&c, 40000, 0, 0, false).enable);
    assert(c.snapshot.fault == ROVER_FAULT_COMMAND_EXPIRED);
    assert(!rover_controller_reset(&c, 40000));
    assert(!rover_controller_arm(&c, 40000));
    assert(!rover_controller_drive(&c, 0.2, 0.2, 30, 2, 40000));
    // A renewal at/after expiry cannot rescue a missed deadline between ticks.
    assert(rover_controller_drive(&c, 0.2, 0.2, 30, 3, 70000));
    assert(c.snapshot.fault == ROVER_FAULT_COMMAND_EXPIRED);
    rover_controller_tick(&c, 70000, 0, 0, false);
    assert(!rover_controller_reset(&c, 70000));
    assert(!rover_controller_arm(&c, 70000));
    assert(!rover_controller_drive(&c, 0.1, 0.1, 100, 4, 70000));
    assert(!rover_controller_drive(&c, 0, 0, 10, 5, 80000));
    assert(c.snapshot.state == ROVER_READY);
    for (int t = 80000; t <= 180000; t += 10000) {
        assert(!rover_controller_tick(&c, t, 0, 0, false).enable);
    }
    assert(c.snapshot.fault == ROVER_FAULT_NONE);
    rover_controller_stop(&c);
    assert(c.snapshot.state == ROVER_DISARMED);
}
static void loop_timing(void) {
    RoverController c;
    rover_controller_init(&c, calibration());
    ready(&c);
    assert(!rover_controller_drive(&c, 0.1, 0.1, 300, 1, 10000));
    assert(!rover_controller_tick(&c, 41000, 0, 0, false).enable);
    assert(c.snapshot.fault == ROVER_FAULT_LOOP_OVERRUN);
    assert(c.snapshot.max_tick_interval_us == 31000);
    rover_controller_stop(&c);
    assert(c.snapshot.fault == ROVER_FAULT_LOOP_OVERRUN);
    RoverController d;
    rover_controller_init(&d, calibration());
    ready(&d);
    rover_controller_tick(&d, 10000, 0, 0, false);
    assert(d.snapshot.fault == ROVER_FAULT_LOOP_OVERRUN);
    RoverController stale;
    rover_controller_init(&stale, calibration());
    ready(&stale);
    rover_controller_stop(&stale);
    assert(rover_controller_arm(&stale, 50000));
}
static void feedback_and_odometry(void) {
    RoverController c;
    rover_controller_init(&c, calibration());
    ready(&c);
    assert(!rover_controller_drive(&c, 0.2, 0.2, 300, 1, 10000));
    const RoverOutput first = rover_controller_tick(&c, 20000, 0, 0, false);
    assert_close(c.snapshot.left_target_mps, 0.01);
    assert(first.left > 0 && first.left <= 0.5);
    rover_controller_tick(&c, 30000, 1, 1, false);
    const double metres_per_count = 2 * 3.14159265358979323846 * 0.03 / 100;
    assert_close(c.snapshot.x_m, metres_per_count);
    assert_close(c.snapshot.y_m, 0);
    assert_close(c.snapshot.left_mps, metres_per_count / 0.01);
    rover_controller_tick(&c, 40000, -1, 1, false);
    assert_close(c.snapshot.heading_rad, 2 * metres_per_count / 0.12);
    assert_close(c.snapshot.x_m, metres_per_count);
    // PI saturation stays bounded; fresh drive leases keep this a control test.
    for (int i = 5; i < 300; ++i) {
        const int64_t now = (int64_t)(i) * 10000;
        assert(!rover_controller_drive(&c, 0.3, 0.3, 100, i, now));
        const RoverOutput out = rover_controller_tick(&c, now, 0, 0, false);
        assert(out.left <= 0.5 && out.left >= 0);
    }
    assert(!rover_controller_drive(&c, -0.1, -0.1, 100, 301, 3000000));
    rover_controller_tick(&c, 3000000, 0, 0, false);
    assert(c.snapshot.left_target_mps > 0); // Direction change ramps through zero.
}
static void antiwindup_recovery(void) {
    RoverConfig cfg = calibration();
    cfg.kp = 20;
    cfg.ki = 100;
    cfg.feedforward = 0;
    cfg.max_acceleration_mps2 = 5;
    RoverController c;
    rover_controller_init(&c, cfg);
    ready(&c);
    for (int i = 2; i <= 100; ++i) {
        assert(!rover_controller_drive(&c, 0.2, 0.2, 100, i, i * 10000));
        rover_controller_tick(&c, i * 10000, 0, 0, false);
    }
    assert_close(c.snapshot.left_duty, 0.5);
    // No encoder movement: prolonged saturation must not retain a huge integral.
    for (int i = 101; i <= 104; ++i) {
        assert(!rover_controller_drive(&c, 0.02, 0.02, 100, i, i * 10000));
        rover_controller_tick(&c, i * 10000, 0, 0, false);
    }
    assert(c.snapshot.left_duty < 0.5);
}
static void single_channel_direction(void) {
    RoverConfig cfg = calibration();
    cfg.left_quadrature = cfg.right_quadrature = false;
    RoverController c;
    rover_controller_init(&c, cfg);
    ready(&c);
    assert(!rover_controller_drive(&c, -0.1, 0.1, 100, 1, 10000));
    rover_controller_tick(&c, 20000, 0, 0, false);
    rover_controller_tick(&c, 30000, 2, 2, false);
    assert(c.snapshot.left_counts == -2 && c.snapshot.right_counts == 2);
    rover_controller_stop(&c);
    rover_controller_tick(&c, 40000, 1, 1, false); // Account for coast using retained direction.
    assert(c.snapshot.left_counts == -3 && c.snapshot.right_counts == 3);
}
static void tickets_and_pins(void) {
    RoverSessionGate g;
    rover_session_reset(&g);
    rover_session_issue(&g, 1000);
    assert(rover_session_authorize(&g, 0, 251000, true, false,
                                   false)); // Exact receiver deadline is expired.
    assert(!rover_session_authorize(&g, 0, 251000, false, false, true)); // Status renews ticket.
    rover_session_issue(&g, 251000);
    assert(!rover_session_authorize(&g, 1, 252000, true, false, false));
    assert(rover_session_authorize(&g, 2, 253000, true, false, false)); // One-use ticket.
    assert(!rover_session_authorize(&g, 2, 1000000, false, true,
                                    false)); // Stop bypasses ticket expiry.
    assert(rover_session_authorize(&g, 2, 1000001, true, true, false)); // But not sequence replay.
    rover_session_issue(&g, 1000001);
    assert(rover_session_authorize(&g, 3, 1000002, false, false, false));
    const int good[] = {4, 13, 14, 18, 19, 21, 22, 34, 35};
    assert(rover_valid_pins(good, 9, 5));
    const int duplicate[] = {4, 4};
    assert(!rover_valid_pins(duplicate, 2, 2));
    const int strap[] = {12};
    assert(!rover_valid_pins(strap, 1, 1));
    const int flash[] = {6};
    assert(!rover_valid_pins(flash, 1, 1));
    const int input_only[] = {34};
    assert(!rover_valid_pins(input_only, 1, 1));
}
int main(void) {
    startup_and_faults();
    deadlines_and_zero();
    loop_timing();
    feedback_and_odometry();
    antiwindup_recovery();
    single_channel_direction();
    tickets_and_pins();
    puts("All native control, fault, odometry, ticket and GPIO assertions passed");
    return 0;
}
