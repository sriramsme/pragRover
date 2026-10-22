import math

import pytest

from prag_rover.controller import SimulatedController, State


class Clock:
    def __init__(self):
        self.at = 0.0

    def __call__(self):
        return self.at


def test_expiry_integrates_only_until_deadline_and_latches_fault():
    clock = Clock()
    controller = SimulatedController(now=clock)
    controller.arm()
    controller.drive(0.1, 0.1, sequence=1, expires_at=0.3)
    clock.at = 10
    state = controller.snapshot()
    assert state["x_m"] == pytest.approx(0.03)
    assert state["state"] == State.FAULT
    assert state["fault"] == "command_timeout"
    assert state["left_mps"] == state["right_mps"] == 0
    with pytest.raises(ValueError):
        controller.arm()
    controller.reset()
    controller.arm()


def test_sequence_deadline_limits_bumper_and_no_automatic_restart():
    clock = Clock()
    controller = SimulatedController(now=clock)
    with pytest.raises(ValueError):
        controller.drive(0.1, 0.1, sequence=1, expires_at=0.3)
    controller.arm()
    for left, seq, deadline in (
        (True, 1, 0.3),
        (math.nan, 1, 0.3),
        (0.2, 1, 0.3),
        (0.1, 1, 1),
        (0.1, 1, -1),
    ):
        with pytest.raises(ValueError):
            controller.drive(left, 0, sequence=seq, expires_at=deadline)
    controller.drive(0.1, -0.1, sequence=1, expires_at=0.3)
    with pytest.raises(ValueError):
        controller.drive(0.1, 0.1, sequence=1, expires_at=0.3)
    clock.at = 0.1
    controller.set_bumper(True)
    assert controller.snapshot()["heading_rad"] < 0
    assert controller.fault == "bumper"
    with pytest.raises(ValueError):
        controller.reset()
    controller.set_bumper(False)
    assert controller.state == State.FAULT
    controller.reset()
    assert controller.state == State.DISARMED


def test_arc_integration_and_backwards_clock():
    clock = Clock()
    controller = SimulatedController(now=clock)
    controller.arm()
    controller.drive(0, 0.1, sequence=1, expires_at=0.5)
    clock.at = 0.2
    state = controller.snapshot()
    theta = 0.02 / controller.geometry.track_m
    radius = controller.geometry.track_m / 2
    assert state["x_m"] == pytest.approx(radius * math.sin(theta))
    assert state["y_m"] == pytest.approx(radius * (1 - math.cos(theta)))
    clock.at = 0
    with pytest.raises(ValueError):
        controller.tick()
