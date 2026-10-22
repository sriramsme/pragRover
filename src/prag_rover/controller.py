"""Ideal differential-drive simulation with independently expiring commands.

No motor dynamics, PI controller, physical encoder, or hardware timing is modeled.
The monotonic command deadline is meaningful only within this simulated clock.
"""

import math
import time
from dataclasses import dataclass
from enum import StrEnum

from prag_rover.protocol import number


class State(StrEnum):
    DISARMED = "disarmed"
    READY = "ready"
    MOVING = "moving"
    FAULT = "fault"


@dataclass(frozen=True)
class Geometry:
    wheel_radius_m: float = 0.0325
    track_m: float = 0.14
    counts_per_revolution: float = 20

    def __post_init__(self):
        number(self.wheel_radius_m, name="wheel radius", minimum=0.001, maximum=1)
        number(self.track_m, name="track", minimum=0.01, maximum=2)
        number(
            self.counts_per_revolution,
            name="encoder counts per revolution",
            minimum=0.001,
            maximum=10000000,
        )


class SimulatedController:
    simulated = True
    MAX_SPEED = 0.15
    MAX_LEASE = 0.5

    def __init__(self, *, now=time.monotonic, geometry=None):
        self.now = now
        self.geometry = geometry or Geometry()
        self.state = State.DISARMED
        self.fault = None
        self.bumper = False
        self.left = self.right = 0.0
        self.left_distance = self.right_distance = 0.0
        self.x = self.y = self.heading = 0.0
        self.deadline = None
        self.sequence = 0
        self._updated = now()

    def tick(self):
        at = self.now()
        if at < self._updated:
            raise ValueError("Controller monotonic clock moved backwards")
        until = min(at, self.deadline) if self.deadline is not None else at
        dt = max(0.0, until - self._updated)
        dl, dr = self.left * dt, self.right * dt
        self.left_distance += dl
        self.right_distance += dr
        distance = (dl + dr) / 2
        angle = (dr - dl) / self.geometry.track_m
        if abs(angle) < 1e-12:
            self.x += distance * math.cos(self.heading)
            self.y += distance * math.sin(self.heading)
        else:
            radius = distance / angle
            self.x += radius * (math.sin(self.heading + angle) - math.sin(self.heading))
            self.y -= radius * (math.cos(self.heading + angle) - math.cos(self.heading))
        self.heading += angle
        self._updated = at
        if self.deadline is not None and at >= self.deadline:
            self.stop("command_timeout")

    def arm(self):
        self.tick()
        if self.state == State.FAULT or self.bumper:
            raise ValueError(
                "Clear the bumper and reset the latched fault before arming"
            )
        if self.state != State.DISARMED:
            raise ValueError("Controller is already armed")
        self.state = State.READY

    def drive(self, left, right, *, sequence, expires_at):
        self.tick()
        if self.state not in (State.READY, State.MOVING):
            raise ValueError("Arm the controller before motion")
        if type(sequence) is not int or sequence <= self.sequence:
            raise ValueError("Motion sequence must advance")
        left = number(
            left, name="left speed", minimum=-self.MAX_SPEED, maximum=self.MAX_SPEED
        )
        right = number(
            right, name="right speed", minimum=-self.MAX_SPEED, maximum=self.MAX_SPEED
        )
        remaining = expires_at - self.now()
        number(
            remaining, name="command lifetime", minimum=0.000001, maximum=self.MAX_LEASE
        )
        self.sequence = sequence
        self.left, self.right, self.deadline = left, right, expires_at
        self.state = State.MOVING if left or right else State.READY

    def stop(self, fault=None):
        # Called after tick by consumers; tick itself uses this at expiration.
        self.left = self.right = 0.0
        self.deadline = None
        if fault is not None:
            self.state, self.fault = State.FAULT, fault
        elif self.state != State.FAULT:
            self.state = State.DISARMED

    def reset(self):
        self.tick()
        if self.bumper:
            raise ValueError("Release the bumper before resetting")
        self.stop()
        self.fault = None
        self.state = State.DISARMED

    def finish_segment(self):
        self.tick()
        if self.state == State.FAULT:
            raise ValueError(f"Controller fault: {self.fault}")
        self.left = self.right = 0.0
        self.deadline = None
        self.state = State.READY

    def set_bumper(self, pressed):
        if type(pressed) is not bool:
            raise ValueError("Bumper state must be boolean")
        self.tick()
        self.bumper = pressed
        if pressed:
            self.stop("bumper")

    def snapshot(self):
        self.tick()
        count_scale = self.geometry.counts_per_revolution / (
            2 * math.pi * self.geometry.wheel_radius_m
        )
        return {
            "simulated": True,
            "state": self.state.value,
            "fault": self.fault,
            "bumper": self.bumper,
            "monotonic_s": self.now(),
            "left_mps": self.left,
            "right_mps": self.right,
            "left_distance_m": self.left_distance,
            "right_distance_m": self.right_distance,
            "left_counts": round(self.left_distance * count_scale),
            "right_counts": round(self.right_distance * count_scale),
            "x_m": self.x,
            "y_m": self.y,
            "heading_rad": self.heading,
            "last_motion_sequence": self.sequence,
        }
