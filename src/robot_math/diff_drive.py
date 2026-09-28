from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from .se2 import SE2

STRAIGHT_LINE_TOLERANCE = 1e-6


def motion_step(v: float, omega: float, dt: float) -> SE2:
    """Pose change after driving at constant v and omega for dt seconds.

    The step is in the robot's frame at the start of the step, so the new
    world pose is pose @ motion_step(v, omega, dt). Units: v in m/s, omega
    in rad/s (counter-clockwise positive), dt in s. When |omega| is below
    STRAIGHT_LINE_TOLERANCE, the straight-line step replaces the arc to
    avoid dividing by (almost) zero.
    """

    if dt < 0:
        raise ValueError(f"Expected non-negative dt, got {dt}")
    if abs(omega) < STRAIGHT_LINE_TOLERANCE:
        return SE2(x=float(v * dt), y=0.0, theta=omega * dt)
    dtheta = omega * dt
    R = v / omega
    x = float(R * np.sin(dtheta))
    y = float(R * (1 - np.cos(dtheta)))
    return SE2(x, y, dtheta)


@dataclass(frozen=True)
class DiffDrive:
    """A two-wheeled differential-drive robot.

    Wheel base in metres, wheel speeds in m/s,
    omega in rad/s (counter-clockwise positive).
    """

    wheel_base: float

    def __post_init__(self) -> None:
        if self.wheel_base <= 0:
            raise ValueError(f"Expected positive wheel base, got {self.wheel_base}")

    def body_velocity(self, v_left: float, v_right: float) -> tuple[float, float]:
        """Wheel speeds to body motion (odometry): (v_left, v_right) -> (v, omega)"""
        v = float((v_left + v_right) / 2)
        omega = float((v_right - v_left) / self.wheel_base)
        return (v, omega)

    def wheel_speeds(self, v: float, omega: float) -> tuple[float, float]:
        """Body motion to wheel speeds: (v, omega) -> (v_left, v_right)"""
        v_left = float(v - omega * self.wheel_base / 2)
        v_right = float(v + omega * self.wheel_base / 2)
        return (v_left, v_right)

    def dead_reckon(
        self, start: SE2, readings: Iterable[tuple[float, float, float]]
    ) -> list[SE2]:
        """Integrate wheel readings into a path, starting from start.

        Each reading is (v_left, v_right, dt): wheel speeds in m/s and how long,
        in s, they were held. Returns start followed by the pose after each
        reading. Errors in the readings accumulate; nothing here corrects them.
        """

        path = [start]
        for v_left, v_right, dt in readings:
            v, omega = self.body_velocity(v_left, v_right)
            new_pose = path[-1] @ motion_step(v, omega, dt)
            path.append(new_pose)
        return path
