from dataclasses import dataclass


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
