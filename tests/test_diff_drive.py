import pytest

from robot_math import DiffDrive


def test_body_velocity_equal_wheels_drives_straight() -> None:
    drive = DiffDrive(0.3)
    result = drive.body_velocity(0.5, 0.5)
    assert result == pytest.approx((0.5, 0.0))


def test_body_velocity_opposite_wheels_spins_in_place() -> None:
    drive = DiffDrive(0.3)
    result = drive.body_velocity(-0.5, 0.5)
    assert result == pytest.approx((0.0, 1.0 / 0.3))


def test_body_velocity_curves_left() -> None:
    drive = DiffDrive(0.3)
    result = drive.body_velocity(0.2, 0.4)
    assert result == pytest.approx((0.3, 2 / 3))


def test_wheel_speeds_turning_left_speeds_up_right_wheel() -> None:
    drive = DiffDrive(0.3)
    result = drive.wheel_speeds(0.2, 0.5)
    assert result == pytest.approx((0.125, 0.275))


def test_body_velocity_undoes_wheel_speeds() -> None:
    drive = DiffDrive(0.287)
    v_left, v_right = drive.wheel_speeds(0.43, -1.7)
    result = drive.body_velocity(v_left, v_right)
    assert result == pytest.approx((0.43, -1.7))


def test_rejects_non_positive_wheel_base() -> None:
    with pytest.raises(ValueError, match="Expected positive"):
        DiffDrive(0)
    with pytest.raises(ValueError, match="Expected positive"):
        DiffDrive(-1)
