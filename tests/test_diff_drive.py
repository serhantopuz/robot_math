import numpy as np
import pytest

from robot_math import SE2, DiffDrive, motion_step
from robot_math.diff_drive import STRAIGHT_LINE_TOLERANCE


def assert_same_pose(actual: SE2, expected: SE2) -> None:
    assert np.allclose(actual.to_matrix(), expected.to_matrix())


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


def test_motion_step_zero_omega_drives_straight() -> None:
    step = motion_step(0.5, 0.0, 2.0)
    assert_same_pose(step, SE2(1.0, 0.0, 0.0))


def test_motion_step_turning_left_follows_quarter_circle() -> None:
    step = motion_step(1.0, 1.0, np.pi / 2)
    assert_same_pose(step, SE2(1.0, 1.0, np.pi / 2))


def test_motion_step_turning_right_follows_quarter_circle() -> None:
    step = motion_step(1.0, -1.0, np.pi / 2)
    assert_same_pose(step, SE2(1.0, -1.0, -np.pi / 2))


def test_motion_step_two_half_steps_equal_one_full_step() -> None:
    v, omega, dt = 0.43, -1.7, 0.37
    full = motion_step(v, omega, dt)
    half = motion_step(v, omega, dt / 2)
    assert_same_pose(half @ half, full)


def test_motion_step_branches_agree_at_tolerance() -> None:
    just_below = motion_step(1.0, 0.999 * STRAIGHT_LINE_TOLERANCE, 0.02)
    just_above = motion_step(1.0, 1.001 * STRAIGHT_LINE_TOLERANCE, 0.02)
    assert_same_pose(just_below, just_above)


def test_motion_step_rejects_negative_dt() -> None:
    with pytest.raises(ValueError, match="non-negative dt"):
        motion_step(1.0, 0.5, -0.1)


def test_dead_reckon_with_no_readings_returns_start() -> None:
    start = SE2(1.5, -2.0, 0.7)
    path = DiffDrive(0.3).dead_reckon(start, [])
    assert len(path) == 1
    assert_same_pose(path[-1], start)


def test_dead_reckon_many_small_steps_match_one_big_step() -> None:
    drive = DiffDrive(0.287)
    start = SE2(1.5, -2.0, 0.7)
    v_left, v_right = 0.31, 0.47
    readings = [(v_left, v_right, 0.01)] * 100
    path = drive.dead_reckon(start, readings)
    v, omega = drive.body_velocity(v_left, v_right)
    assert len(path) == 101
    assert_same_pose(path[-1], start @ motion_step(v, omega, 1.0))


def test_dead_reckon_full_circle_returns_to_start() -> None:
    drive = DiffDrive(0.3)
    start = SE2(1.5, -2.0, 0.7)
    _, omega = drive.body_velocity(0.2, 0.4)
    period = 2 * np.pi / omega
    readings = [(0.2, 0.4, period / 1000)] * 1000
    path = drive.dead_reckon(start, readings)
    assert_same_pose(path[-1], start)


def test_dead_reckon_square_returns_to_start() -> None:
    drive = DiffDrive(0.3)
    start = SE2(1.5, -2.0, 0.7)
    _, turn_rate = drive.body_velocity(-0.15, 0.15)
    turn_dt = (np.pi / 2) / turn_rate
    one_side = [(0.5, 0.5, 2.0), (-0.15, 0.15, turn_dt)]
    readings = one_side * 4
    path = drive.dead_reckon(start, readings)
    assert_same_pose(path[-1], start)
