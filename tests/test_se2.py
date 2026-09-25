import numpy as np
import pytest

from robot_math import SE2, wrap_angle


def assert_pose_close(actual: SE2, expected: SE2) -> None:
    assert actual.x == pytest.approx(expected.x)
    assert actual.y == pytest.approx(expected.y)
    assert wrap_angle(actual.theta - expected.theta) == pytest.approx(0.0)


def test_to_matrix_with_no_rotation() -> None:
    pose = SE2(3.0, 4.0, 0.0)
    expected = np.array([[1, 0, 3], [0, 1, 4], [0, 0, 1]])
    assert np.allclose(pose.to_matrix(), expected)


def test_identity_is_identity_matrix() -> None:
    pose = SE2.identity()
    assert np.allclose(pose.to_matrix(), np.identity(3))


def test_from_matrix_round_trip_with_negative_angle() -> None:
    original = SE2(1.5, -2.0, -2.0)
    recovered = SE2.from_matrix(original.to_matrix())
    assert_pose_close(recovered, original)


def test_from_matrix_rejects_wrong_shape() -> None:
    bad = np.eye(2)
    with pytest.raises(ValueError, match="3x3"):
        SE2.from_matrix(bad)


def test_from_matrix_rejects_bad_bottom_row() -> None:
    bad = np.array([[1, 0, 0], [0, 1, 0], [1, 1, 1]])
    with pytest.raises(ValueError, match="bottom row"):
        SE2.from_matrix(bad)


def test_from_matrix_rejects_area_preserving_stretch() -> None:
    bad = np.array([[2, 0, 0], [0, 0.5, 0], [0, 0, 1]])
    with pytest.raises(ValueError, match="stretch"):
        SE2.from_matrix(bad)


def test_from_matrix_rejects_one_stretched_column() -> None:
    bad = np.array([[2, 0, 0], [0, 1, 0], [0, 0, 1]])
    with pytest.raises(ValueError, match="stretch"):
        SE2.from_matrix(bad)


def test_from_matrix_rejects_shear() -> None:
    bad = np.array([[1, 0.5, 0], [0, np.sqrt(3) / 2, 0], [0, 0, 1]])
    with pytest.raises(ValueError, match="shear"):
        SE2.from_matrix(bad)


def test_from_matrix_rejects_mirror() -> None:
    bad = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
    with pytest.raises(ValueError, match="mirror"):
        SE2.from_matrix(bad)


def test_compose_applies_motion_in_robot_frame() -> None:
    result = SE2(0, 0, np.pi / 2) @ SE2(1, 0, np.pi / 2)
    assert_pose_close(result, SE2(0, 1, np.pi))


def test_compose_with_identity_changes_nothing() -> None:
    p = SE2(2.0, 1.0, np.pi / 2)
    assert np.allclose((SE2.identity() @ p).to_matrix(), p.to_matrix())
    assert np.allclose((p @ SE2.identity()).to_matrix(), p.to_matrix())


def test_compose_matches_matrix_multiplication() -> None:
    a = SE2(1.5, -2.0, 0.7)
    b = SE2(-0.3, 4.0, -2.1)
    assert np.allclose((a @ b).to_matrix(), a.to_matrix() @ b.to_matrix())


def test_wrap_angle_maps_into_minus_pi_to_pi() -> None:
    assert wrap_angle(6.0) == pytest.approx(6.0 - 2 * np.pi)
    assert wrap_angle(np.deg2rad(370)) == pytest.approx(np.deg2rad(10))
    assert wrap_angle(-4.0) == pytest.approx(-4.0 + 2 * np.pi)
    assert wrap_angle(0.5) == pytest.approx(0.5)


def test_pose_stores_wrapped_theta() -> None:
    pose = SE2(0, 0, 6.0)
    assert pose.theta == pytest.approx(6.0 - 2 * np.pi)


def test_compose_result_is_wrapped() -> None:
    a = SE2(0, 0, 3.0)
    b = SE2(0, 0, 3.0)
    via_matrices = SE2.from_matrix(a.to_matrix() @ b.to_matrix())
    assert (a @ b).theta == pytest.approx(via_matrices.theta)


def test_inverse_rotates_translation_back() -> None:
    assert_pose_close(SE2(2, 0, np.pi / 2).inverse(), SE2(0, 2, -np.pi / 2))


def test_inverse_composes_to_identity() -> None:
    p = SE2(1.5, -2.0, 0.7)
    assert_pose_close(p @ p.inverse(), SE2.identity())
    assert_pose_close(p.inverse() @ p, SE2.identity())


def test_inverse_matches_matrix_inverse() -> None:
    p = SE2(1.5, -2.0, 0.7)
    assert np.allclose(p.inverse().to_matrix(), np.linalg.inv(p.to_matrix()))
