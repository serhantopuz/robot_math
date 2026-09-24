import numpy as np
import pytest

from robot_math import SE2


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

    assert recovered.x == pytest.approx(original.x)
    assert recovered.y == pytest.approx(original.y)
    assert recovered.theta == pytest.approx(original.theta)


def test_from_matrix_rejects_wrong_shape() -> None:
    bad = np.eye(2)
    with pytest.raises(ValueError, match="3x3"):
        SE2.from_matrix(bad)


def test_from_matrix_rejects_bad_bottom_row() -> None:
    bad = np.array([[1, 0, 0], [0, 1, 0], [1, 1, 1]])
    with pytest.raises(ValueError, match="bottom row"):
        SE2.from_matrix(bad)
