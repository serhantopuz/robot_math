import numpy as np

from robot_math import SE2


def test_to_matrix_with_no_rotation() -> None:
    pose = SE2(3.0, 4.0, 0.0)
    expected = np.array([[1, 0, 3], [0, 1, 4], [0, 0, 1]])
    assert np.allclose(pose.to_matrix(), expected)
