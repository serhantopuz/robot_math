from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

ROTATION_TOLERANCE = 1e-6


@dataclass(frozen=True)
class SE2:
    """A rigid-body pose in the plane: a position and a heading."""

    x: float
    y: float
    theta: float

    def to_matrix(self) -> npt.NDArray[np.float64]:
        """Returns the 3x3 homogeneous transformation matrix for this pose."""
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)
        return np.array(
            [[cos_theta, -sin_theta, self.x], [sin_theta, cos_theta, self.y], [0, 0, 1]]
        )

    @classmethod
    def identity(cls) -> "SE2":
        """The pose that does nothing: no translation, no rotation"""
        return cls(0, 0, 0)

    @classmethod
    def from_matrix(cls, m: npt.NDArray[np.float64]) -> "SE2":
        """Build a pose from a 3x3 homogeneous transform"""
        if m.shape != (3, 3):
            raise ValueError(f"Expected a 3x3 matrix, got shape {m.shape}")
        if not np.allclose(m[2], [0, 0, 1]):
            raise ValueError(f"Expected [0, 0, 1] for bottom row, got {m[2]}")
        R = m[:2, :2]
        c1 = R[:, 0]
        c2 = R[:, 1]
        if not np.isclose(
            c1 @ c1, 1, atol=ROTATION_TOLERANCE, rtol=0
        ) or not np.isclose(c2 @ c2, 1, atol=ROTATION_TOLERANCE, rtol=0):
            raise ValueError(
                "Expected rotation columns of length 1 (a rotation cannot stretch),"
                f"got squared lengths {c1@c1:.6g} and {c2@c2:.6g}"
            )
        if not np.isclose(c1 @ c2, 0, atol=ROTATION_TOLERANCE, rtol=0):
            raise ValueError(
                "Expected perpendicular rotation columns (a rotation cannot shear),"
                f"got dot product {c1@c2:.6g}"
            )
        if not np.isclose(np.linalg.det(R), 1, atol=ROTATION_TOLERANCE, rtol=0):
            raise ValueError(
                "Expected rotation determinant +1 (a rotation cannot mirror),"
                f"got {np.linalg.det(R):.6g}"
            )

        x = float(m[0, 2])
        y = float(m[1, 2])
        theta = float(np.arctan2(m[1, 0], m[0, 0]))
        return cls(x, y, theta)

    def __matmul__(self, other: "SE2") -> "SE2":
        """Compose two poses: T_A_B @ T_B_C gives T_A_C"""
        c = np.cos(self.theta)
        s = np.sin(self.theta)
        x = float(self.x + c * other.x - s * other.y)
        y = float(self.y + s * other.x + c * other.y)
        theta = float(self.theta + other.theta)
        return type(self)(x, y, theta)
