from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


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
        x = float(m[0, 2])
        y = float(m[1, 2])
        theta = float(np.arctan2(m[1, 0], m[0, 0]))
        return cls(x, y, theta)
