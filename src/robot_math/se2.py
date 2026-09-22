from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class SE2:
    """Theta is in radian."""

    x: float
    y: float
    theta: float

    def to_matrix(self) -> npt.NDArray[np.float64]:
        """Returns 3x3 matrix"""
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)
        return np.array(
            [[cos_theta, -sin_theta, self.x], [sin_theta, cos_theta, self.y], [0, 0, 1]]
        )
