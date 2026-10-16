# robot_math

2D rigid-body transforms (SE(2)) and differential-drive odometry, implemented twice: once in
Python with NumPy, once in C++17 with Eigen. Both implementations have the same features and
mirrored test suites (39 pytest tests, 37 GTest tests).

## Features

- **`SE2` poses** — construct from `(x, y, θ)` or a 3×3 homogeneous matrix (validated), convert
  back to a matrix, compose, invert, and transform one point or many.
- **Differential-drive kinematics** — wheel speeds → body velocity `(v, ω)` for odometry, and
  `(v, ω)` → wheel speeds for control.
- **Exact motion step** — integrates constant `(v, ω)` over `dt` along the true arc, with a
  straight-line fallback when `ω` is near zero.
- **Dead reckoning** — chains motion steps over a sequence of wheel readings into a path.

## Example

Drive a 1 m square and check that the robot ends where it started.

**Python**

```python
import numpy as np
from robot_math import SE2, DiffDrive

drive = DiffDrive(wheel_base=0.3)
start = SE2(1.5, -2.0, 0.7)

_, turn_rate = drive.body_velocity(-0.15, 0.15)  # spin on the spot
straight = (0.5, 0.5, 2.0)  # 1 m straight
turn = (-0.15, 0.15, (np.pi / 2) / turn_rate)  # 90° left
path = drive.dead_reckon(start, [straight, turn] * 4)

print(np.allclose(path[-1].to_matrix(), start.to_matrix()))  # True
```

**C++**

```cpp
#include <cmath>
#include <iostream>
#include <vector>

#include "robot_math/diff_drive.hpp"

int main() {
  const double pi = std::acos(-1.0);
  const robot_math::DiffDrive drive(0.3);
  const robot_math::SE2 start(1.5, -2.0, 0.7);

  const double turn_dt = (pi / 2) / drive.body_velocity(-0.15, 0.15).omega;
  std::vector<robot_math::WheelReading> readings;
  for (int i = 0; i < 4; ++i) {
    readings.push_back({0.5, 0.5, 2.0});         // 1 m straight
    readings.push_back({-0.15, 0.15, turn_dt});  // spin 90° left on the spot
  }

  const std::vector<robot_math::SE2> path = drive.dead_reckon(start, readings);
  const robot_math::SE2& end = path.back();
  std::cout << end.x() << ", " << end.y() << ", " << end.theta() << "\n";  // 1.5, -2, 0.7
}
```

## Build and test

Tested on Ubuntu 24.04. System packages for both halves:

```bash
sudo apt update
sudo apt install python3-venv build-essential cmake libeigen3-dev libgtest-dev
```

**Python** (3.12 or newer):

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -e ".[dev]"
pytest
ruff check . && ruff format --check . && mypy
```

The project uses a standard `pyproject.toml`, so `uv venv` and `uv pip install -e ".[dev]"`
work too. If ROS is sourced in your shell, run the Python tests in a shell without it: ROS
puts its packages on `PYTHONPATH`, which leaks into virtual environments.

**C++** (C++17, CMake 3.22 or newer):

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build
ctest --test-dir cpp/build --output-on-failure
```

## Design decisions

- **θ is stored, not a rotation matrix.** Any float is a valid angle, so integration can never
  produce an invalid rotation. θ is wrapped into [−π, π] on construction, and angles are
  compared by their wrapped difference, so the ±π seam never causes false mismatches.
- **Validation at the boundary.** `from_matrix` rejects a bad bottom row, stretch, shear and
  mirror, each with a message naming the failure. In C++ there is no shape check at all:
  `Eigen::Matrix3d` makes a wrongly sized matrix fail to compile.
- **Exact-arc integration.** `motion_step` follows the true arc instead of "drive straight,
  then turn". A test checks that two half steps equal one full step, which the naive formula
  fails. A second test checks that the arc and the straight-line fallback agree at the
  switch-over tolerance.
- **Each language follows its own conventions.** Points are rows `(N, 2)` in NumPy and columns
  `2 × N` in Eigen. Poses compose with `@` in Python and `*` in C++, as with Eigen matrices.
  C++ results are named structs (`BodyVelocity`, `WheelSpeeds`) instead of tuples, so
  `(v, ω)` and `(left, right)` cannot be mixed up.
- **Tests built to fail.** Oracle tests use deliberately messy inputs; round trips and closed
  paths (a full circle, a square) check properties instead of hand-computed numbers. Key tests
  were mutation-checked: swapping the wheels or the composition order is caught by the
  many-small-steps test, which the closed paths alone would miss.

## Layout

```
robot_math/
├── pyproject.toml
├── src/robot_math/         # Python: se2.py, diff_drive.py
├── tests/                  # pytest
└── cpp/
    ├── CMakeLists.txt
    ├── include/robot_math/ # se2.hpp, diff_drive.hpp
    ├── src/                # se2.cpp, diff_drive.cpp
    └── tests/              # GTest
```

Code style: ruff and strict mypy for Python; clang-format (Google style, 100 columns) and
`-Wall -Wextra` with zero warnings for C++.

## License

Apache-2.0
