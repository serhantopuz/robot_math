# robot_math

Rigid-body transforms and differential-drive kinematics, implemented twice: once in Python, once in C++ with Eigen.

**Status:** in development.

A deliberately small library written to be correct rather than clever. SE(2)/SE(3) composition, inversion and interpolation; quaternion, rotation-matrix and Euler conversions; differential-drive forward kinematics and a dead-reckoning integrator. Both implementations expose the same API and are verified against each other.

## Stack

- Python 3.12 · NumPy · pytest · mypy · ruff
- C++17 · Eigen · CMake · GTest

## Roadmap

- [ ] Python SE(2)/SE(3) with property-based round-trip tests
- [ ] C++/Eigen port, installable CMake target, mirrored GTest suite
- [ ] Stretch: pybind11 bindings, bit-identical results across both paths

## License

Apache-2.0
