# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-07-05

Initial release.

### Added

- ROS-independent C++20 / CMake driver library for RobStride
  quasi-direct-drive motors, implementing the RobStride private CAN
  protocol (29-bit extended frames, 1 Mbps).
- Two transports behind the `CanInterface` abstraction:
  - Linux SocketCAN (`SocketCanInterface`), compatible with any
    SocketCAN adapter.
  - The official RobStride USB-CAN module (`AtSerialCanInterface`,
    CH340 serial bridge with AT framing at 921600 baud).
- High-level `RobstrideMotor` API: enable/disable, velocity mode, CSP /
  PP position modes, current mode, operation (MIT) control, feedback
  parsing, parameter read/write and mechanical zero.
- `PositionUnwrapper` helper that converts the wrapped feedback position
  (±4π on RS02) into a continuous position.
- Actuator range tables for RS00–RS06 (RS02 validated on real hardware).
- GoogleTest unit tests covering the protocol, motor logic and both
  transports without hardware.
- CLI examples: `velocity_control` and `tracking_capture_<mode>` capture
  programs with a Python plotting tool.
- Documentation: hardware setup, host setup, architecture, CAN protocol
  summary and hardware-in-the-loop tracking results for all control
  modes on a real RS02.
- CMake package config (`find_package(robstride_driver)`) and a
  `package.xml` so the library can be built in a ROS 2 colcon workspace.
- CI (gcc/clang build + tests), lint workflow (clang-format, clang-tidy,
  ruff) and pre-commit configuration.

[Unreleased]: https://github.com/masayuki-kono/robstride_driver/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/masayuki-kono/robstride_driver/releases/tag/v0.1.0
