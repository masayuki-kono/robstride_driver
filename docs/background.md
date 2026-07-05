# Background and Motivation

This document explains why `robstride_driver` was created, how it relates to
existing RobStride software, and the design choices behind a ROS-independent
library.

## Purpose

`robstride_driver` is a C++20 library for controlling RobStride quasi-direct-drive
motors from Linux. It was developed to answer a practical question: can RobStride
motors be controlled reliably from Linux, using the hardware that is actually
available (including the official RobStride USB-CAN module)?

The goal was not a minimal demo but a library suitable from initial feasibility
work through production integration: all control modes, parameter read/write,
unit tests, CI, and a transport abstraction that works with both SocketCAN and
the official USB-CAN module.

Hardware-in-the-loop results are documented in [test_results.md](test_results.md).

## Existing implementations surveyed

Before writing this library, several existing implementations were reviewed.

### Official ROS samples

RobStride publishes ROS sample packages that implement the **private CAN
protocol** over **SocketCAN only**:

- [robstride_ros_sample](https://github.com/RobStride/robstride_ros_sample) (ROS 2)
- [robstride_actuator_bridge](https://github.com/RobStride/robstride_actuator_bridge) (ROS 1)

Both use `PF_CAN` / `SOCK_RAW` to send and receive 29-bit extended frames directly.
They do not support the official USB-CAN module's AT serial framing. Their README
states that the official serial-to-CAN module is intended for the vendor's tuning
software only and that a separate Canable adapter is needed on Ubuntu.

### Community library

[tianrking/RobStride_Control](https://github.com/tianrking/RobStride_Control)
provides Python, C++, Rust, and Arduino implementations. The Linux targets are
also **SocketCAN-only**; the C++ side is essentially a single-file position-control
demo rather than a full motor library.

### Comparison

| Aspect | Official ROS samples | RobStride_Control | robstride_driver |
|--------|---------------------|-------------------|------------------|
| Transport | SocketCAN only | SocketCAN only (Linux) | SocketCAN + **AT serial** |
| Official USB-CAN module | No | No | **Yes (hardware-verified)** |
| Control modes | Partial | Position demo focused | All five modes + param read/write |
| Form | ROS-coupled samples | Sample collection | ROS-independent library |
| Tests / CI | None | None | Unit tests + CI (format/lint) |

## Why a new library was needed

The decisive gap was **transport**. Every surveyed implementation assumes
SocketCAN. The official RobStride USB-CAN module does not expose a SocketCAN
interface; it speaks AT serial framing over a CH340 serial port (see
[hardware.md](hardware.md)). With only that module on hand, none of the existing
code could communicate with the motor at all.

A SocketCAN adapter (e.g. Canable) would have allowed basic motion checks with
existing samples, but several further requirements pointed to a new library:

- **Official module support** — MotorStudio, the vendor maintenance tool, works
  only with the official USB-CAN module. Deployments that use MotorStudio for
  maintenance benefit from using the same adapter on the control host (see
  [hardware.md](hardware.md#operational-guidance)).
- **Full control-mode coverage** — velocity, CSP/PP position, current, and
  operation control, plus parameter read/write.
- **Testability without hardware** — protocol and motor logic behind a transport
  abstraction, with unit tests and CI.
- **Framework independence** — a reusable library, not a ROS-tied sample.

For protocol selection, see [protocol.md](protocol.md#supported-protocol-variants).
Only the private protocol is implemented; that is sufficient for feasibility and
initial migration work.

## ROS-independent design

The library deliberately has **no ROS dependency**. Integration with a robot
framework (ROS 2, or any other middleware) belongs in a separate downstream
package that wraps this library.

Rationale:

- Protocol and motor logic can be unit-tested and CI-verified without a ROS
  workspace.
- The library is reusable across projects and ROS distributions.
- Transport and protocol layers stay stable while the application-facing API
  (topics, services, lifecycle) evolves in the wrapper.

See [architecture.md](architecture.md) for the internal layering (`CanInterface`,
`protocol`, `RobstrideMotor`).
