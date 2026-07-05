// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Default CAN bus topology for unit tests (motor and host identifiers).

#pragma once

#include <cstdint>

namespace robstride::test_fixtures::can_bus {

/// Single-motor bus used by protocol and RobstrideMotor unit tests.
namespace single {
inline constexpr std::uint8_t motor_id = 0x01;
inline constexpr std::uint8_t host_id = 0xFD;
}  // namespace single

/// Dual-motor bus used by StubCanInterface integration tests.
namespace dual {
inline constexpr std::uint8_t left_motor_id = 0x01;
inline constexpr std::uint8_t right_motor_id = 0x02;
inline constexpr std::uint8_t host_id = 0xFD;
}  // namespace dual

}  // namespace robstride::test_fixtures::can_bus
