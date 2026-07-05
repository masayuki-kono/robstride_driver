// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// AT serial framing samples from vendor documentation.

#pragma once

#include <array>
#include <cstdint>

namespace robstride::test_fixtures::at_serial_samples {

/// Worked example from the RS02 User Manual, section 3.3.5:
///   41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
/// carries CAN id 0x1200FD01 (comm type 18, host 0xFD, motor 0x01).
inline constexpr std::array<std::uint8_t, 17> manual_run_mode_frame = {
    0x41, 0x54, 0x90, 0x07, 0xe8, 0x0c, 0x08, 0x05, 0x70,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};

}  // namespace robstride::test_fixtures::at_serial_samples
