// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

namespace robstride {

/// RobStride actuator models supported by this driver.
///
/// RS02 is the primary, hardware-validated target. Range values for the
/// other models are taken from the vendor sample code and have not been
/// cross-checked against their official manuals.
enum class ActuatorType : std::uint8_t {
  Rs00 = 0,
  Rs01 = 1,
  Rs02 = 2,
  Rs03 = 3,
  Rs04 = 4,
  Rs05 = 5,
  Rs06 = 6,
};

/// Physical ranges used to encode/decode 16-bit fixed-point values in
/// motion-control command frames (communication type 0x01) and feedback
/// frames (communication type 0x02).
///
/// All ranges are symmetric: a value maps linearly from
/// [-limit, +limit] to [0, 65535].
struct ActuatorLimits {
  double position;  ///< [rad]   feedback/command angle range (+-)
  double velocity;  ///< [rad/s] feedback/command velocity range (+-)
  double torque;    ///< [Nm]    feedback/command torque range (+-)
  double kp_max;    ///< motion-control Kp range [0, kp_max]
  double kd_max;    ///< motion-control Kd range [0, kd_max]
};

/// Returns the encode/decode limits for the given actuator type.
///
/// RS02 (per RS02 User Manual): position +-4*pi rad, velocity +-44 rad/s,
/// torque +-17 Nm, Kp 0-500, Kd 0-5.
const ActuatorLimits& get_actuator_limits(ActuatorType type);

}  // namespace robstride
