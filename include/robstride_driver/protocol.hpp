// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// RobStride private CAN protocol: frame encoding/decoding.
//
// The protocol uses CAN 2.0 extended frames (29-bit identifier) at 1 Mbps.
// The identifier layout is:
//
//   bit28-24 : communication type
//   bit23-8  : data area 2 (host id, torque, fault bits, ... type dependent)
//   bit7-0   : destination address
//
// Reference: RobStride RS02 User Manual, chapter 4 "Driver protocol and
// instructions".

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "robstride_driver/actuator_types.hpp"

namespace robstride {

/// A raw CAN 2.0B data frame. `id` holds the 29-bit extended identifier
/// without any transport-specific flag bits (e.g. Linux CAN_EFF_FLAG).
struct CanFrame {
  std::uint32_t id = 0;               ///< 29-bit extended identifier
  std::array<std::uint8_t, 8> data{}; ///< data field (valid bytes: dlc)
  std::uint8_t dlc = 8;               ///< data length code (0-8)
};

/// Communication types (bit28-24 of the 29-bit identifier).
enum class CommType : std::uint8_t {
  kGetDeviceId = 0x00,
  kMotionControl = 0x01,
  kFeedback = 0x02,
  kEnable = 0x03,
  kStop = 0x04,
  kSetMechanicalZero = 0x06,
  kSetCanId = 0x07,
  kReadParam = 0x11,     // 17: single parameter read
  kWriteParam = 0x12,    // 18: single parameter write (volatile)
  kFaultFeedback = 0x15, // 21: spontaneous fault/warning report
  kSaveData = 0x16,      // 22: persist parameters to flash
  kSetBaudRate = 0x17,   // 23: change CAN bit rate (effective on repower)
  kActiveReport = 0x18,  // 24: periodic active reporting control
  kSetProtocol = 0x19,   // 25: switch private / CANopen / MIT protocol
};

/// run_mode (parameter 0x7005) values.
enum class RunMode : std::uint8_t {
  kOperationControl = 0, ///< MIT-style motion control (communication type 1)
  kPositionPp = 1,       ///< profile position
  kVelocity = 2,         ///< velocity control (target: spd_ref 0x700A)
  kCurrent = 3,          ///< Iq current control (target: iq_ref 0x7006)
  kSetZero = 4,          ///< zero-calibration mode
  kPositionCsp = 5,      ///< cyclic synchronous position
};

/// Writable / readable parameter indices (communication types 17/18).
/// See RS02 User Manual 4.1.13 "Read and write a single parameter list".
namespace param_index {
inline constexpr std::uint16_t kRunMode = 0x7005;      ///< uint8
inline constexpr std::uint16_t kIqRef = 0x7006;        ///< float [A]
inline constexpr std::uint16_t kSpdRef = 0x700A;       ///< float [rad/s]
inline constexpr std::uint16_t kLimitTorque = 0x700B;  ///< float [Nm]
inline constexpr std::uint16_t kCurKp = 0x7010;        ///< float
inline constexpr std::uint16_t kCurKi = 0x7011;        ///< float
inline constexpr std::uint16_t kCurFiltGain = 0x7014;  ///< float
inline constexpr std::uint16_t kLocRef = 0x7016;       ///< float [rad]
inline constexpr std::uint16_t kLimitSpd = 0x7017;     ///< float [rad/s] (CSP)
inline constexpr std::uint16_t kLimitCur = 0x7018;     ///< float [A]
inline constexpr std::uint16_t kMechPos = 0x7019;      ///< float [rad] (R)
inline constexpr std::uint16_t kIqFiltered = 0x701A;   ///< float [A] (R)
inline constexpr std::uint16_t kMechVel = 0x701B;      ///< float [rad/s] (R)
inline constexpr std::uint16_t kVBus = 0x701C;         ///< float [V] (R)
inline constexpr std::uint16_t kLocKp = 0x701E;        ///< float
inline constexpr std::uint16_t kSpdKp = 0x701F;        ///< float
inline constexpr std::uint16_t kSpdKi = 0x7020;        ///< float
inline constexpr std::uint16_t kSpdFiltGain = 0x7021;  ///< float
inline constexpr std::uint16_t kAccRad = 0x7022;       ///< float [rad/s^2] velocity-mode acceleration
inline constexpr std::uint16_t kVelMax = 0x7024;       ///< float [rad/s] (PP)
inline constexpr std::uint16_t kAccSet = 0x7025;       ///< float [rad/s^2] (PP)
inline constexpr std::uint16_t kCanTimeout = 0x7028;   ///< uint32, 20000 = 1 s
}  // namespace param_index

/// Mode status reported in feedback frames (bit23-22 of the identifier).
enum class MotorMode : std::uint8_t {
  kReset = 0,
  kCalibration = 1,
  kRun = 2,
};

/// Fault bits reported in feedback frames (bit21-16 of the identifier).
struct FaultStatus {
  std::uint8_t raw = 0; ///< 6-bit raw fault field (bit21-16)

  bool undervoltage() const { return raw & 0x01; }       // bit16
  bool overcurrent() const { return raw & 0x02; }        // bit17
  bool overtemperature() const { return raw & 0x04; }    // bit18
  bool magnetic_encoding() const { return raw & 0x08; }  // bit19
  bool stall_overload() const { return raw & 0x10; }     // bit20
  bool uncalibrated() const { return raw & 0x20; }       // bit21
  bool any() const { return raw != 0; }
};

/// Decoded motor feedback (communication type 2).
struct Feedback {
  std::uint8_t motor_id = 0;          ///< motor that sent the feedback
  std::uint8_t host_id = 0;           ///< destination host id
  MotorMode mode = MotorMode::kReset; ///< reset / calibration / run
  FaultStatus fault{};                ///< fault bits (bit21-16 of the id)
  double position = 0.0;    ///< [rad]
  double velocity = 0.0;    ///< [rad/s]
  double torque = 0.0;      ///< [Nm]
  double temperature = 0.0; ///< [Celsius]
};

/// Decoded parameter-read response (communication type 17).
struct ParamResponse {
  std::uint8_t motor_id = 0;          ///< motor that sent the response
  std::uint16_t index = 0;            ///< parameter index (0x7000 family)
  std::array<std::uint8_t, 4> data{}; ///< little-endian value bytes

  float AsFloat() const;
  std::uint8_t AsUint8() const { return data[0]; }
};

// ---------------------------------------------------------------------------
// Scaling helpers
// ---------------------------------------------------------------------------

/// Linearly maps `x` (clamped to [min, max]) onto [0, 2^bits - 1].
std::uint16_t FloatToUint(double x, double min, double max, int bits = 16);

/// Inverse of FloatToUint.
double UintToFloat(std::uint16_t value, double min, double max, int bits = 16);

// ---------------------------------------------------------------------------
// Frame encoding (host -> motor)
// ---------------------------------------------------------------------------

/// Communication type 3: enable the motor.
CanFrame MakeEnableFrame(std::uint8_t motor_id, std::uint8_t host_id);

/// Communication type 4: stop the motor. Set `clear_fault` to clear a
/// latched fault.
CanFrame MakeStopFrame(std::uint8_t motor_id, std::uint8_t host_id,
                       bool clear_fault = false);

/// Communication type 6: set the current position as the mechanical zero.
CanFrame MakeSetMechanicalZeroFrame(std::uint8_t motor_id,
                                    std::uint8_t host_id);

/// Communication type 7: change the motor CAN id (effective immediately).
CanFrame MakeSetCanIdFrame(std::uint8_t motor_id, std::uint8_t host_id,
                           std::uint8_t new_motor_id);

/// Communication type 0: request the device id / MCU unique identifier.
CanFrame MakeGetDeviceIdFrame(std::uint8_t motor_id, std::uint8_t host_id);

/// Communication type 17: read a single parameter.
CanFrame MakeReadParamFrame(std::uint8_t motor_id, std::uint8_t host_id,
                            std::uint16_t index);

/// Communication type 18: write a single float parameter.
CanFrame MakeWriteParamFrame(std::uint8_t motor_id, std::uint8_t host_id,
                             std::uint16_t index, float value);

/// Communication type 18: write a single uint8 parameter (e.g. run_mode).
CanFrame MakeWriteParamFrame(std::uint8_t motor_id, std::uint8_t host_id,
                             std::uint16_t index, std::uint8_t value);

/// Communication type 1: operation (motion) control command. The torque
/// feed-forward is carried in the identifier; position/velocity/Kp/Kd are
/// carried big-endian in the data field, scaled by `limits`.
CanFrame MakeMotionControlFrame(std::uint8_t motor_id, double torque,
                                double position, double velocity, double kp,
                                double kd, const ActuatorLimits& limits);

// ---------------------------------------------------------------------------
// Frame decoding (motor -> host)
// ---------------------------------------------------------------------------

/// Returns the communication type encoded in bit28-24 of the identifier.
std::uint8_t GetCommType(const CanFrame& frame);

/// Returns the motor id of a motor-originated frame (bit15-8).
std::uint8_t GetSourceMotorId(const CanFrame& frame);

/// Decodes a feedback frame (communication type 2). Returns nullopt if the
/// frame is not a feedback frame. Physical values are scaled by `limits`.
std::optional<Feedback> ParseFeedback(const CanFrame& frame,
                                      const ActuatorLimits& limits);

/// Decodes a parameter-read response (communication type 17). Returns
/// nullopt if the frame is not a parameter response.
std::optional<ParamResponse> ParseParamResponse(const CanFrame& frame);

}  // namespace robstride
