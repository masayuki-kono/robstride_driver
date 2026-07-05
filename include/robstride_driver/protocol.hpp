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
  std::uint32_t id = 0;                ///< 29-bit extended identifier
  std::array<std::uint8_t, 8> data{};  ///< data field (valid bytes: dlc)
  std::uint8_t dlc = 8;                ///< data length code (0-8)
};

/// Communication types (bit28-24 of the 29-bit identifier).
enum class CommType : std::uint8_t {
  GetDeviceId = 0x00,
  MotionControl = 0x01,
  Feedback = 0x02,
  Enable = 0x03,
  Stop = 0x04,
  SetMechanicalZero = 0x06,
  SetCanId = 0x07,
  ReadParam = 0x11,      // 17: single parameter read
  WriteParam = 0x12,     // 18: single parameter write (volatile)
  FaultFeedback = 0x15,  // 21: spontaneous fault/warning report
  SaveData = 0x16,       // 22: persist parameters to flash
  SetBaudRate = 0x17,    // 23: change CAN bit rate (effective on repower)
  ActiveReport = 0x18,   // 24: periodic active reporting control
  SetProtocol = 0x19,    // 25: switch private / CANopen / MIT protocol
};

/// run_mode (parameter 0x7005) values.
enum class RunMode : std::uint8_t {
  OperationControl = 0,  ///< MIT-style motion control (communication type 1)
  PositionPp = 1,        ///< profile position
  Velocity = 2,          ///< velocity control (target: spd_ref 0x700A)
  Current = 3,           ///< Iq current control (target: iq_ref 0x7006)
  SetZero = 4,           ///< zero-calibration mode
  PositionCsp = 5,       ///< cyclic synchronous position
};

/// Writable / readable parameter indices (communication types 17/18).
/// See RS02 User Manual 4.1.13 "Read and write a single parameter list".
namespace param_index {
inline constexpr std::uint16_t run_mode = 0x7005;       ///< uint8
inline constexpr std::uint16_t iq_ref = 0x7006;         ///< float [A]
inline constexpr std::uint16_t spd_ref = 0x700A;        ///< float [rad/s]
inline constexpr std::uint16_t limit_torque = 0x700B;   ///< float [Nm]
inline constexpr std::uint16_t cur_kp = 0x7010;         ///< float
inline constexpr std::uint16_t cur_ki = 0x7011;         ///< float
inline constexpr std::uint16_t cur_filt_gain = 0x7014;  ///< float
inline constexpr std::uint16_t loc_ref = 0x7016;        ///< float [rad]
inline constexpr std::uint16_t limit_spd = 0x7017;      ///< float [rad/s] (CSP)
inline constexpr std::uint16_t limit_cur = 0x7018;      ///< float [A]
inline constexpr std::uint16_t mech_pos = 0x7019;       ///< float [rad] (R)
inline constexpr std::uint16_t iq_filtered = 0x701A;    ///< float [A] (R)
inline constexpr std::uint16_t mech_vel = 0x701B;       ///< float [rad/s] (R)
inline constexpr std::uint16_t v_bus = 0x701C;          ///< float [V] (R)
inline constexpr std::uint16_t loc_kp = 0x701E;         ///< float
inline constexpr std::uint16_t spd_kp = 0x701F;         ///< float
inline constexpr std::uint16_t spd_ki = 0x7020;         ///< float
inline constexpr std::uint16_t spd_filt_gain = 0x7021;  ///< float
inline constexpr std::uint16_t acc_rad =
    0x7022;  ///< float [rad/s^2] velocity-mode acceleration
inline constexpr std::uint16_t vel_max = 0x7024;      ///< float [rad/s] (PP)
inline constexpr std::uint16_t acc_set = 0x7025;      ///< float [rad/s^2] (PP)
inline constexpr std::uint16_t can_timeout = 0x7028;  ///< uint32, 20000 = 1 s
}  // namespace param_index

/// Mode status reported in feedback frames (bit23-22 of the identifier).
enum class MotorMode : std::uint8_t {
  Reset = 0,
  Calibration = 1,
  Run = 2,
};

/// Fault bits reported in feedback frames (bit21-16 of the identifier).
struct FaultStatus {
  std::uint8_t raw = 0;  ///< 6-bit raw fault field (bit21-16)

  // bit16
  [[nodiscard]] bool undervoltage() const { return (raw & 0x01) != 0; }
  // bit17
  [[nodiscard]] bool overcurrent() const { return (raw & 0x02) != 0; }
  // bit18
  [[nodiscard]] bool overtemperature() const { return (raw & 0x04) != 0; }
  // bit19
  [[nodiscard]] bool magnetic_encoding() const { return (raw & 0x08) != 0; }
  // bit20
  [[nodiscard]] bool stall_overload() const { return (raw & 0x10) != 0; }
  // bit21
  [[nodiscard]] bool uncalibrated() const { return (raw & 0x20) != 0; }
  [[nodiscard]] bool any() const { return raw != 0; }
};

/// Decoded motor feedback (communication type 2).
struct Feedback {
  std::uint8_t motor_id = 0;          ///< motor that sent the feedback
  std::uint8_t host_id = 0;           ///< destination host id
  MotorMode mode = MotorMode::Reset;  ///< reset / calibration / run
  FaultStatus fault{};                ///< fault bits (bit21-16 of the id)
  double position = 0.0;              ///< [rad]
  double velocity = 0.0;              ///< [rad/s]
  double torque = 0.0;                ///< [Nm]
  double temperature = 0.0;           ///< [Celsius]
};

/// Decoded parameter-read response (communication type 17).
struct ParamResponse {
  std::uint8_t motor_id = 0;           ///< motor that sent the response
  std::uint16_t index = 0;             ///< parameter index (0x7000 family)
  std::array<std::uint8_t, 4> data{};  ///< little-endian value bytes

  [[nodiscard]] float as_float() const;
  [[nodiscard]] std::uint8_t as_uint8() const { return data[0]; }
};

/// Decoded parameter-read request (communication type 17, host -> motor).
struct ParamReadRequest {
  std::uint8_t motor_id = 0;  ///< destination motor
  std::uint8_t host_id = 0;   ///< requesting host
  std::uint16_t index = 0;    ///< parameter index (0x7000 family)
};

/// Decoded parameter-write request (communication type 18, host -> motor).
struct ParamWriteRequest {
  std::uint8_t motor_id = 0;           ///< destination motor
  std::uint8_t host_id = 0;            ///< requesting host
  std::uint16_t index = 0;             ///< parameter index (0x7000 family)
  std::array<std::uint8_t, 4> data{};  ///< little-endian value bytes

  [[nodiscard]] float as_float() const;
  [[nodiscard]] std::uint8_t as_uint8() const { return data[0]; }
};

// ---------------------------------------------------------------------------
// Scaling helpers
// ---------------------------------------------------------------------------

/// Linearly maps `x` (clamped to [min, max]) onto [0, 2^bits - 1].
std::uint16_t float_to_uint(double x, double min, double max, int bits = 16);

/// Inverse of float_to_uint.
double uint_to_float(std::uint16_t value, double min, double max,
                     int bits = 16);

// ---------------------------------------------------------------------------
// Frame encoding (host -> motor)
// ---------------------------------------------------------------------------

/// Communication type 3: enable the motor.
CanFrame make_enable_frame(std::uint8_t motor_id, std::uint8_t host_id);

/// Communication type 4: stop the motor. Set `clear_fault` to clear a
/// latched fault.
CanFrame make_stop_frame(std::uint8_t motor_id, std::uint8_t host_id,
                         bool clear_fault = false);

/// Communication type 6: set the current position as the mechanical zero.
CanFrame make_set_mechanical_zero_frame(std::uint8_t motor_id,
                                        std::uint8_t host_id);

/// Communication type 7: change the motor CAN id (effective immediately).
CanFrame make_set_can_id_frame(std::uint8_t motor_id, std::uint8_t host_id,
                               std::uint8_t new_motor_id);

/// Communication type 0: request the device id / MCU unique identifier.
CanFrame make_get_device_id_frame(std::uint8_t motor_id, std::uint8_t host_id);

/// Communication type 17: read a single parameter.
CanFrame make_read_param_frame(std::uint8_t motor_id, std::uint8_t host_id,
                               std::uint16_t index);

/// Communication type 18: write a single float parameter.
CanFrame make_write_param_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                std::uint16_t index, float value);

/// Communication type 18: write a single uint8 parameter (e.g. run_mode).
CanFrame make_write_param_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                std::uint16_t index, std::uint8_t value);

/// Communication type 1: operation (motion) control command. The torque
/// feed-forward is carried in the identifier; position/velocity/Kp/Kd are
/// carried big-endian in the data field, scaled by `limits`.
CanFrame make_motion_control_frame(std::uint8_t motor_id, double torque,
                                   double position, double velocity, double kp,
                                   double kd, const ActuatorLimits& limits);

// ---------------------------------------------------------------------------
// Frame encoding (motor -> host, used by simulated transports)
// ---------------------------------------------------------------------------

/// Communication type 2: feedback frame as sent by a motor. Physical
/// values are scaled by `limits`; the inverse of parse_feedback.
CanFrame make_feedback_frame(const Feedback& feedback,
                             const ActuatorLimits& limits);

/// Communication type 17: parameter-read response with a float value.
CanFrame make_param_response_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                   std::uint16_t index, float value);

/// Communication type 17: parameter-read response with a uint8 value
/// (e.g. run_mode).
CanFrame make_param_response_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                   std::uint16_t index, std::uint8_t value);

// ---------------------------------------------------------------------------
// Frame decoding (motor -> host)
// ---------------------------------------------------------------------------

/// Returns the communication type encoded in bit28-24 of the identifier.
std::uint8_t get_comm_type(const CanFrame& frame);

/// Returns the motor id of a motor-originated frame (bit15-8).
std::uint8_t get_source_motor_id(const CanFrame& frame);

// ---------------------------------------------------------------------------
// Frame decoding (host -> motor, used by simulated transports)
// ---------------------------------------------------------------------------

/// Returns the destination motor id of a host-originated frame (bit7-0).
std::uint8_t get_target_motor_id(const CanFrame& frame);

/// Returns the host id of a host-originated frame (bit15-8). Valid for
/// enable/stop/read/write frames where data area 2 carries the host id.
std::uint8_t get_host_id(const CanFrame& frame);

/// Decodes a parameter-read request (communication type 17, host ->
/// motor). Returns nullopt if the frame is not a read request.
std::optional<ParamReadRequest> parse_param_read_request(const CanFrame& frame);

/// Decodes a parameter-write request (communication type 18, host ->
/// motor). Returns nullopt if the frame is not a write request.
std::optional<ParamWriteRequest> parse_param_write_request(
    const CanFrame& frame);

/// Decodes a feedback frame (communication type 2). Returns nullopt if the
/// frame is not a feedback frame. Physical values are scaled by `limits`.
std::optional<Feedback> parse_feedback(const CanFrame& frame,
                                       const ActuatorLimits& limits);

/// Decodes a parameter-read response (communication type 17). Returns
/// nullopt if the frame is not a parameter response.
std::optional<ParamResponse> parse_param_response(const CanFrame& frame);

}  // namespace robstride
