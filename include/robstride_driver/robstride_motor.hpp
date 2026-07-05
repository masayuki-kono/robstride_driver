// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>

#include "robstride_driver/actuator_types.hpp"
#include "robstride_driver/can_interface.hpp"
#include "robstride_driver/protocol.hpp"

namespace robstride {

/// Thrown when the motor does not answer a command within the configured
/// response timeout.
class TimeoutError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// High-level control of a single RobStride motor over a CanInterface.
///
/// Each command sends one frame and synchronously waits for the motor's
/// response frame (feedback, communication type 2, unless noted otherwise).
/// The latest feedback is cached and accessible via last_feedback().
///
/// This class is not thread-safe; serialize access externally. Multiple
/// RobstrideMotor instances may share one CanInterface only if their calls
/// are serialized, because responses are matched by motor id and frames of
/// other motors are discarded.
class RobstrideMotor {
 public:
  struct Config {
    /// CAN id of the target motor (0x7F is the factory default).
    std::uint8_t motor_id = 0x7F;
    /// CAN id representing this host in command frames.
    std::uint8_t host_id = 0xFD;
    /// Motor model; selects the fixed-point scaling ranges.
    ActuatorType actuator_type = ActuatorType::Rs02;
    /// Max time to wait for a response frame per command.
    std::chrono::milliseconds response_timeout{100};
  };

  RobstrideMotor(std::shared_ptr<CanInterface> can, const Config& config);

  /// Enables the motor (communication type 3).
  Feedback enable();

  /// Stops the motor (communication type 4). Set `clear_fault` to also
  /// clear a latched fault.
  Feedback disable(bool clear_fault = false);

  /// Switches run_mode (parameter 0x7005). Per the vendor manual the motor
  /// must not be running while the mode changes, so this stops the motor,
  /// writes the mode, and reads it back to verify. The motor is left
  /// disabled; call enable() afterwards.
  void set_run_mode(RunMode mode);

  /// Reads run_mode (parameter 0x7005) from the motor.
  RunMode get_run_mode();

  /// Writes the current limit (0x7018 [A]) and acceleration (0x7022
  /// [rad/s^2]) used by velocity mode.
  void configure_velocity_mode(double current_limit, double acceleration);

  /// Sends a velocity-mode target (0x700A [rad/s]). Requires
  /// RunMode::Velocity and an enabled motor. Returns the feedback frame
  /// answered by the motor.
  Feedback send_velocity_command(double velocity);

  /// Sends a CSP position-mode target: speed limit (0x7017 [rad/s]) then
  /// target angle (0x7016 [rad]). Requires RunMode::PositionCsp and an
  /// enabled motor.
  Feedback send_position_csp_command(double position, double speed_limit);

  /// Sends an operation-control (MIT) command (communication type 1).
  /// Requires RunMode::OperationControl and an enabled motor.
  Feedback send_motion_command(double torque, double position, double velocity,
                               double kp, double kd);

  /// Sets the current position as mechanical zero (communication type 6).
  /// The motor must be disabled; this stops it first and leaves it
  /// disabled.
  void set_mechanical_zero();

  /// Reads a float parameter (communication type 17).
  float read_param_float(std::uint16_t index);

  /// Reads a uint8 parameter (communication type 17).
  std::uint8_t read_param_uint8(std::uint16_t index);

  /// Writes a float parameter (communication type 18).
  Feedback write_param(std::uint16_t index, float value);

  /// Writes a uint8 parameter (communication type 18).
  Feedback write_param(std::uint16_t index, std::uint8_t value);

  /// Latest feedback received from the motor, if any.
  [[nodiscard]] const std::optional<Feedback>& last_feedback() const {
    return last_feedback_;
  }

  /// CAN id of the controlled motor.
  [[nodiscard]] std::uint8_t motor_id() const { return config_.motor_id; }

  /// Fixed-point scaling ranges of the configured actuator model.
  [[nodiscard]] const ActuatorLimits& limits() const { return limits_; }

 private:
  /// Sends `frame` and waits for a response with communication type
  /// `expected_type` from this motor. Feedback frames encountered along
  /// the way update last_feedback_. Throws TimeoutError on timeout.
  CanFrame transceive(const CanFrame& frame, CommType expected_type);

  /// Sends `frame` and waits for the feedback response, returning it.
  Feedback transceive_feedback(const CanFrame& frame);

  /// Transport shared with other motors on the same bus.
  std::shared_ptr<CanInterface> can_;
  /// Motor id, host id, model and timeout settings (fixed at construction).
  Config config_;
  /// Fixed-point scaling ranges for config_.actuator_type.
  ActuatorLimits limits_;
  /// Most recent feedback frame seen from this motor (nullopt until the
  /// first response arrives).
  std::optional<Feedback> last_feedback_;
};

}  // namespace robstride
