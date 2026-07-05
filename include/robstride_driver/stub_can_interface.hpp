// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>

#include "robstride_driver/actuator_types.hpp"
#include "robstride_driver/can_interface.hpp"
#include "robstride_driver/protocol.hpp"

namespace robstride {

/// In-process CAN transport that simulates RobStride motors, for running
/// applications without hardware.
///
/// Frames sent through `Send` are decoded and answered on the next
/// `Receive` call, exactly like a real bus: enable/stop and parameter
/// writes are answered with feedback frames, parameter reads with type-17
/// responses. One instance simulates every motor id addressed through it,
/// so it can be shared by multiple `RobstrideMotor` instances or used for
/// a single axis while other axes use a real transport.
///
/// Simulated motion model:
/// - velocity mode: the commanded `spd_ref` is integrated into the
///   position while the motor is enabled;
/// - CSP position mode: the target `loc_ref` is reached instantly;
/// - the feedback position wraps at the actuator range like real
///   hardware, so `PositionUnwrapper` behaves identically.
///
/// Like the real transports, this class is not thread-safe.
class StubCanInterface : public CanInterface {
 public:
  /// Creates a stub bus. `actuator_type` selects the fixed-point scaling
  /// used to encode feedback frames; it must match the type configured in
  /// the `RobstrideMotor` instances using this interface.
  explicit StubCanInterface(ActuatorType actuator_type = ActuatorType::kRs02);

  /// Decodes the command frame, updates the simulated motor state, and
  /// queues the response frame(s) for `Receive`. Unsupported communication
  /// types are ignored (the caller then observes a response timeout).
  void Send(const CanFrame& frame) override;

  /// Returns the next queued response, or nullopt immediately when no
  /// response is pending (simulating a response timeout without waiting).
  std::optional<CanFrame> Receive(std::chrono::milliseconds timeout) override;

  /// Sets the temperature [Celsius] reported in the feedback frames of
  /// every simulated motor (default 30.0).
  void set_temperature(double temperature) { temperature_ = temperature; }

  /// Sets the raw fault bits (bit21-16 layout of the feedback identifier)
  /// reported by the motor `motor_id`, for fault-handling tests.
  void set_fault_bits(std::uint8_t motor_id, std::uint8_t fault_bits);

 private:
  /// Simulated state of one motor id.
  struct MotorState {
    bool enabled = false;                               ///< enabled by type 3
    std::uint8_t run_mode = 0;                          ///< parameter 0x7005
    double position = 0.0;                              ///< continuous [rad]
    std::uint8_t fault_bits = 0;                        ///< injected fault bits
    std::map<std::uint16_t, float> params;              ///< float parameters
    std::chrono::steady_clock::time_point last_update;  ///< sim timestamp
    bool has_last_update = false;  ///< last_update validity
  };

  /// Returns the state of `motor_id`, creating it on first use.
  MotorState& Motor(std::uint8_t motor_id);

  /// Advances the motion simulation of `state` up to `now`.
  static void Advance(MotorState& state,
                      std::chrono::steady_clock::time_point now);

  /// Queues the feedback frame reflecting the current state of `state`.
  void QueueFeedback(std::uint8_t motor_id, std::uint8_t host_id,
                     const MotorState& state);

  /// Fixed-point scaling ranges used to encode feedback frames.
  const ActuatorLimits& limits_;

  /// Simulated motors keyed by motor id.
  std::map<std::uint8_t, MotorState> motors_;

  /// Responses queued by `Send`, drained by `Receive`.
  std::deque<CanFrame> responses_;

  /// [Celsius] temperature reported in feedback frames.
  double temperature_ = 30.0;
};

}  // namespace robstride
