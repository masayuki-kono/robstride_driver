// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/robstride_motor.hpp"

#include <stdexcept>
#include <string>

namespace robstride {

namespace {

std::string describe_timeout(std::uint8_t motor_id, CommType expected) {
  return "robstride: motor 0x" + std::to_string(motor_id) +
         " did not answer (expected communication type " +
         std::to_string(static_cast<int>(expected)) + ")";
}

ParamResponse parse_param_response_or_throw(const CanFrame& frame) {
  const auto response = parse_param_response(frame);
  if (!response) {
    throw std::runtime_error("robstride: malformed parameter response frame");
  }
  return *response;
}

}  // namespace

RobstrideMotor::RobstrideMotor(std::shared_ptr<CanInterface> can,
                               const Config& config)
    : can_(std::move(can)),
      config_(config),
      limits_(get_actuator_limits(config.actuator_type)) {}

CanFrame RobstrideMotor::transceive(const CanFrame& frame,
                                    CommType expected_type) {
  can_->send(frame);

  const auto deadline =
      std::chrono::steady_clock::now() + config_.response_timeout;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      throw TimeoutError(describe_timeout(config_.motor_id, expected_type));
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const auto received = can_->receive(remaining);
    if (!received) {
      throw TimeoutError(describe_timeout(config_.motor_id, expected_type));
    }
    if (get_source_motor_id(*received) != config_.motor_id) {
      continue;  // frame from another motor on the same bus
    }
    // Opportunistically cache any feedback frame we see.
    if (auto feedback = parse_feedback(*received, limits_)) {
      last_feedback_ = *feedback;
    }
    if (get_comm_type(*received) == static_cast<std::uint8_t>(expected_type)) {
      return *received;
    }
  }
}

Feedback RobstrideMotor::transceive_feedback(const CanFrame& frame) {
  const CanFrame response = transceive(frame, CommType::Feedback);
  const auto feedback = parse_feedback(response, limits_);
  if (!feedback) {
    throw std::runtime_error("robstride: malformed feedback frame");
  }
  return *feedback;
}

Feedback RobstrideMotor::enable() {
  return transceive_feedback(
      make_enable_frame(config_.motor_id, config_.host_id));
}

Feedback RobstrideMotor::disable(bool clear_fault) {
  return transceive_feedback(
      make_stop_frame(config_.motor_id, config_.host_id, clear_fault));
}

void RobstrideMotor::set_run_mode(RunMode mode) {
  disable();
  write_param(param_index::run_mode, static_cast<std::uint8_t>(mode));
  const RunMode actual = get_run_mode();
  if (actual != mode) {
    throw std::runtime_error(
        "robstride: run_mode verification failed (requested " +
        std::to_string(static_cast<int>(mode)) + ", got " +
        std::to_string(static_cast<int>(actual)) + ")");
  }
}

RunMode RobstrideMotor::get_run_mode() {
  return static_cast<RunMode>(read_param_uint8(param_index::run_mode));
}

void RobstrideMotor::configure_velocity_mode(double current_limit,
                                             double acceleration) {
  write_param(param_index::limit_cur, static_cast<float>(current_limit));
  write_param(param_index::acc_rad, static_cast<float>(acceleration));
}

Feedback RobstrideMotor::send_velocity_command(double velocity) {
  return write_param(param_index::spd_ref, static_cast<float>(velocity));
}

Feedback RobstrideMotor::send_position_csp_command(double position,
                                                   double speed_limit) {
  write_param(param_index::limit_spd, static_cast<float>(speed_limit));
  return write_param(param_index::loc_ref, static_cast<float>(position));
}

Feedback RobstrideMotor::send_motion_command(double torque, double position,
                                             double velocity, double kp,
                                             double kd) {
  return transceive_feedback(make_motion_control_frame(
      config_.motor_id, torque, position, velocity, kp, kd, limits_));
}

void RobstrideMotor::set_mechanical_zero() {
  disable();
  transceive_feedback(
      make_set_mechanical_zero_frame(config_.motor_id, config_.host_id));
}

float RobstrideMotor::read_param_float(std::uint16_t index) {
  const CanFrame response = transceive(
      make_read_param_frame(config_.motor_id, config_.host_id, index),
      CommType::ReadParam);
  return parse_param_response_or_throw(response).as_float();
}

std::uint8_t RobstrideMotor::read_param_uint8(std::uint16_t index) {
  const CanFrame response = transceive(
      make_read_param_frame(config_.motor_id, config_.host_id, index),
      CommType::ReadParam);
  return parse_param_response_or_throw(response).as_uint8();
}

Feedback RobstrideMotor::write_param(std::uint16_t index, float value) {
  return transceive_feedback(
      make_write_param_frame(config_.motor_id, config_.host_id, index, value));
}

Feedback RobstrideMotor::write_param(std::uint16_t index, std::uint8_t value) {
  return transceive_feedback(
      make_write_param_frame(config_.motor_id, config_.host_id, index, value));
}

}  // namespace robstride
