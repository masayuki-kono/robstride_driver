// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/robstride_motor.hpp"

#include <string>

namespace robstride {

namespace {

std::string DescribeTimeout(std::uint8_t motor_id, CommType expected) {
  return "robstride: motor 0x" + std::to_string(motor_id) +
         " did not answer (expected communication type " +
         std::to_string(static_cast<int>(expected)) + ")";
}

}  // namespace

RobstrideMotor::RobstrideMotor(std::shared_ptr<CanInterface> can,
                               const Config& config)
    : can_(std::move(can)),
      config_(config),
      limits_(GetActuatorLimits(config.actuator_type)) {}

CanFrame RobstrideMotor::Transceive(const CanFrame& frame,
                                    CommType expected_type) {
  can_->Send(frame);

  const auto deadline =
      std::chrono::steady_clock::now() + config_.response_timeout;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      throw TimeoutError(DescribeTimeout(config_.motor_id, expected_type));
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const auto received = can_->Receive(remaining);
    if (!received) {
      throw TimeoutError(DescribeTimeout(config_.motor_id, expected_type));
    }
    if (GetSourceMotorId(*received) != config_.motor_id) {
      continue;  // frame from another motor on the same bus
    }
    // Opportunistically cache any feedback frame we see.
    if (auto feedback = ParseFeedback(*received, limits_)) {
      last_feedback_ = *feedback;
    }
    if (GetCommType(*received) == static_cast<std::uint8_t>(expected_type)) {
      return *received;
    }
  }
}

Feedback RobstrideMotor::TransceiveFeedback(const CanFrame& frame) {
  const CanFrame response = Transceive(frame, CommType::kFeedback);
  return *ParseFeedback(response, limits_);
}

Feedback RobstrideMotor::Enable() {
  return TransceiveFeedback(MakeEnableFrame(config_.motor_id, config_.host_id));
}

Feedback RobstrideMotor::Disable(bool clear_fault) {
  return TransceiveFeedback(
      MakeStopFrame(config_.motor_id, config_.host_id, clear_fault));
}

void RobstrideMotor::SetRunMode(RunMode mode) {
  Disable();
  WriteParam(param_index::kRunMode, static_cast<std::uint8_t>(mode));
  const RunMode actual = GetRunMode();
  if (actual != mode) {
    throw std::runtime_error(
        "robstride: run_mode verification failed (requested " +
        std::to_string(static_cast<int>(mode)) + ", got " +
        std::to_string(static_cast<int>(actual)) + ")");
  }
}

RunMode RobstrideMotor::GetRunMode() {
  return static_cast<RunMode>(ReadParamUint8(param_index::kRunMode));
}

void RobstrideMotor::ConfigureVelocityMode(double current_limit,
                                           double acceleration) {
  WriteParam(param_index::kLimitCur, static_cast<float>(current_limit));
  WriteParam(param_index::kAccRad, static_cast<float>(acceleration));
}

Feedback RobstrideMotor::SendVelocityCommand(double velocity) {
  return WriteParam(param_index::kSpdRef, static_cast<float>(velocity));
}

Feedback RobstrideMotor::SendPositionCspCommand(double position,
                                                double speed_limit) {
  WriteParam(param_index::kLimitSpd, static_cast<float>(speed_limit));
  return WriteParam(param_index::kLocRef, static_cast<float>(position));
}

Feedback RobstrideMotor::SendMotionCommand(double torque, double position,
                                           double velocity, double kp,
                                           double kd) {
  return TransceiveFeedback(MakeMotionControlFrame(
      config_.motor_id, torque, position, velocity, kp, kd, limits_));
}

void RobstrideMotor::SetMechanicalZero() {
  Disable();
  TransceiveFeedback(
      MakeSetMechanicalZeroFrame(config_.motor_id, config_.host_id));
}

float RobstrideMotor::ReadParamFloat(std::uint16_t index) {
  const CanFrame response =
      Transceive(MakeReadParamFrame(config_.motor_id, config_.host_id, index),
                 CommType::kReadParam);
  return ParseParamResponse(response)->AsFloat();
}

std::uint8_t RobstrideMotor::ReadParamUint8(std::uint16_t index) {
  const CanFrame response =
      Transceive(MakeReadParamFrame(config_.motor_id, config_.host_id, index),
                 CommType::kReadParam);
  return ParseParamResponse(response)->AsUint8();
}

Feedback RobstrideMotor::WriteParam(std::uint16_t index, float value) {
  return TransceiveFeedback(
      MakeWriteParamFrame(config_.motor_id, config_.host_id, index, value));
}

Feedback RobstrideMotor::WriteParam(std::uint16_t index, std::uint8_t value) {
  return TransceiveFeedback(
      MakeWriteParamFrame(config_.motor_id, config_.host_id, index, value));
}

}  // namespace robstride
