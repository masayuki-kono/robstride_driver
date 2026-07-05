// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/stub_can_interface.hpp"

#include <cmath>

namespace robstride {

namespace {

float ParamOrZero(const std::map<std::uint16_t, float>& params,
                  std::uint16_t index) {
  const auto it = params.find(index);
  return it != params.end() ? it->second : 0.0F;
}

}  // namespace

StubCanInterface::StubCanInterface(ActuatorType actuator_type)
    : limits_(GetActuatorLimits(actuator_type)) {}

void StubCanInterface::Send(const CanFrame& frame) {
  const auto now = std::chrono::steady_clock::now();
  const std::uint8_t motor_id = GetTargetMotorId(frame);
  const std::uint8_t host_id = GetHostId(frame);
  const auto comm_type = static_cast<CommType>(GetCommType(frame));

  switch (comm_type) {
    case CommType::kEnable: {
      MotorState& state = Motor(motor_id);
      Advance(state, now);
      state.enabled = true;
      QueueFeedback(motor_id, host_id, state);
      break;
    }
    case CommType::kStop: {
      MotorState& state = Motor(motor_id);
      Advance(state, now);
      state.enabled = false;
      if (frame.data[0] == 1) {
        state.fault_bits = 0;
      }
      QueueFeedback(motor_id, host_id, state);
      break;
    }
    case CommType::kSetMechanicalZero: {
      MotorState& state = Motor(motor_id);
      Advance(state, now);
      state.position = 0.0;
      QueueFeedback(motor_id, host_id, state);
      break;
    }
    case CommType::kMotionControl: {
      // The MIT motion model is not simulated; only the feedback exchange.
      MotorState& state = Motor(motor_id);
      Advance(state, now);
      QueueFeedback(motor_id, host_id, state);
      break;
    }
    case CommType::kReadParam: {
      const auto request = ParseParamReadRequest(frame);
      if (!request) {
        break;
      }
      MotorState& state = Motor(motor_id);
      if (request->index == param_index::kRunMode) {
        responses_.push_back(MakeParamResponseFrame(
            motor_id, host_id, request->index, state.run_mode));
      } else {
        responses_.push_back(
            MakeParamResponseFrame(motor_id, host_id, request->index,
                                   ParamOrZero(state.params, request->index)));
      }
      break;
    }
    case CommType::kWriteParam: {
      const auto request = ParseParamWriteRequest(frame);
      if (!request) {
        break;
      }
      MotorState& state = Motor(motor_id);
      Advance(state, now);
      if (request->index == param_index::kRunMode) {
        state.run_mode = request->AsUint8();
      } else {
        state.params[request->index] = request->AsFloat();
      }
      if (request->index == param_index::kLocRef && state.enabled &&
          state.run_mode == static_cast<std::uint8_t>(RunMode::kPositionCsp)) {
        // Reach the target instantly: move to the nearest continuous
        // position whose wrapped value equals loc_ref.
        state.position += std::remainder(
            static_cast<double>(request->AsFloat()) - state.position,
            2.0 * limits_.position);
      }
      QueueFeedback(motor_id, host_id, state);
      break;
    }
    default:
      // Unsupported communication types get no response; the caller
      // observes a response timeout, as with a real silent motor.
      break;
  }
}

std::optional<CanFrame> StubCanInterface::Receive(
    std::chrono::milliseconds /*timeout*/) {
  if (responses_.empty()) {
    return std::nullopt;
  }
  const CanFrame frame = responses_.front();
  responses_.pop_front();
  return frame;
}

void StubCanInterface::set_fault_bits(std::uint8_t motor_id,
                                      std::uint8_t fault_bits) {
  Motor(motor_id).fault_bits = fault_bits;
}

StubCanInterface::MotorState& StubCanInterface::Motor(std::uint8_t motor_id) {
  return motors_[motor_id];
}

void StubCanInterface::Advance(MotorState& state,
                               std::chrono::steady_clock::time_point now) {
  if (state.has_last_update && state.enabled &&
      state.run_mode == static_cast<std::uint8_t>(RunMode::kVelocity)) {
    const double dt =
        std::chrono::duration<double>(now - state.last_update).count();
    state.position +=
        static_cast<double>(ParamOrZero(state.params, param_index::kSpdRef)) *
        dt;
  }
  state.last_update = now;
  state.has_last_update = true;
}

void StubCanInterface::QueueFeedback(std::uint8_t motor_id,
                                     std::uint8_t host_id,
                                     const MotorState& state) {
  const bool moving =
      state.enabled &&
      state.run_mode == static_cast<std::uint8_t>(RunMode::kVelocity);

  Feedback feedback;
  feedback.motor_id = motor_id;
  feedback.host_id = host_id;
  feedback.mode = state.enabled ? MotorMode::kRun : MotorMode::kReset;
  feedback.fault.raw = state.fault_bits;
  feedback.position = std::remainder(state.position, 2.0 * limits_.position);
  feedback.velocity =
      moving
          ? static_cast<double>(ParamOrZero(state.params, param_index::kSpdRef))
          : 0.0;
  feedback.torque = 0.0;
  feedback.temperature = temperature_;
  responses_.push_back(MakeFeedbackFrame(feedback, limits_));
}

}  // namespace robstride
