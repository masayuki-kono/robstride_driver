// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/stub_can_interface.hpp"

#include <cmath>

namespace robstride {

namespace {

float param_or_zero(const std::map<std::uint16_t, float>& params,
                    std::uint16_t index) {
  const auto it = params.find(index);
  return it != params.end() ? it->second : 0.0F;
}

}  // namespace

StubCanInterface::StubCanInterface(ActuatorType actuator_type)
    : limits_(get_actuator_limits(actuator_type)) {}

void StubCanInterface::send(const CanFrame& frame) {
  const auto now = std::chrono::steady_clock::now();
  const std::uint8_t motor_id = get_target_motor_id(frame);
  const std::uint8_t host_id = get_host_id(frame);
  const auto comm_type = static_cast<CommType>(get_comm_type(frame));

  switch (comm_type) {
    case CommType::Enable: {
      MotorState& state = motor(motor_id);
      advance(state, now);
      state.enabled = true;
      queue_feedback(motor_id, host_id, state);
      break;
    }
    case CommType::Stop: {
      MotorState& state = motor(motor_id);
      advance(state, now);
      state.enabled = false;
      if (frame.data[0] == 1) {
        state.fault_bits = 0;
      }
      queue_feedback(motor_id, host_id, state);
      break;
    }
    case CommType::SetMechanicalZero: {
      MotorState& state = motor(motor_id);
      advance(state, now);
      state.position = 0.0;
      queue_feedback(motor_id, host_id, state);
      break;
    }
    case CommType::MotionControl: {
      // The MIT motion model is not simulated; only the feedback exchange.
      MotorState& state = motor(motor_id);
      advance(state, now);
      queue_feedback(motor_id, host_id, state);
      break;
    }
    case CommType::ReadParam: {
      const auto request = parse_param_read_request(frame);
      if (!request) {
        break;
      }
      MotorState& state = motor(motor_id);
      if (request->index == param_index::run_mode) {
        responses_.push_back(make_param_response_frame(
            motor_id, host_id, request->index, state.run_mode));
      } else {
        responses_.push_back(make_param_response_frame(
            motor_id, host_id, request->index,
            param_or_zero(state.params, request->index)));
      }
      break;
    }
    case CommType::WriteParam: {
      const auto request = parse_param_write_request(frame);
      if (!request) {
        break;
      }
      MotorState& state = motor(motor_id);
      advance(state, now);
      if (request->index == param_index::run_mode) {
        state.run_mode = request->as_uint8();
      } else {
        state.params[request->index] = request->as_float();
      }
      if (request->index == param_index::loc_ref && state.enabled &&
          state.run_mode == static_cast<std::uint8_t>(RunMode::PositionCsp)) {
        // Reach the target instantly: move to the nearest continuous
        // position whose wrapped value equals loc_ref.
        state.position += std::remainder(
            static_cast<double>(request->as_float()) - state.position,
            2.0 * limits_.position);
      }
      queue_feedback(motor_id, host_id, state);
      break;
    }
    default:
      // Unsupported communication types get no response; the caller
      // observes a response timeout, as with a real silent motor.
      break;
  }
}

std::optional<CanFrame> StubCanInterface::receive(
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
  motor(motor_id).fault_bits = fault_bits;
}

StubCanInterface::MotorState& StubCanInterface::motor(std::uint8_t motor_id) {
  return motors_[motor_id];
}

void StubCanInterface::advance(MotorState& state,
                               std::chrono::steady_clock::time_point now) {
  if (state.has_last_update && state.enabled &&
      state.run_mode == static_cast<std::uint8_t>(RunMode::Velocity)) {
    const double dt =
        std::chrono::duration<double>(now - state.last_update).count();
    state.position +=
        static_cast<double>(param_or_zero(state.params, param_index::spd_ref)) *
        dt;
  }
  state.last_update = now;
  state.has_last_update = true;
}

void StubCanInterface::queue_feedback(std::uint8_t motor_id,
                                      std::uint8_t host_id,
                                      const MotorState& state) {
  const bool moving =
      state.enabled &&
      state.run_mode == static_cast<std::uint8_t>(RunMode::Velocity);

  Feedback feedback;
  feedback.motor_id = motor_id;
  feedback.host_id = host_id;
  feedback.mode = state.enabled ? MotorMode::Run : MotorMode::Reset;
  feedback.fault.raw = state.fault_bits;
  feedback.position = std::remainder(state.position, 2.0 * limits_.position);
  feedback.velocity = moving ? static_cast<double>(param_or_zero(
                                   state.params, param_index::spd_ref))
                             : 0.0;
  feedback.torque = 0.0;
  feedback.temperature = temperature_;
  responses_.push_back(make_feedback_frame(feedback, limits_));
}

}  // namespace robstride
