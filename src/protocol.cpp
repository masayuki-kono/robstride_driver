// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>

namespace robstride {

namespace {

constexpr double pi = std::numbers::pi;

// Ranges from the vendor sample (robstride_ros_sample). RS02 values match
// the RS02 User Manual: P +-4*pi (=12.57) rad, V +-44 rad/s, T +-17 Nm,
// Kp 0-500, Kd 0-5.
constexpr std::array<ActuatorLimits, 7> limits_table = {{
    /* RS00 */ {4 * pi, 50.0, 17.0, 500.0, 5.0},
    /* RS01 */ {4 * pi, 44.0, 17.0, 500.0, 5.0},
    /* RS02 */ {4 * pi, 44.0, 17.0, 500.0, 5.0},
    /* RS03 */ {4 * pi, 50.0, 60.0, 5000.0, 100.0},
    /* RS04 */ {4 * pi, 15.0, 120.0, 5000.0, 100.0},
    /* RS05 */ {4 * pi, 33.0, 17.0, 500.0, 5.0},
    /* RS06 */ {4 * pi, 20.0, 36.0, 5000.0, 100.0},
}};

std::uint32_t make_id(CommType type, std::uint16_t data_area2,
                      std::uint8_t dest) {
  return (static_cast<std::uint32_t>(type) << 24) |
         (static_cast<std::uint32_t>(data_area2) << 8) | dest;
}

}  // namespace

const ActuatorLimits& get_actuator_limits(ActuatorType type) {
  const auto index = static_cast<std::size_t>(type);
  if (index >= limits_table.size()) {
    throw std::out_of_range("robstride: unknown actuator type");
  }
  return limits_table[index];
}

float ParamResponse::as_float() const {
  float value = 0.0F;
  std::memcpy(&value, data.data(), sizeof(value));
  return value;
}

float ParamWriteRequest::as_float() const {
  float value = 0.0F;
  std::memcpy(&value, data.data(), sizeof(value));
  return value;
}

std::uint16_t float_to_uint(double x, double min, double max, int bits) {
  x = std::clamp(x, min, max);
  const double span = max - min;
  const double scaled = (x - min) * ((1 << bits) - 1) / span;
  return static_cast<std::uint16_t>(std::lround(scaled));
}

double uint_to_float(std::uint16_t value, double min, double max, int bits) {
  const double span = max - min;
  return (static_cast<double>(value) * span / ((1 << bits) - 1)) + min;
}

CanFrame make_enable_frame(std::uint8_t motor_id, std::uint8_t host_id) {
  CanFrame frame;
  frame.id = make_id(CommType::Enable, host_id, motor_id);
  return frame;
}

CanFrame make_stop_frame(std::uint8_t motor_id, std::uint8_t host_id,
                         bool clear_fault) {
  CanFrame frame;
  frame.id = make_id(CommType::Stop, host_id, motor_id);
  frame.data[0] = clear_fault ? 1 : 0;
  return frame;
}

CanFrame make_set_mechanical_zero_frame(std::uint8_t motor_id,
                                        std::uint8_t host_id) {
  CanFrame frame;
  frame.id = make_id(CommType::SetMechanicalZero, host_id, motor_id);
  frame.data[0] = 1;
  return frame;
}

CanFrame make_set_can_id_frame(std::uint8_t motor_id, std::uint8_t host_id,
                               std::uint8_t new_motor_id) {
  CanFrame frame;
  const std::uint16_t data_area2 =
      static_cast<std::uint16_t>(new_motor_id) << 8 | host_id;
  frame.id = make_id(CommType::SetCanId, data_area2, motor_id);
  return frame;
}

CanFrame make_get_device_id_frame(std::uint8_t motor_id, std::uint8_t host_id) {
  CanFrame frame;
  frame.id = make_id(CommType::GetDeviceId, host_id, motor_id);
  return frame;
}

CanFrame make_read_param_frame(std::uint8_t motor_id, std::uint8_t host_id,
                               std::uint16_t index) {
  CanFrame frame;
  frame.id = make_id(CommType::ReadParam, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  return frame;
}

namespace {

CanFrame make_write_param_frame_base(std::uint8_t motor_id,
                                     std::uint8_t host_id,
                                     std::uint16_t index) {
  CanFrame frame;
  frame.id = make_id(CommType::WriteParam, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  return frame;
}

}  // namespace

CanFrame make_write_param_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                std::uint16_t index, float value) {
  CanFrame frame = make_write_param_frame_base(motor_id, host_id, index);
  std::memcpy(&frame.data[4], &value, sizeof(value));
  return frame;
}

CanFrame make_write_param_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                std::uint16_t index, std::uint8_t value) {
  CanFrame frame = make_write_param_frame_base(motor_id, host_id, index);
  frame.data[4] = value;
  return frame;
}

CanFrame make_motion_control_frame(std::uint8_t motor_id, double torque,
                                   double position, double velocity, double kp,
                                   double kd, const ActuatorLimits& limits) {
  CanFrame frame;
  const std::uint16_t torque_u =
      float_to_uint(torque, -limits.torque, limits.torque);
  frame.id = make_id(CommType::MotionControl, torque_u, motor_id);

  const std::uint16_t pos_u =
      float_to_uint(position, -limits.position, limits.position);
  const std::uint16_t vel_u =
      float_to_uint(velocity, -limits.velocity, limits.velocity);
  const std::uint16_t kp_u = float_to_uint(kp, 0.0, limits.kp_max);
  const std::uint16_t kd_u = float_to_uint(kd, 0.0, limits.kd_max);

  frame.data[0] = static_cast<std::uint8_t>(pos_u >> 8);
  frame.data[1] = static_cast<std::uint8_t>(pos_u & 0xFF);
  frame.data[2] = static_cast<std::uint8_t>(vel_u >> 8);
  frame.data[3] = static_cast<std::uint8_t>(vel_u & 0xFF);
  frame.data[4] = static_cast<std::uint8_t>(kp_u >> 8);
  frame.data[5] = static_cast<std::uint8_t>(kp_u & 0xFF);
  frame.data[6] = static_cast<std::uint8_t>(kd_u >> 8);
  frame.data[7] = static_cast<std::uint8_t>(kd_u & 0xFF);
  return frame;
}

CanFrame make_feedback_frame(const Feedback& feedback,
                             const ActuatorLimits& limits) {
  CanFrame frame;
  const std::uint16_t data_area2 =
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(feedback.mode)
                                 << 14) |
      static_cast<std::uint16_t>((feedback.fault.raw & 0x3F) << 8) |
      feedback.motor_id;
  frame.id = make_id(CommType::Feedback, data_area2, feedback.host_id);

  const std::uint16_t pos_u =
      float_to_uint(feedback.position, -limits.position, limits.position);
  const std::uint16_t vel_u =
      float_to_uint(feedback.velocity, -limits.velocity, limits.velocity);
  const std::uint16_t torque_u =
      float_to_uint(feedback.torque, -limits.torque, limits.torque);
  const auto temp_u =
      static_cast<std::uint16_t>(std::lround(feedback.temperature * 10.0));

  frame.data[0] = static_cast<std::uint8_t>(pos_u >> 8);
  frame.data[1] = static_cast<std::uint8_t>(pos_u & 0xFF);
  frame.data[2] = static_cast<std::uint8_t>(vel_u >> 8);
  frame.data[3] = static_cast<std::uint8_t>(vel_u & 0xFF);
  frame.data[4] = static_cast<std::uint8_t>(torque_u >> 8);
  frame.data[5] = static_cast<std::uint8_t>(torque_u & 0xFF);
  frame.data[6] = static_cast<std::uint8_t>(temp_u >> 8);
  frame.data[7] = static_cast<std::uint8_t>(temp_u & 0xFF);
  return frame;
}

namespace {

CanFrame make_param_response_frame_base(std::uint8_t motor_id,
                                        std::uint8_t host_id,
                                        std::uint16_t index) {
  CanFrame frame;
  frame.id = make_id(CommType::ReadParam, motor_id, host_id);
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  return frame;
}

}  // namespace

CanFrame make_param_response_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                   std::uint16_t index, float value) {
  CanFrame frame = make_param_response_frame_base(motor_id, host_id, index);
  std::memcpy(&frame.data[4], &value, sizeof(value));
  return frame;
}

CanFrame make_param_response_frame(std::uint8_t motor_id, std::uint8_t host_id,
                                   std::uint16_t index, std::uint8_t value) {
  CanFrame frame = make_param_response_frame_base(motor_id, host_id, index);
  frame.data[4] = value;
  return frame;
}

std::uint8_t get_comm_type(const CanFrame& frame) {
  return static_cast<std::uint8_t>((frame.id >> 24) & 0x1F);
}

std::uint8_t get_source_motor_id(const CanFrame& frame) {
  return static_cast<std::uint8_t>((frame.id >> 8) & 0xFF);
}

std::uint8_t get_target_motor_id(const CanFrame& frame) {
  return static_cast<std::uint8_t>(frame.id & 0xFF);
}

std::uint8_t get_host_id(const CanFrame& frame) {
  return static_cast<std::uint8_t>((frame.id >> 8) & 0xFF);
}

std::optional<ParamReadRequest> parse_param_read_request(
    const CanFrame& frame) {
  if (get_comm_type(frame) != static_cast<std::uint8_t>(CommType::ReadParam)) {
    return std::nullopt;
  }

  ParamReadRequest request;
  request.motor_id = get_target_motor_id(frame);
  request.host_id = get_host_id(frame);
  request.index =
      static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8));
  return request;
}

std::optional<ParamWriteRequest> parse_param_write_request(
    const CanFrame& frame) {
  if (get_comm_type(frame) != static_cast<std::uint8_t>(CommType::WriteParam)) {
    return std::nullopt;
  }

  ParamWriteRequest request;
  request.motor_id = get_target_motor_id(frame);
  request.host_id = get_host_id(frame);
  request.index =
      static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8));
  std::copy(frame.data.begin() + 4, frame.data.begin() + 8,
            request.data.begin());
  return request;
}

std::optional<Feedback> parse_feedback(const CanFrame& frame,
                                       const ActuatorLimits& limits) {
  if (get_comm_type(frame) != static_cast<std::uint8_t>(CommType::Feedback)) {
    return std::nullopt;
  }
  if (frame.dlc < 8) {
    return std::nullopt;
  }

  Feedback feedback;
  feedback.motor_id = get_source_motor_id(frame);
  feedback.host_id = static_cast<std::uint8_t>(frame.id & 0xFF);
  feedback.mode = static_cast<MotorMode>((frame.id >> 22) & 0x03);
  feedback.fault.raw = static_cast<std::uint8_t>((frame.id >> 16) & 0x3F);

  const auto pos_u =
      static_cast<std::uint16_t>((frame.data[0] << 8) | frame.data[1]);
  const auto vel_u =
      static_cast<std::uint16_t>((frame.data[2] << 8) | frame.data[3]);
  const auto torque_u =
      static_cast<std::uint16_t>((frame.data[4] << 8) | frame.data[5]);
  const auto temp_u =
      static_cast<std::uint16_t>((frame.data[6] << 8) | frame.data[7]);

  feedback.position = uint_to_float(pos_u, -limits.position, limits.position);
  feedback.velocity = uint_to_float(vel_u, -limits.velocity, limits.velocity);
  feedback.torque = uint_to_float(torque_u, -limits.torque, limits.torque);
  feedback.temperature = static_cast<double>(temp_u) * 0.1;
  return feedback;
}

std::optional<ParamResponse> parse_param_response(const CanFrame& frame) {
  if (get_comm_type(frame) != static_cast<std::uint8_t>(CommType::ReadParam)) {
    return std::nullopt;
  }
  if (frame.dlc < 8) {
    return std::nullopt;
  }

  ParamResponse response;
  response.motor_id = get_source_motor_id(frame);
  response.index =
      static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8));
  std::copy(frame.data.begin() + 4, frame.data.begin() + 8,
            response.data.begin());
  return response;
}

}  // namespace robstride
