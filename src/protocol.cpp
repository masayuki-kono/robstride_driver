// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace robstride {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Ranges from the vendor sample (robstride_ros_sample). RS02 values match
// the RS02 User Manual: P +-4*pi (=12.57) rad, V +-44 rad/s, T +-17 Nm,
// Kp 0-500, Kd 0-5.
constexpr ActuatorLimits kLimitsTable[] = {
    /* RS00 */ {4 * kPi, 50.0, 17.0, 500.0, 5.0},
    /* RS01 */ {4 * kPi, 44.0, 17.0, 500.0, 5.0},
    /* RS02 */ {4 * kPi, 44.0, 17.0, 500.0, 5.0},
    /* RS03 */ {4 * kPi, 50.0, 60.0, 5000.0, 100.0},
    /* RS04 */ {4 * kPi, 15.0, 120.0, 5000.0, 100.0},
    /* RS05 */ {4 * kPi, 33.0, 17.0, 500.0, 5.0},
    /* RS06 */ {4 * kPi, 20.0, 36.0, 5000.0, 100.0},
};

std::uint32_t MakeId(CommType type, std::uint16_t data_area2,
                     std::uint8_t dest) {
  return (static_cast<std::uint32_t>(type) << 24) |
         (static_cast<std::uint32_t>(data_area2) << 8) | dest;
}

}  // namespace

const ActuatorLimits& GetActuatorLimits(ActuatorType type) {
  const auto index = static_cast<std::size_t>(type);
  if (index >= std::size(kLimitsTable)) {
    throw std::out_of_range("robstride: unknown actuator type");
  }
  return kLimitsTable[index];
}

float ParamResponse::AsFloat() const {
  float value = 0.0F;
  std::memcpy(&value, data.data(), sizeof(value));
  return value;
}

std::uint16_t FloatToUint(double x, double min, double max, int bits) {
  x = std::clamp(x, min, max);
  const double span = max - min;
  const double scaled = (x - min) * ((1 << bits) - 1) / span;
  return static_cast<std::uint16_t>(std::lround(scaled));
}

double UintToFloat(std::uint16_t value, double min, double max, int bits) {
  const double span = max - min;
  return static_cast<double>(value) * span / ((1 << bits) - 1) + min;
}

CanFrame MakeEnableFrame(std::uint8_t motor_id, std::uint8_t host_id) {
  CanFrame frame;
  frame.id = MakeId(CommType::kEnable, host_id, motor_id);
  return frame;
}

CanFrame MakeStopFrame(std::uint8_t motor_id, std::uint8_t host_id,
                       bool clear_fault) {
  CanFrame frame;
  frame.id = MakeId(CommType::kStop, host_id, motor_id);
  frame.data[0] = clear_fault ? 1 : 0;
  return frame;
}

CanFrame MakeSetMechanicalZeroFrame(std::uint8_t motor_id,
                                    std::uint8_t host_id) {
  CanFrame frame;
  frame.id = MakeId(CommType::kSetMechanicalZero, host_id, motor_id);
  frame.data[0] = 1;
  return frame;
}

CanFrame MakeSetCanIdFrame(std::uint8_t motor_id, std::uint8_t host_id,
                           std::uint8_t new_motor_id) {
  CanFrame frame;
  const std::uint16_t data_area2 =
      static_cast<std::uint16_t>(new_motor_id) << 8 | host_id;
  frame.id = MakeId(CommType::kSetCanId, data_area2, motor_id);
  return frame;
}

CanFrame MakeGetDeviceIdFrame(std::uint8_t motor_id, std::uint8_t host_id) {
  CanFrame frame;
  frame.id = MakeId(CommType::kGetDeviceId, host_id, motor_id);
  return frame;
}

CanFrame MakeReadParamFrame(std::uint8_t motor_id, std::uint8_t host_id,
                            std::uint16_t index) {
  CanFrame frame;
  frame.id = MakeId(CommType::kReadParam, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  return frame;
}

namespace {

CanFrame MakeWriteParamFrameBase(std::uint8_t motor_id, std::uint8_t host_id,
                                 std::uint16_t index) {
  CanFrame frame;
  frame.id = MakeId(CommType::kWriteParam, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  return frame;
}

}  // namespace

CanFrame MakeWriteParamFrame(std::uint8_t motor_id, std::uint8_t host_id,
                             std::uint16_t index, float value) {
  CanFrame frame = MakeWriteParamFrameBase(motor_id, host_id, index);
  std::memcpy(&frame.data[4], &value, sizeof(value));
  return frame;
}

CanFrame MakeWriteParamFrame(std::uint8_t motor_id, std::uint8_t host_id,
                             std::uint16_t index, std::uint8_t value) {
  CanFrame frame = MakeWriteParamFrameBase(motor_id, host_id, index);
  frame.data[4] = value;
  return frame;
}

CanFrame MakeMotionControlFrame(std::uint8_t motor_id, double torque,
                                double position, double velocity, double kp,
                                double kd, const ActuatorLimits& limits) {
  CanFrame frame;
  const std::uint16_t torque_u =
      FloatToUint(torque, -limits.torque, limits.torque);
  frame.id = MakeId(CommType::kMotionControl, torque_u, motor_id);

  const std::uint16_t pos_u =
      FloatToUint(position, -limits.position, limits.position);
  const std::uint16_t vel_u =
      FloatToUint(velocity, -limits.velocity, limits.velocity);
  const std::uint16_t kp_u = FloatToUint(kp, 0.0, limits.kp_max);
  const std::uint16_t kd_u = FloatToUint(kd, 0.0, limits.kd_max);

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

std::uint8_t GetCommType(const CanFrame& frame) {
  return static_cast<std::uint8_t>((frame.id >> 24) & 0x1F);
}

std::uint8_t GetSourceMotorId(const CanFrame& frame) {
  return static_cast<std::uint8_t>((frame.id >> 8) & 0xFF);
}

std::optional<Feedback> ParseFeedback(const CanFrame& frame,
                                      const ActuatorLimits& limits) {
  if (GetCommType(frame) != static_cast<std::uint8_t>(CommType::kFeedback)) {
    return std::nullopt;
  }
  if (frame.dlc < 8) {
    return std::nullopt;
  }

  Feedback feedback;
  feedback.motor_id = GetSourceMotorId(frame);
  feedback.host_id = static_cast<std::uint8_t>(frame.id & 0xFF);
  feedback.mode = static_cast<MotorMode>((frame.id >> 22) & 0x03);
  feedback.fault.raw = static_cast<std::uint8_t>((frame.id >> 16) & 0x3F);

  const std::uint16_t pos_u =
      static_cast<std::uint16_t>((frame.data[0] << 8) | frame.data[1]);
  const std::uint16_t vel_u =
      static_cast<std::uint16_t>((frame.data[2] << 8) | frame.data[3]);
  const std::uint16_t torque_u =
      static_cast<std::uint16_t>((frame.data[4] << 8) | frame.data[5]);
  const std::uint16_t temp_u =
      static_cast<std::uint16_t>((frame.data[6] << 8) | frame.data[7]);

  feedback.position = UintToFloat(pos_u, -limits.position, limits.position);
  feedback.velocity = UintToFloat(vel_u, -limits.velocity, limits.velocity);
  feedback.torque = UintToFloat(torque_u, -limits.torque, limits.torque);
  feedback.temperature = static_cast<double>(temp_u) * 0.1;
  return feedback;
}

std::optional<ParamResponse> ParseParamResponse(const CanFrame& frame) {
  if (GetCommType(frame) != static_cast<std::uint8_t>(CommType::kReadParam)) {
    return std::nullopt;
  }
  if (frame.dlc < 8) {
    return std::nullopt;
  }

  ParamResponse response;
  response.motor_id = GetSourceMotorId(frame);
  response.index =
      static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8));
  std::copy(frame.data.begin() + 4, frame.data.begin() + 8,
            response.data.begin());
  return response;
}

}  // namespace robstride
