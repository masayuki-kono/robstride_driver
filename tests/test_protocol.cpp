// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>

#include "robstride_driver/protocol.hpp"

namespace robstride {
namespace {

constexpr std::uint8_t kMotorId = 0x01;
constexpr std::uint8_t kHostId = 0xFD;

const ActuatorLimits& Rs02() { return GetActuatorLimits(ActuatorType::kRs02); }

TEST(ActuatorLimitsTest, Rs02MatchesManual) {
  const auto& limits = Rs02();
  EXPECT_DOUBLE_EQ(limits.position, 4 * M_PI);  // +-12.57 rad
  EXPECT_DOUBLE_EQ(limits.velocity, 44.0);      // +-44 rad/s
  EXPECT_DOUBLE_EQ(limits.torque, 17.0);        // +-17 Nm
  EXPECT_DOUBLE_EQ(limits.kp_max, 500.0);
  EXPECT_DOUBLE_EQ(limits.kd_max, 5.0);
}

TEST(ScalingTest, FloatToUintClampsAndMaps) {
  EXPECT_EQ(FloatToUint(-17.0, -17.0, 17.0), 0);
  EXPECT_EQ(FloatToUint(17.0, -17.0, 17.0), 65535);
  EXPECT_EQ(FloatToUint(-100.0, -17.0, 17.0), 0);        // clamped low
  EXPECT_EQ(FloatToUint(100.0, -17.0, 17.0), 65535);     // clamped high
  EXPECT_NEAR(FloatToUint(0.0, -17.0, 17.0), 32767, 1);  // midpoint
}

TEST(ScalingTest, RoundTripIsAccurate) {
  const std::array<double, 6> values = {-40.0, -1.5, 0.0, 0.001, 12.3, 44.0};
  for (const double value : values) {
    const std::uint16_t encoded = FloatToUint(value, -44.0, 44.0);
    const double decoded = UintToFloat(encoded, -44.0, 44.0);
    EXPECT_NEAR(decoded, value, 88.0 / 65535.0);  // one LSB
  }
}

TEST(EncodeTest, EnableFrame) {
  const CanFrame frame = MakeEnableFrame(kMotorId, kHostId);
  EXPECT_EQ(frame.id, 0x0300FD01U);
  EXPECT_EQ(frame.dlc, 8);
  for (const auto byte : frame.data) {
    EXPECT_EQ(byte, 0);
  }
}

TEST(EncodeTest, StopFrame) {
  const CanFrame frame = MakeStopFrame(kMotorId, kHostId, /*clear_fault=*/true);
  EXPECT_EQ(frame.id, 0x0400FD01U);
  EXPECT_EQ(frame.data[0], 1);

  const CanFrame no_clear = MakeStopFrame(kMotorId, kHostId);
  EXPECT_EQ(no_clear.data[0], 0);
}

TEST(EncodeTest, SetMechanicalZeroFrame) {
  const CanFrame frame = MakeSetMechanicalZeroFrame(kMotorId, kHostId);
  EXPECT_EQ(frame.id, 0x0600FD01U);
  EXPECT_EQ(frame.data[0], 1);
}

TEST(EncodeTest, SetCanIdFrame) {
  const CanFrame frame = MakeSetCanIdFrame(kMotorId, kHostId, 0x05);
  // type 0x07 | new id in bit23-16 | host id in bit15-8 | current motor id
  EXPECT_EQ(frame.id, 0x0705FD01U);
}

TEST(EncodeTest, ReadParamFrame) {
  const CanFrame frame =
      MakeReadParamFrame(kMotorId, kHostId, param_index::kRunMode);
  EXPECT_EQ(frame.id, 0x1100FD01U);
  EXPECT_EQ(frame.data[0], 0x05);  // index low byte first
  EXPECT_EQ(frame.data[1], 0x70);
  EXPECT_EQ(frame.data[4], 0x00);
}

TEST(EncodeTest, WriteParamFloatFrame) {
  const float value = 5.0F;
  const CanFrame frame =
      MakeWriteParamFrame(kMotorId, kHostId, param_index::kSpdRef, value);
  EXPECT_EQ(frame.id, 0x1200FD01U);
  EXPECT_EQ(frame.data[0], 0x0A);
  EXPECT_EQ(frame.data[1], 0x70);
  float decoded = 0.0F;
  std::memcpy(&decoded, &frame.data[4], sizeof(decoded));
  EXPECT_FLOAT_EQ(decoded, value);
}

TEST(EncodeTest, WriteParamUint8Frame) {
  const CanFrame frame =
      MakeWriteParamFrame(kMotorId, kHostId, param_index::kRunMode,
                          static_cast<std::uint8_t>(RunMode::kVelocity));
  EXPECT_EQ(frame.id, 0x1200FD01U);
  EXPECT_EQ(frame.data[4], 2);
  EXPECT_EQ(frame.data[5], 0);
}

TEST(EncodeTest, MotionControlFrame) {
  // Manual example (4.4.2): torque in the id, pos/vel/kp/kd big-endian in
  // the data field.
  const CanFrame frame = MakeMotionControlFrame(
      kMotorId, /*torque=*/0.0, /*position=*/0.0, /*velocity=*/0.0,
      /*kp=*/0.0, /*kd=*/0.0, Rs02());
  EXPECT_EQ((frame.id >> 24) & 0x1F, 0x01U);
  EXPECT_EQ(frame.id & 0xFF, kMotorId);
  // torque 0 -> midpoint 0x7FFF or 0x8000 depending on rounding
  const std::uint16_t torque_u = (frame.id >> 8) & 0xFFFF;
  EXPECT_NEAR(torque_u, 32767, 1);
  const std::uint16_t pos_u = (frame.data[0] << 8) | frame.data[1];
  EXPECT_NEAR(pos_u, 32767, 1);
  EXPECT_EQ(frame.data[4], 0);  // kp = 0 -> 0x0000
  EXPECT_EQ(frame.data[5], 0);
}

TEST(DecodeTest, ParseFeedback) {
  CanFrame frame;
  // type 2, mode=run (bit23-22 = 2), no fault, motor id 0x01, host 0xFD
  frame.id = (0x02U << 24) | (0x02U << 22) | (kMotorId << 8) | kHostId;
  const std::uint16_t pos_u =
      FloatToUint(1.0, -Rs02().position, Rs02().position);
  const std::uint16_t vel_u = FloatToUint(-2.0, -44.0, 44.0);
  const std::uint16_t torque_u = FloatToUint(3.5, -17.0, 17.0);
  const std::uint16_t temp_u = 305;  // 30.5 C
  frame.data = {
      static_cast<std::uint8_t>(pos_u >> 8),
      static_cast<std::uint8_t>(pos_u & 0xFF),
      static_cast<std::uint8_t>(vel_u >> 8),
      static_cast<std::uint8_t>(vel_u & 0xFF),
      static_cast<std::uint8_t>(torque_u >> 8),
      static_cast<std::uint8_t>(torque_u & 0xFF),
      static_cast<std::uint8_t>(temp_u >> 8),
      static_cast<std::uint8_t>(temp_u & 0xFF),
  };

  const auto feedback = ParseFeedback(frame, Rs02());
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->motor_id, kMotorId);
  EXPECT_EQ(feedback->host_id, kHostId);
  EXPECT_EQ(feedback->mode, MotorMode::kRun);
  EXPECT_FALSE(feedback->fault.any());
  EXPECT_NEAR(feedback->position, 1.0, 1e-3);
  EXPECT_NEAR(feedback->velocity, -2.0, 1e-2);
  EXPECT_NEAR(feedback->torque, 3.5, 1e-3);
  EXPECT_NEAR(feedback->temperature, 30.5, 1e-9);
}

TEST(DecodeTest, ParseFeedbackFaultBits) {
  CanFrame frame;
  // fault bits occupy bit21-16: undervoltage (bit16) + overtemp (bit18)
  frame.id = (0x02U << 24) | (0x05U << 16) | (kMotorId << 8) | kHostId;
  const auto feedback = ParseFeedback(frame, Rs02());
  ASSERT_TRUE(feedback.has_value());
  EXPECT_TRUE(feedback->fault.any());
  EXPECT_TRUE(feedback->fault.undervoltage());
  EXPECT_TRUE(feedback->fault.overtemperature());
  EXPECT_FALSE(feedback->fault.overcurrent());
  EXPECT_FALSE(feedback->fault.uncalibrated());
}

TEST(DecodeTest, ParseFeedbackRejectsOtherTypes) {
  CanFrame frame;
  frame.id = (0x11U << 24) | (kMotorId << 8) | kHostId;
  EXPECT_FALSE(ParseFeedback(frame, Rs02()).has_value());
}

TEST(DecodeTest, ParseParamResponse) {
  // Manual read example (4.1.14): loc_kp = 30.0 -> data "1E 70 00 00 00 00
  // F0 41" with id 0x1100 7FFD.
  CanFrame frame;
  frame.id = 0x11007FFDU;
  frame.data = {0x1E, 0x70, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x41};

  const auto response = ParseParamResponse(frame);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->motor_id, 0x7F);
  EXPECT_EQ(response->index, 0x701E);
  EXPECT_FLOAT_EQ(response->AsFloat(), 30.0F);
}

TEST(DecodeTest, ParseParamResponseUint8) {
  CanFrame frame;
  frame.id = (0x11U << 24) | (kMotorId << 8) | kHostId;
  frame.data = {0x05, 0x70, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};

  const auto response = ParseParamResponse(frame);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->index, param_index::kRunMode);
  EXPECT_EQ(response->AsUint8(), static_cast<std::uint8_t>(RunMode::kVelocity));
}

TEST(DecodeTest, GetSourceMotorId) {
  CanFrame frame;
  frame.id = (0x02U << 24) | (0x42U << 8) | kHostId;
  EXPECT_EQ(GetSourceMotorId(frame), 0x42);
}

TEST(MotorSideEncodeTest, FeedbackFrameRoundTrip) {
  Feedback feedback;
  feedback.motor_id = kMotorId;
  feedback.host_id = kHostId;
  feedback.mode = MotorMode::kRun;
  feedback.fault.raw = 0x05;  // undervoltage + overtemperature
  feedback.position = 1.0;
  feedback.velocity = -2.0;
  feedback.torque = 3.5;
  feedback.temperature = 30.5;

  const CanFrame frame = MakeFeedbackFrame(feedback, Rs02());
  const auto decoded = ParseFeedback(frame, Rs02());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->motor_id, kMotorId);
  EXPECT_EQ(decoded->host_id, kHostId);
  EXPECT_EQ(decoded->mode, MotorMode::kRun);
  EXPECT_EQ(decoded->fault.raw, 0x05);
  EXPECT_NEAR(decoded->position, 1.0, 1e-3);
  EXPECT_NEAR(decoded->velocity, -2.0, 1e-2);
  EXPECT_NEAR(decoded->torque, 3.5, 1e-3);
  EXPECT_NEAR(decoded->temperature, 30.5, 1e-9);
}

TEST(MotorSideEncodeTest, ParamResponseFrameRoundTrip) {
  const CanFrame float_frame =
      MakeParamResponseFrame(kMotorId, kHostId, param_index::kSpdRef, 4.5F);
  const auto float_response = ParseParamResponse(float_frame);
  ASSERT_TRUE(float_response.has_value());
  EXPECT_EQ(float_response->motor_id, kMotorId);
  EXPECT_EQ(float_response->index, param_index::kSpdRef);
  EXPECT_FLOAT_EQ(float_response->AsFloat(), 4.5F);

  const CanFrame uint8_frame =
      MakeParamResponseFrame(kMotorId, kHostId, param_index::kRunMode,
                             static_cast<std::uint8_t>(RunMode::kVelocity));
  const auto uint8_response = ParseParamResponse(uint8_frame);
  ASSERT_TRUE(uint8_response.has_value());
  EXPECT_EQ(uint8_response->AsUint8(),
            static_cast<std::uint8_t>(RunMode::kVelocity));
}

TEST(CommandDecodeTest, ParseParamReadRequestRoundTrip) {
  const CanFrame frame =
      MakeReadParamFrame(kMotorId, kHostId, param_index::kRunMode);
  const auto request = ParseParamReadRequest(frame);
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->motor_id, kMotorId);
  EXPECT_EQ(request->host_id, kHostId);
  EXPECT_EQ(request->index, param_index::kRunMode);
}

TEST(CommandDecodeTest, ParseParamWriteRequestRoundTrip) {
  const CanFrame float_frame =
      MakeWriteParamFrame(kMotorId, kHostId, param_index::kSpdRef, 4.5F);
  const auto float_request = ParseParamWriteRequest(float_frame);
  ASSERT_TRUE(float_request.has_value());
  EXPECT_EQ(float_request->motor_id, kMotorId);
  EXPECT_EQ(float_request->host_id, kHostId);
  EXPECT_EQ(float_request->index, param_index::kSpdRef);
  EXPECT_FLOAT_EQ(float_request->AsFloat(), 4.5F);

  const CanFrame uint8_frame =
      MakeWriteParamFrame(kMotorId, kHostId, param_index::kRunMode,
                          static_cast<std::uint8_t>(RunMode::kPositionCsp));
  const auto uint8_request = ParseParamWriteRequest(uint8_frame);
  ASSERT_TRUE(uint8_request.has_value());
  EXPECT_EQ(uint8_request->AsUint8(),
            static_cast<std::uint8_t>(RunMode::kPositionCsp));
}

TEST(CommandDecodeTest, ParseRequestsRejectOtherTypes) {
  const CanFrame enable = MakeEnableFrame(kMotorId, kHostId);
  EXPECT_FALSE(ParseParamReadRequest(enable).has_value());
  EXPECT_FALSE(ParseParamWriteRequest(enable).has_value());
}

}  // namespace
}  // namespace robstride
