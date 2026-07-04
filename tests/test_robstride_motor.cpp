// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstring>
#include <deque>
#include <memory>
#include <vector>

#include "robstride_driver/robstride_motor.hpp"

namespace robstride {
namespace {

constexpr std::uint8_t kMotorId = 0x01;
constexpr std::uint8_t kHostId = 0xFD;

const ActuatorLimits& Rs02() { return GetActuatorLimits(ActuatorType::kRs02); }

/// Scripted CAN interface: records sent frames and plays back queued
/// responses.
class MockCanInterface : public CanInterface {
 public:
  void Send(const CanFrame& frame) override { sent.push_back(frame); }

  std::optional<CanFrame> Receive(std::chrono::milliseconds) override {
    if (responses.empty()) {
      return std::nullopt;
    }
    const CanFrame frame = responses.front();
    responses.pop_front();
    return frame;
  }

  std::vector<CanFrame> sent;
  std::deque<CanFrame> responses;
};

CanFrame FeedbackFrame(std::uint8_t motor_id, double position, double velocity,
                       double torque, double temperature,
                       MotorMode mode = MotorMode::kRun,
                       std::uint8_t fault = 0) {
  CanFrame frame;
  frame.id = (0x02U << 24) | (static_cast<std::uint32_t>(mode) << 22) |
             (static_cast<std::uint32_t>(fault & 0x3F) << 16) |
             (static_cast<std::uint32_t>(motor_id) << 8) | kHostId;
  const auto& limits = Rs02();
  const std::uint16_t pos_u =
      FloatToUint(position, -limits.position, limits.position);
  const std::uint16_t vel_u =
      FloatToUint(velocity, -limits.velocity, limits.velocity);
  const std::uint16_t torque_u =
      FloatToUint(torque, -limits.torque, limits.torque);
  const auto temp_u = static_cast<std::uint16_t>(temperature * 10);
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
  return frame;
}

CanFrame ParamResponseUint8(std::uint8_t motor_id, std::uint16_t index,
                            std::uint8_t value) {
  CanFrame frame;
  frame.id =
      (0x11U << 24) | (static_cast<std::uint32_t>(motor_id) << 8) | kHostId;
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  frame.data[4] = value;
  return frame;
}

CanFrame ParamResponseFloat(std::uint8_t motor_id, std::uint16_t index,
                            float value) {
  CanFrame frame;
  frame.id =
      (0x11U << 24) | (static_cast<std::uint32_t>(motor_id) << 8) | kHostId;
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8);
  std::memcpy(&frame.data[4], &value, sizeof(value));
  return frame;
}

class RobstrideMotorTest : public ::testing::Test {
 protected:
  RobstrideMotorTest() {
    RobstrideMotor::Config config;
    config.motor_id = kMotorId;
    config.host_id = kHostId;
    config.actuator_type = ActuatorType::kRs02;
    config.response_timeout = std::chrono::milliseconds(10);
    motor_ = std::make_unique<RobstrideMotor>(can_, config);
  }

  std::shared_ptr<MockCanInterface> can_ = std::make_shared<MockCanInterface>();
  std::unique_ptr<RobstrideMotor> motor_;
};

TEST_F(RobstrideMotorTest, EnableSendsFrameAndReturnsFeedback) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 1.0, 2.0, 0.5, 31.0));

  const Feedback feedback = motor_->Enable();

  ASSERT_EQ(can_->sent.size(), 1U);
  EXPECT_EQ(can_->sent[0].id, 0x0300FD01U);
  EXPECT_NEAR(feedback.position, 1.0, 1e-3);
  EXPECT_NEAR(feedback.velocity, 2.0, 1e-2);
  EXPECT_NEAR(feedback.temperature, 31.0, 1e-9);
  ASSERT_TRUE(motor_->last_feedback().has_value());
  EXPECT_NEAR(motor_->last_feedback()->position, 1.0, 1e-3);
}

TEST_F(RobstrideMotorTest, DisableSendsStopFrame) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0.0, 0.0, 0.0, 30.0));

  motor_->Disable(/*clear_fault=*/true);

  ASSERT_EQ(can_->sent.size(), 1U);
  EXPECT_EQ(can_->sent[0].id, 0x0400FD01U);
  EXPECT_EQ(can_->sent[0].data[0], 1);
}

TEST_F(RobstrideMotorTest, ThrowsTimeoutWhenNoResponse) {
  EXPECT_THROW(motor_->Enable(), TimeoutError);
}

TEST_F(RobstrideMotorTest, IgnoresFramesFromOtherMotors) {
  can_->responses.push_back(FeedbackFrame(0x22, 5.0, 5.0, 5.0, 50.0));
  can_->responses.push_back(FeedbackFrame(kMotorId, 1.0, 0.0, 0.0, 30.0));

  const Feedback feedback = motor_->Enable();
  EXPECT_NEAR(feedback.position, 1.0, 1e-3);
}

TEST_F(RobstrideMotorTest, SetRunModeStopsWritesAndVerifies) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));  // stop
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));  // write
  can_->responses.push_back(ParamResponseUint8(
      kMotorId, param_index::kRunMode,
      static_cast<std::uint8_t>(RunMode::kVelocity)));  // read back

  motor_->SetRunMode(RunMode::kVelocity);

  ASSERT_EQ(can_->sent.size(), 3U);
  EXPECT_EQ(can_->sent[0].id, 0x0400FD01U);  // stop
  EXPECT_EQ(can_->sent[1].id, 0x1200FD01U);  // write run_mode
  EXPECT_EQ(can_->sent[1].data[0], 0x05);
  EXPECT_EQ(can_->sent[1].data[1], 0x70);
  EXPECT_EQ(can_->sent[1].data[4], 2);
  EXPECT_EQ(can_->sent[2].id, 0x1100FD01U);  // read run_mode
}

TEST_F(RobstrideMotorTest, SetRunModeThrowsOnVerificationMismatch) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));
  can_->responses.push_back(ParamResponseUint8(
      kMotorId, param_index::kRunMode,
      static_cast<std::uint8_t>(RunMode::kOperationControl)));

  EXPECT_THROW(motor_->SetRunMode(RunMode::kVelocity), std::runtime_error);
}

TEST_F(RobstrideMotorTest, SendVelocityCommandWritesSpdRef) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0.0, 4.9, 0.2, 32.0));

  const Feedback feedback = motor_->SendVelocityCommand(5.0);

  ASSERT_EQ(can_->sent.size(), 1U);
  EXPECT_EQ(can_->sent[0].id, 0x1200FD01U);
  EXPECT_EQ(can_->sent[0].data[0], 0x0A);  // spd_ref index 0x700A
  EXPECT_EQ(can_->sent[0].data[1], 0x70);
  float value = 0.0F;
  std::memcpy(&value, &can_->sent[0].data[4], sizeof(value));
  EXPECT_FLOAT_EQ(value, 5.0F);
  EXPECT_NEAR(feedback.velocity, 4.9, 1e-2);
}

TEST_F(RobstrideMotorTest, ConfigureVelocityModeWritesLimits) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));

  motor_->ConfigureVelocityMode(/*current_limit=*/10.0, /*acceleration=*/20.0);

  ASSERT_EQ(can_->sent.size(), 2U);
  EXPECT_EQ(can_->sent[0].data[0], 0x18);  // limit_cur 0x7018
  EXPECT_EQ(can_->sent[1].data[0], 0x22);  // acc_rad 0x7022
}

TEST_F(RobstrideMotorTest, SendPositionCspCommandWritesLimitThenTarget) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));
  can_->responses.push_back(FeedbackFrame(kMotorId, 1.5, 0, 0, 30));

  const Feedback feedback =
      motor_->SendPositionCspCommand(/*position=*/1.5, /*speed_limit=*/3.0);

  ASSERT_EQ(can_->sent.size(), 2U);
  EXPECT_EQ(can_->sent[0].data[0], 0x17);  // limit_spd 0x7017
  EXPECT_EQ(can_->sent[1].data[0], 0x16);  // loc_ref 0x7016
  EXPECT_NEAR(feedback.position, 1.5, 1e-3);
}

TEST_F(RobstrideMotorTest, ReadParamFloat) {
  can_->responses.push_back(
      ParamResponseFloat(kMotorId, param_index::kVBus, 48.5F));

  EXPECT_FLOAT_EQ(motor_->ReadParamFloat(param_index::kVBus), 48.5F);
  ASSERT_EQ(can_->sent.size(), 1U);
  EXPECT_EQ(can_->sent[0].id, 0x1100FD01U);
}

TEST_F(RobstrideMotorTest, FeedbackFramesUpdateCacheWhileWaitingForParam) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 2.5, 0, 0, 33.0));
  can_->responses.push_back(
      ParamResponseFloat(kMotorId, param_index::kVBus, 48.0F));

  motor_->ReadParamFloat(param_index::kVBus);

  ASSERT_TRUE(motor_->last_feedback().has_value());
  EXPECT_NEAR(motor_->last_feedback()->position, 2.5, 1e-3);
}

TEST_F(RobstrideMotorTest, SetMechanicalZeroStopsFirst) {
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));
  can_->responses.push_back(FeedbackFrame(kMotorId, 0, 0, 0, 30));

  motor_->SetMechanicalZero();

  ASSERT_EQ(can_->sent.size(), 2U);
  EXPECT_EQ(can_->sent[0].id, 0x0400FD01U);  // stop
  EXPECT_EQ(can_->sent[1].id, 0x0600FD01U);  // set zero
  EXPECT_EQ(can_->sent[1].data[0], 1);
}

TEST_F(RobstrideMotorTest, FaultBitsExposedInFeedback) {
  can_->responses.push_back(
      FeedbackFrame(kMotorId, 0, 0, 0, 30, MotorMode::kRun, /*fault=*/0x04));

  const Feedback feedback = motor_->Enable();
  EXPECT_TRUE(feedback.fault.any());
  EXPECT_TRUE(feedback.fault.overtemperature());
}

}  // namespace
}  // namespace robstride
