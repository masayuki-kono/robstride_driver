// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "robstride_driver/robstride_motor.hpp"
#include "robstride_driver/stub_can_interface.hpp"

namespace robstride {
namespace {

constexpr std::uint8_t kLeftId = 0x01;
constexpr std::uint8_t kRightId = 0x02;
constexpr std::uint8_t kHostId = 0xFD;

RobstrideMotor::Config MotorConfig(std::uint8_t motor_id) {
  RobstrideMotor::Config config;
  config.motor_id = motor_id;
  config.host_id = kHostId;
  config.actuator_type = ActuatorType::kRs02;
  config.response_timeout = std::chrono::milliseconds(10);
  return config;
}

class StubCanInterfaceTest : public ::testing::Test {
 protected:
  StubCanInterfaceTest()
      : left_(std::make_unique<RobstrideMotor>(can_, MotorConfig(kLeftId))),
        right_(std::make_unique<RobstrideMotor>(can_, MotorConfig(kRightId))) {}

  /// Runs the velocity-mode initialization sequence used by applications.
  static void InitializeVelocityMode(RobstrideMotor& motor) {
    motor.SetRunMode(RunMode::kVelocity);
    motor.Enable();
    motor.ConfigureVelocityMode(/*current_limit=*/10.0,
                                /*acceleration=*/20.0);
  }

  std::shared_ptr<StubCanInterface> can_ =
      std::make_shared<StubCanInterface>(ActuatorType::kRs02);
  std::unique_ptr<RobstrideMotor> left_;
  std::unique_ptr<RobstrideMotor> right_;
};

TEST_F(StubCanInterfaceTest, EnableAnswersWithFeedback) {
  const Feedback feedback = left_->Enable();
  EXPECT_EQ(feedback.motor_id, kLeftId);
  EXPECT_EQ(feedback.mode, MotorMode::kRun);
  EXPECT_FALSE(feedback.fault.any());
}

TEST_F(StubCanInterfaceTest, DisableReportsResetMode) {
  left_->Enable();
  const Feedback feedback = left_->Disable();
  EXPECT_EQ(feedback.mode, MotorMode::kReset);
}

TEST_F(StubCanInterfaceTest, SetRunModeReadBackVerificationSucceeds) {
  // SetRunMode internally stops the motor, writes run_mode, and reads it
  // back; a stub that does not remember the written mode would throw.
  EXPECT_NO_THROW(left_->SetRunMode(RunMode::kVelocity));
  EXPECT_EQ(left_->GetRunMode(), RunMode::kVelocity);
  EXPECT_NO_THROW(left_->SetRunMode(RunMode::kPositionCsp));
  EXPECT_EQ(left_->GetRunMode(), RunMode::kPositionCsp);
}

TEST_F(StubCanInterfaceTest, VelocityCommandIntegratesPosition) {
  InitializeVelocityMode(*left_);

  const Feedback start = left_->SendVelocityCommand(2.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const Feedback end = left_->SendVelocityCommand(2.0);

  EXPECT_NEAR(end.velocity, 2.0, 1e-2);
  EXPECT_GT(end.position, start.position);
}

TEST_F(StubCanInterfaceTest, PositionCommandReachesTargetInstantly) {
  InitializeVelocityMode(*left_);
  left_->SetRunMode(RunMode::kPositionCsp);
  left_->Enable();

  const Feedback feedback =
      left_->SendPositionCspCommand(/*position=*/1.5, /*speed_limit=*/3.0);
  EXPECT_NEAR(feedback.position, 1.5, 1e-3);
  EXPECT_NEAR(feedback.velocity, 0.0, 1e-2);
}

TEST_F(StubCanInterfaceTest, MotorsAreSimulatedIndependently) {
  InitializeVelocityMode(*left_);
  InitializeVelocityMode(*right_);

  left_->SendVelocityCommand(2.0);
  right_->SendVelocityCommand(-1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const Feedback left_feedback = left_->SendVelocityCommand(2.0);
  const Feedback right_feedback = right_->SendVelocityCommand(-1.0);

  EXPECT_GT(left_feedback.position, 0.0);
  EXPECT_LT(right_feedback.position, 0.0);
}

TEST_F(StubCanInterfaceTest, InjectedFaultBitsAppearInFeedback) {
  can_->set_fault_bits(kLeftId, 0x04);  // overtemperature

  const Feedback feedback = left_->Enable();
  EXPECT_TRUE(feedback.fault.overtemperature());
  EXPECT_FALSE(feedback.fault.undervoltage());
}

TEST_F(StubCanInterfaceTest, StopWithClearFaultClearsInjectedFaults) {
  can_->set_fault_bits(kLeftId, 0x04);

  const Feedback feedback = left_->Disable(/*clear_fault=*/true);
  EXPECT_FALSE(feedback.fault.any());
}

TEST_F(StubCanInterfaceTest, TemperatureIsReported) {
  can_->set_temperature(42.5);

  const Feedback feedback = left_->Enable();
  EXPECT_NEAR(feedback.temperature, 42.5, 1e-9);
}

TEST_F(StubCanInterfaceTest, UnsupportedCommandTimesOut) {
  CanFrame frame;
  frame.id = (0x00U << 24) | (kHostId << 8) | kLeftId;  // kGetDeviceId
  can_->Send(frame);
  EXPECT_FALSE(can_->Receive(std::chrono::milliseconds(1)).has_value());
}

}  // namespace
}  // namespace robstride
