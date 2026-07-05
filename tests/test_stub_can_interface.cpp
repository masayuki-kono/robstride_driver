// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "fixtures/can_bus.hpp"
#include "robstride_driver/robstride_motor.hpp"
#include "robstride_driver/stub_can_interface.hpp"

namespace robstride {
namespace {

namespace bus = test_fixtures::can_bus::dual;

RobstrideMotor::Config MotorConfig(std::uint8_t motor_id) {
  RobstrideMotor::Config config;
  config.motor_id = motor_id;
  config.host_id = bus::host_id;
  config.actuator_type = ActuatorType::Rs02;
  config.response_timeout = std::chrono::milliseconds(10);
  return config;
}

class StubCanInterfaceTest : public ::testing::Test {
 protected:
  StubCanInterfaceTest()
      : left_(std::make_unique<RobstrideMotor>(
            can_, MotorConfig(bus::left_motor_id))),
        right_(std::make_unique<RobstrideMotor>(
            can_, MotorConfig(bus::right_motor_id))) {}

  /// Runs the velocity-mode initialization sequence used by applications.
  static void InitializeVelocityMode(RobstrideMotor& motor) {
    motor.set_run_mode(RunMode::Velocity);
    motor.enable();
    motor.configure_velocity_mode(/*current_limit=*/10.0,
                                  /*acceleration=*/20.0);
  }

  std::shared_ptr<StubCanInterface> can_ =
      std::make_shared<StubCanInterface>(ActuatorType::Rs02);
  std::unique_ptr<RobstrideMotor> left_;
  std::unique_ptr<RobstrideMotor> right_;
};

TEST_F(StubCanInterfaceTest, EnableAnswersWithFeedback) {
  const Feedback feedback = left_->enable();
  EXPECT_EQ(feedback.motor_id, bus::left_motor_id);
  EXPECT_EQ(feedback.mode, MotorMode::Run);
  EXPECT_FALSE(feedback.fault.any());
}

TEST_F(StubCanInterfaceTest, DisableReportsResetMode) {
  left_->enable();
  const Feedback feedback = left_->disable();
  EXPECT_EQ(feedback.mode, MotorMode::Reset);
}

TEST_F(StubCanInterfaceTest, SetRunModeReadBackVerificationSucceeds) {
  // set_run_mode internally stops the motor, writes run_mode, and reads it
  // back; a stub that does not remember the written mode would throw.
  EXPECT_NO_THROW(left_->set_run_mode(RunMode::Velocity));
  EXPECT_EQ(left_->get_run_mode(), RunMode::Velocity);
  EXPECT_NO_THROW(left_->set_run_mode(RunMode::PositionCsp));
  EXPECT_EQ(left_->get_run_mode(), RunMode::PositionCsp);
}

TEST_F(StubCanInterfaceTest, VelocityCommandIntegratesPosition) {
  InitializeVelocityMode(*left_);

  const Feedback start = left_->send_velocity_command(2.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const Feedback end = left_->send_velocity_command(2.0);

  EXPECT_NEAR(end.velocity, 2.0, 1e-2);
  EXPECT_GT(end.position, start.position);
}

TEST_F(StubCanInterfaceTest, PositionCommandReachesTargetInstantly) {
  InitializeVelocityMode(*left_);
  left_->set_run_mode(RunMode::PositionCsp);
  left_->enable();

  const Feedback feedback =
      left_->send_position_csp_command(/*position=*/1.5, /*speed_limit=*/3.0);
  EXPECT_NEAR(feedback.position, 1.5, 1e-3);
  EXPECT_NEAR(feedback.velocity, 0.0, 1e-2);
}

TEST_F(StubCanInterfaceTest, MotorsAreSimulatedIndependently) {
  InitializeVelocityMode(*left_);
  InitializeVelocityMode(*right_);

  left_->send_velocity_command(2.0);
  right_->send_velocity_command(-1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const Feedback left_feedback = left_->send_velocity_command(2.0);
  const Feedback right_feedback = right_->send_velocity_command(-1.0);

  EXPECT_GT(left_feedback.position, 0.0);
  EXPECT_LT(right_feedback.position, 0.0);
}

TEST_F(StubCanInterfaceTest, InjectedFaultBitsAppearInFeedback) {
  can_->set_fault_bits(bus::left_motor_id, 0x04);  // overtemperature

  const Feedback feedback = left_->enable();
  EXPECT_TRUE(feedback.fault.overtemperature());
  EXPECT_FALSE(feedback.fault.undervoltage());
}

TEST_F(StubCanInterfaceTest, StopWithClearFaultClearsInjectedFaults) {
  can_->set_fault_bits(bus::left_motor_id, 0x04);

  const Feedback feedback = left_->disable(/*clear_fault=*/true);
  EXPECT_FALSE(feedback.fault.any());
}

TEST_F(StubCanInterfaceTest, TemperatureIsReported) {
  can_->set_temperature(42.5);

  const Feedback feedback = left_->enable();
  EXPECT_NEAR(feedback.temperature, 42.5, 1e-9);
}

TEST_F(StubCanInterfaceTest, UnsupportedCommandTimesOut) {
  CanFrame frame;
  frame.id =
      (0x00U << 24) | (bus::host_id << 8) | bus::left_motor_id;  // GetDeviceId
  can_->send(frame);
  EXPECT_FALSE(can_->receive(std::chrono::milliseconds(1)).has_value());
}

}  // namespace
}  // namespace robstride
