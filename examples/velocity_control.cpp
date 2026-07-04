// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Velocity-mode control example / bench test tool.
//
// Runs the motor at a target velocity for a fixed duration while printing
// feedback, then stops it.
//
// Usage:
//   velocity_control <interface> <motor-id> <velocity-rad-s> [duration-s]
//
// <interface> is either a SocketCAN interface name (e.g. "can0") or the
// serial device of the official RobStride USB-CAN module (e.g.
// "/dev/ttyUSB0").
//
// Examples:
//   velocity_control can0 1 2.0 3.0
//   velocity_control /dev/ttyUSB0 1 2.0 3.0

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "robstride_driver/robstride_driver.hpp"

namespace {

void PrintFeedback(const robstride::Feedback& feedback) {
  std::cout << "pos=" << feedback.position << " rad"
            << "  vel=" << feedback.velocity << " rad/s"
            << "  torque=" << feedback.torque << " Nm"
            << "  temp=" << feedback.temperature << " C";
  if (feedback.fault.any()) {
    std::cout << "  FAULT(raw=0x" << std::hex
              << static_cast<int>(feedback.fault.raw) << std::dec << ")";
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0]
              << " <interface> <motor-id> <velocity-rad-s> [duration-s]\n";
    return EXIT_FAILURE;
  }
  const std::string interface_name = argv[1];
  const auto motor_id = static_cast<std::uint8_t>(std::stoi(argv[2]));
  const double velocity = std::stod(argv[3]);
  const double duration_s = (argc > 4) ? std::stod(argv[4]) : 3.0;

  try {
    std::shared_ptr<robstride::CanInterface> can;
    if (interface_name.rfind("/dev/", 0) == 0) {
      can = std::make_shared<robstride::AtSerialCanInterface>(interface_name);
    } else {
      auto socket_can =
          std::make_shared<robstride::SocketCanInterface>(interface_name);
      socket_can->SetMotorIdFilter(motor_id);
      can = socket_can;
    }

    robstride::RobstrideMotor::Config config;
    config.motor_id = motor_id;
    config.actuator_type = robstride::ActuatorType::kRs02;
    robstride::RobstrideMotor motor(can, config);

    std::cout << "Switching to velocity mode...\n";
    motor.SetRunMode(robstride::RunMode::kVelocity);
    motor.Enable();
    motor.ConfigureVelocityMode(/*current_limit=*/10.0,
                                /*acceleration=*/20.0);

    std::cout << "Running at " << velocity << " rad/s for " << duration_s
              << " s...\n";
    const auto end =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(duration_s * 1000));
    while (std::chrono::steady_clock::now() < end) {
      PrintFeedback(motor.SendVelocityCommand(velocity));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Stopping...\n";
    motor.SendVelocityCommand(0.0);
    motor.Disable();
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
