// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Command-tracking data capture used for docs/test_results.md.
//
// Runs a fixed command profile in velocity mode or CSP position mode and
// logs the feedback answered by every command to CSV at 100 Hz. Plot the
// CSV with tools/plot_tracking.py.
//
// Precondition: the motor output shaft must be unloaded (as-delivered
// state) and free to rotate.
//
// Usage:
//   tracking_capture <velocity|position> <interface> <motor-id> <out.csv>
//
// <interface> is either a SocketCAN interface name (e.g. "can0") or the
// serial device of the official RobStride USB-CAN module (e.g.
// "/dev/ttyUSB0").
//
// Examples:
//   tracking_capture velocity /dev/ttyUSB0 127 velocity_tracking.csv
//   tracking_capture position can0 1 position_tracking.csv
//
// Profiles:
//   velocity: 0 -> +2 -> +4 -> -2 -> 0 rad/s
//             (current limit 10 A, acceleration 20 rad/s^2)
//   position: 0 -> +pi -> -pi -> 0 rad, speed limit 4 rad/s
//             (the current position is set as mechanical zero first)

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include "robstride_driver/robstride_driver.hpp"

namespace {

using Clock = std::chrono::steady_clock;

/// One step of the command profile: hold `target` for `duration_s`.
struct Segment {
  double duration_s;
  double target;
};

double SecondsSince(const Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: " << argv[0]
              << " <velocity|position> <interface> <motor-id> <out.csv>\n";
    return EXIT_FAILURE;
  }
  const std::string mode = argv[1];
  const std::string interface_name = argv[2];
  const auto motor_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
  const std::string out_path = argv[4];
  if (mode != "velocity" && mode != "position") {
    std::cerr << "error: mode must be 'velocity' or 'position'\n";
    return EXIT_FAILURE;
  }

  constexpr double kPi = std::numbers::pi;
  const std::vector<Segment> velocity_profile = {
      {1.0, 0.0}, {4.0, 2.0}, {4.0, 4.0}, {4.0, -2.0}, {3.0, 0.0}};
  const std::vector<Segment> position_profile = {
      {1.0, 0.0}, {4.0, kPi}, {4.0, -kPi}, {4.0, 0.0}};

  constexpr double kSamplePeriodS = 0.01;   // 100 Hz command/feedback
  constexpr double kCurrentLimitA = 10.0;   // velocity mode
  constexpr double kAccelerationRad = 20.0; // velocity mode [rad/s^2]
  constexpr double kCspSpeedLimit = 4.0;    // position mode [rad/s]

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
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv(out_path);
    if (!csv) {
      std::cerr << "error: cannot open '" << out_path << "' for writing\n";
      return EXIT_FAILURE;
    }
    csv << "t,target,position,velocity,torque,temperature\n";

    if (mode == "velocity") {
      motor.SetRunMode(robstride::RunMode::kVelocity);
      motor.Enable();
      motor.ConfigureVelocityMode(kCurrentLimitA, kAccelerationRad);
    } else {
      motor.SetRunMode(robstride::RunMode::kPositionCsp);
      // Zero here so the recorded positions are relative to the start.
      motor.SetMechanicalZero();
      motor.Enable();
      motor.WriteParam(robstride::param_index::kLimitSpd,
                       static_cast<float>(kCspSpeedLimit));
    }

    const auto& profile =
        (mode == "velocity") ? velocity_profile : position_profile;
    const auto start = Clock::now();
    for (const auto& segment : profile) {
      const double segment_end = SecondsSince(start) + segment.duration_s;
      while (SecondsSince(start) < segment_end) {
        const auto cycle_end =
            Clock::now() + std::chrono::duration_cast<Clock::duration>(
                               std::chrono::duration<double>(kSamplePeriodS));
        const robstride::Feedback feedback =
            (mode == "velocity")
                ? motor.SendVelocityCommand(segment.target)
                : motor.WriteParam(robstride::param_index::kLocRef,
                                   static_cast<float>(segment.target));
        csv << SecondsSince(start) << ',' << segment.target << ','
            << unwrapper.Update(feedback.position) << ',' << feedback.velocity
            << ',' << feedback.torque << ',' << feedback.temperature << '\n';
        std::this_thread::sleep_until(cycle_end);
      }
    }

    if (mode == "velocity") {
      motor.SendVelocityCommand(0.0);
    }
    motor.Disable();
    std::cout << "wrote " << out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
