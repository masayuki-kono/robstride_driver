// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Command-tracking capture, velocity mode (run_mode 2, target spd_ref).
//
// Profile: 0 -> +2 -> +4 -> -2 -> 0 rad/s
// (current limit 10 A, acceleration 20 rad/s^2)
//
// Usage:
//   tracking_capture_velocity <interface> <motor-id> <out.csv>
//
// Plot with: tools/plot_tracking.py velocity <out.csv>
// See tracking_capture_common.hpp for the preconditions.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

#include "tracking_capture_common.hpp"

int main(int argc, char** argv) {
  const auto args = tracking_capture::ParseArgs(argc, argv);
  if (!args) {
    return EXIT_FAILURE;
  }

  const std::vector<tracking_capture::Segment> profile = {
      {1.0, 0.0}, {4.0, 2.0}, {4.0, 4.0}, {4.0, -2.0}, {3.0, 0.0}};
  constexpr double kCurrentLimitA = 10.0;
  constexpr double kAccelerationRad = 20.0;  // [rad/s^2]

  try {
    auto can = tracking_capture::MakeCanInterface(args->interface_name,
                                                  args->motor_id);
    auto motor = tracking_capture::MakeRs02Motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::OpenCsv(args->out_path);
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.SetRunMode(robstride::RunMode::kVelocity);
    motor.Enable();
    motor.ConfigureVelocityMode(kCurrentLimitA, kAccelerationRad);

    tracking_capture::RunProfile(
        profile, unwrapper, csv,
        [&motor](double target) { return motor.SendVelocityCommand(target); });

    motor.SendVelocityCommand(0.0);
    motor.Disable();
    std::cout << "wrote " << args->out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
