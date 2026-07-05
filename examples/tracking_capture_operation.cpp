// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Command-tracking capture, operation (MIT) control (run_mode 0,
// motion command communication type 1).
//
// Profile: 0 -> +pi/2 -> -pi/2 -> 0 rad as position targets, zero
// velocity target and zero torque feed-forward. The gains default to
// Kp 4 / Kd 1 and can be overridden with the optional [kp] [kd]
// arguments, e.g. to demonstrate the effect of a stiffer Kp
// (see docs/test_results.md).
// The current position is set as mechanical zero first so the recorded
// positions are relative to the start.
//
// Usage:
//   tracking_capture_operation <interface> <motor-id> <out.csv> [kp] [kd]
//
// Plot with: tools/plot_tracking.py operation <out.csv>
// See tracking_capture_common.hpp for the preconditions.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include "tracking_capture_common.hpp"

int main(int argc, char** argv) {
  const auto args = tracking_capture::parse_args(argc, argv, " [kp] [kd]");
  if (!args) {
    return EXIT_FAILURE;
  }
  constexpr double pi = std::numbers::pi;
  const std::vector<tracking_capture::Segment> profile = {
      {1.0, 0.0}, {3.0, pi / 2}, {3.0, -pi / 2}, {3.0, 0.0}};

  try {
    const double kp = (argc > 4) ? std::stod(argv[4]) : 4.0;
    const double kd = (argc > 5) ? std::stod(argv[5]) : 1.0;

    auto can = tracking_capture::make_can_interface(args->interface_name,
                                                    args->motor_id);
    auto motor = tracking_capture::make_rs02_motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::open_csv(args->out_path);
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.set_run_mode(robstride::RunMode::OperationControl);
    // Zero here so the recorded positions are relative to the start.
    motor.set_mechanical_zero();
    motor.enable();

    tracking_capture::run_profile(
        profile, unwrapper, csv, [&motor, kp, kd](double target) {
          return motor.send_motion_command(0.0, target, 0.0, kp, kd);
        });

    motor.disable();
    std::cout << "wrote " << args->out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
