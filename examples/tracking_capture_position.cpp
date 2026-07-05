// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Command-tracking capture, CSP position mode (run_mode 5, target loc_ref).
//
// Profile: 0 -> +pi -> -pi -> 0 rad, speed limit 4 rad/s.
// The current position is set as mechanical zero first so the recorded
// positions are relative to the start.
//
// Usage:
//   tracking_capture_position <interface> <motor-id> <out.csv>
//
// Plot with: tools/plot_tracking.py position <out.csv>
// See tracking_capture_common.hpp for the preconditions.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <numbers>
#include <vector>

#include "tracking_capture_common.hpp"

int main(int argc, char** argv) {
  const auto args = tracking_capture::ParseArgs(argc, argv);
  if (!args) {
    return EXIT_FAILURE;
  }

  constexpr double kPi = std::numbers::pi;
  const std::vector<tracking_capture::Segment> profile = {
      {1.0, 0.0}, {4.0, kPi}, {4.0, -kPi}, {4.0, 0.0}};
  constexpr double kSpeedLimit = 4.0;  // [rad/s]

  try {
    auto can = tracking_capture::MakeCanInterface(args->interface_name,
                                                  args->motor_id);
    auto motor = tracking_capture::MakeRs02Motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::OpenCsv(args->out_path);
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.SetRunMode(robstride::RunMode::kPositionCsp);
    // Zero here so the recorded positions are relative to the start.
    motor.SetMechanicalZero();
    motor.Enable();
    motor.WriteParam(robstride::param_index::kLimitSpd,
                     static_cast<float>(kSpeedLimit));

    tracking_capture::RunProfile(
        profile, unwrapper, csv, [&motor](double target) {
          return motor.WriteParam(robstride::param_index::kLocRef,
                                  static_cast<float>(target));
        });

    motor.Disable();
    std::cout << "wrote " << args->out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
