// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Command-tracking capture, PP position mode (run_mode 1, target loc_ref).
//
// Profile: 0 -> +pi -> -pi -> 0 rad, vel_max 4 rad/s, acc_set 20 rad/s^2.
// The current position is set as mechanical zero first so the recorded
// positions are relative to the start.
//
// Usage:
//   tracking_capture_pp <interface> <motor-id> <out.csv>
//
// Plot with: tools/plot_tracking.py pp <out.csv>
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
  constexpr double kVelMax = 4.0;   // [rad/s]
  constexpr double kAccSet = 20.0;  // [rad/s^2]

  try {
    auto can = tracking_capture::MakeCanInterface(args->interface_name,
                                                  args->motor_id);
    auto motor = tracking_capture::MakeRs02Motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::OpenCsv(args->out_path);
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.SetRunMode(robstride::RunMode::kPositionPp);
    // Zero here so the recorded positions are relative to the start.
    motor.SetMechanicalZero();
    motor.Enable();
    motor.WriteParam(robstride::param_index::kVelMax,
                     static_cast<float>(kVelMax));
    motor.WriteParam(robstride::param_index::kAccSet,
                     static_cast<float>(kAccSet));

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
