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
  const auto args = tracking_capture::parse_args(argc, argv);
  if (!args) {
    return EXIT_FAILURE;
  }

  constexpr double pi = std::numbers::pi;
  const std::vector<tracking_capture::Segment> profile = {
      {1.0, 0.0}, {4.0, pi}, {4.0, -pi}, {4.0, 0.0}};

  try {
    auto can = tracking_capture::make_can_interface(args->interface_name,
                                                    args->motor_id);
    auto motor = tracking_capture::make_rs02_motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::open_csv(args->out_path);
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.set_run_mode(robstride::RunMode::PositionCsp);
    // Zero here so the recorded positions are relative to the start.
    motor.set_mechanical_zero();
    motor.enable();
    motor.write_param(
        robstride::param_index::limit_spd,
        static_cast<float>(tracking_capture::bench::csp_speed_limit));

    tracking_capture::run_profile(
        profile, unwrapper, csv, [&motor](double target) {
          return motor.write_param(robstride::param_index::loc_ref,
                                   static_cast<float>(target));
        });

    motor.disable();
    std::cout << "wrote " << args->out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
