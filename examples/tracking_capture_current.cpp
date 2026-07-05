// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Command-tracking capture, current mode (run_mode 3, target iq_ref).
//
// Profile: 0 -> +0.3 -> -0.3 -> +0.3 -> -0.3 -> 0 A.
// Beside the regular feedback, every cycle also reads the filtered Iq
// (parameter 0x701A) and logs it as an extra "iq" CSV column.
//
// Usage:
//   tracking_capture_current <interface> <motor-id> <out.csv>
//
// Plot with: tools/plot_tracking.py current <out.csv>
// See tracking_capture_common.hpp for the preconditions.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

#include "tracking_capture_common.hpp"

int main(int argc, char** argv) {
  const auto args = tracking_capture::parse_args(argc, argv);
  if (!args) {
    return EXIT_FAILURE;
  }

  const std::vector<tracking_capture::Segment> profile = {
      {1.0, 0.0}, {1.5, 0.3}, {1.5, -0.3}, {1.5, 0.3}, {1.5, -0.3}, {1.0, 0.0}};

  try {
    auto can = tracking_capture::make_can_interface(args->interface_name,
                                                    args->motor_id);
    auto motor = tracking_capture::make_rs02_motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::open_csv(args->out_path, ",iq");
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.set_run_mode(robstride::RunMode::Current);
    motor.enable();

    tracking_capture::run_profile(
        profile, unwrapper, csv,
        [&motor](double target) {
          return motor.write_param(robstride::param_index::iq_ref,
                                   static_cast<float>(target));
        },
        [&motor](std::ofstream& row) {
          row << ','
              << motor.read_param_float(robstride::param_index::iq_filtered);
        });

    motor.write_param(robstride::param_index::iq_ref, 0.0F);
    motor.disable();
    std::cout << "wrote " << args->out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
