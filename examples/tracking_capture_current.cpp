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
  const auto args = tracking_capture::ParseArgs(argc, argv);
  if (!args) {
    return EXIT_FAILURE;
  }

  const std::vector<tracking_capture::Segment> profile = {
      {1.0, 0.0}, {1.5, 0.3}, {1.5, -0.3}, {1.5, 0.3}, {1.5, -0.3}, {1.0, 0.0}};

  try {
    auto can = tracking_capture::MakeCanInterface(args->interface_name,
                                                  args->motor_id);
    auto motor = tracking_capture::MakeRs02Motor(can, args->motor_id);
    robstride::PositionUnwrapper unwrapper(motor.limits());

    std::ofstream csv = tracking_capture::OpenCsv(args->out_path, ",iq");
    if (!csv) {
      return EXIT_FAILURE;
    }

    motor.SetRunMode(robstride::RunMode::kCurrent);
    motor.Enable();

    tracking_capture::RunProfile(
        profile, unwrapper, csv,
        [&motor](double target) {
          return motor.WriteParam(robstride::param_index::kIqRef,
                                  static_cast<float>(target));
        },
        [&motor](std::ofstream& row) {
          row << ','
              << motor.ReadParamFloat(robstride::param_index::kIqFiltered);
        });

    motor.WriteParam(robstride::param_index::kIqRef, 0.0F);
    motor.Disable();
    std::cout << "wrote " << args->out_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
