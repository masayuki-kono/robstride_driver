// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Shared helpers for the tracking_capture_* examples (one per control
// mode) that record the command-tracking data used in
// docs/test_results.md. Each example runs a fixed command profile and
// logs the feedback answered by every command to CSV at 100 Hz; plot the
// CSV with tools/plot_tracking.py.
//
// Precondition for all captures: the motor output shaft must be unloaded
// (as-delivered state) and free to rotate.

#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "robstride_driver/robstride_driver.hpp"

namespace tracking_capture {

using Clock = std::chrono::steady_clock;

/// One step of a command profile: hold `target` for `duration_s`.
struct Segment {
  double duration_s;
  double target;
};

/// Command / feedback period (100 Hz).
inline constexpr double kSamplePeriodS = 0.01;

/// Command-line arguments shared by every capture example.
struct Args {
  std::string interface_name;
  std::uint8_t motor_id = 0;
  std::string out_path;
};

inline double SecondsSince(const Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

/// Parses the common "<interface> <motor-id> <out.csv>" arguments
/// (argv[1..3]). Prints a usage message and returns nullopt when they are
/// missing. `extra_usage` documents additional, example-specific
/// arguments (e.g. " [kp] [kd]").
inline std::optional<Args> ParseArgs(int argc, char** argv,
                                     const char* extra_usage = "") {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0] << " <interface> <motor-id> <out.csv>"
              << extra_usage << '\n'
              << "  <interface> is a SocketCAN interface name (e.g. can0) or"
                 " the serial\n  device of the official RobStride USB-CAN"
                 " module (e.g. /dev/ttyUSB0)\n";
    return std::nullopt;
  }
  Args args;
  args.interface_name = argv[1];
  try {
    args.motor_id = static_cast<std::uint8_t>(std::stoi(argv[2]));
  } catch (const std::exception&) {
    std::cerr << "error: invalid motor-id '" << argv[2] << "'\n";
    return std::nullopt;
  }
  args.out_path = argv[3];
  return args;
}

/// Opens the transport: the AT-serial USB-CAN module for /dev/... paths,
/// SocketCAN otherwise.
inline std::shared_ptr<robstride::CanInterface> MakeCanInterface(
    const std::string& interface_name, std::uint8_t motor_id) {
  if (interface_name.starts_with("/dev/")) {
    return std::make_shared<robstride::AtSerialCanInterface>(interface_name);
  }
  auto socket_can =
      std::make_shared<robstride::SocketCanInterface>(interface_name);
  socket_can->SetMotorIdFilter(motor_id);
  return socket_can;
}

/// Creates a RobstrideMotor for an RS02 with the default host id and
/// response timeout.
inline robstride::RobstrideMotor MakeRs02Motor(
    const std::shared_ptr<robstride::CanInterface>& can,
    std::uint8_t motor_id) {
  robstride::RobstrideMotor::Config config;
  config.motor_id = motor_id;
  config.actuator_type = robstride::ActuatorType::kRs02;
  return robstride::RobstrideMotor{can, config};
}

/// Opens the output CSV and writes the header row. `extra_header` appends
/// example-specific columns (e.g. ",iq"). Prints an error message when
/// the file cannot be opened; check the returned stream.
inline std::ofstream OpenCsv(const std::string& out_path,
                             const char* extra_header = "") {
  std::ofstream csv(out_path);
  if (csv) {
    csv << "t,target,position,velocity,torque,temperature" << extra_header
        << '\n';
  } else {
    std::cerr << "error: cannot open '" << out_path << "' for writing\n";
  }
  return csv;
}

/// Runs `profile` at 100 Hz: every cycle sends the segment target via
/// `send_command` (which must return the feedback answered by the motor)
/// and appends a CSV row. `append_extra(csv)` is called after the
/// standard columns so an example can append its extra columns.
template <typename SendCommand, typename AppendExtra>
void RunProfile(const std::vector<Segment>& profile,
                robstride::PositionUnwrapper& unwrapper, std::ofstream& csv,
                SendCommand&& send_command, AppendExtra&& append_extra) {
  const auto start = Clock::now();
  for (const auto& segment : profile) {
    const double segment_end = SecondsSince(start) + segment.duration_s;
    while (SecondsSince(start) < segment_end) {
      const auto cycle_end =
          Clock::now() + std::chrono::duration_cast<Clock::duration>(
                             std::chrono::duration<double>(kSamplePeriodS));
      const robstride::Feedback feedback = send_command(segment.target);
      csv << SecondsSince(start) << ',' << segment.target << ','
          << unwrapper.Update(feedback.position) << ',' << feedback.velocity
          << ',' << feedback.torque << ',' << feedback.temperature;
      append_extra(csv);
      csv << '\n';
      std::this_thread::sleep_until(cycle_end);
    }
  }
}

/// RunProfile without extra CSV columns.
template <typename SendCommand>
void RunProfile(const std::vector<Segment>& profile,
                robstride::PositionUnwrapper& unwrapper, std::ofstream& csv,
                SendCommand&& send_command) {
  RunProfile(profile, unwrapper, csv, std::forward<SendCommand>(send_command),
             [](std::ofstream&) {});
}

}  // namespace tracking_capture
