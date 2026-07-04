// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include "robstride_driver/can_interface.hpp"

namespace robstride {

/// Linux SocketCAN transport (PF_CAN / SOCK_RAW / CAN_RAW).
///
/// Only extended data frames are received; an optional kernel-side filter
/// can restrict reception to frames originated by a single motor id
/// (identifier bits 15-8).
class SocketCanInterface : public CanInterface {
 public:
  /// Opens a raw CAN socket bound to `interface_name` (e.g. "can0").
  /// Throws std::runtime_error if the socket cannot be opened or bound.
  explicit SocketCanInterface(const std::string& interface_name);

  SocketCanInterface(const SocketCanInterface&) = delete;
  SocketCanInterface& operator=(const SocketCanInterface&) = delete;

  ~SocketCanInterface() override;

  /// Installs a kernel CAN filter that accepts only extended frames whose
  /// identifier bits 15-8 equal `motor_id` (i.e. frames sent by that
  /// motor). Call before the first Receive.
  void SetMotorIdFilter(std::uint8_t motor_id);

  void Send(const CanFrame& frame) override;
  std::optional<CanFrame> Receive(std::chrono::milliseconds timeout) override;

 private:
  int socket_fd_ = -1;
  std::string interface_name_;
};

}  // namespace robstride
