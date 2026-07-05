// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <optional>

#include "robstride_driver/protocol.hpp"

namespace robstride {

/// Abstract CAN transport used by RobstrideMotor.
///
/// Implementations must transmit/receive CAN 2.0B extended data frames.
/// The `CanFrame::id` field carries the bare 29-bit identifier; transport
/// flag bits (such as Linux CAN_EFF_FLAG) are the implementation's concern.
class CanInterface {
 public:
  virtual ~CanInterface() = default;

  /// Sends one frame. Throws std::runtime_error on transport failure.
  virtual void send(const CanFrame& frame) = 0;

  /// Receives one frame, waiting up to `timeout`. Returns nullopt on
  /// timeout. Throws std::runtime_error on transport failure.
  virtual std::optional<CanFrame> receive(
      std::chrono::milliseconds timeout) = 0;
};

}  // namespace robstride
