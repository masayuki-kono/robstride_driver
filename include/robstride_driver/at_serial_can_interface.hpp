// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "robstride_driver/can_interface.hpp"

namespace robstride {

namespace at_serial {

/// Serializes one CAN frame into the RobStride USB-CAN module serial
/// format:
///
///   "AT" | 4-byte big-endian ((id << 3) | 0x4) | dlc | data | "\r\n"
///
/// Reference: RS02 User Manual 3.3.5 (communication box instruction
/// example).
std::vector<std::uint8_t> encode_frame(const CanFrame& frame);

/// Incremental parser for the byte stream coming back from the USB-CAN
/// module. Feed raw bytes with push() and drain complete frames with
/// poll(). Bytes that do not form a valid frame (e.g. "OK" style command
/// replies) are skipped.
class FrameParser {
 public:
  /// Appends raw bytes received from the serial port to the internal
  /// buffer.
  void push(const std::uint8_t* data, std::size_t size);

  /// Extracts the next complete frame from the internal buffer, or
  /// nullopt if none is available yet.
  std::optional<CanFrame> poll();

 private:
  /// Unconsumed raw bytes; poll() removes parsed frames and skipped
  /// garbage from the front.
  std::vector<std::uint8_t> buffer_;
};

}  // namespace at_serial

/// CanInterface implementation for the official RobStride USB-CAN module
/// (CH340 USB-serial bridge, AT framing, 921600 baud by default).
///
/// The module transparently converts between the serial AT framing and
/// CAN 2.0B extended frames on the bus, so RobstrideMotor works on top of
/// this transport unchanged.
class AtSerialCanInterface : public CanInterface {
 public:
  /// Opens `device` (e.g. "/dev/ttyUSB0") in raw mode at `baud_rate`.
  /// Throws std::system_error if the port cannot be opened or configured.
  explicit AtSerialCanInterface(const std::string& device,
                                int baud_rate = 921600);

  AtSerialCanInterface(const AtSerialCanInterface&) = delete;
  AtSerialCanInterface& operator=(const AtSerialCanInterface&) = delete;

  ~AtSerialCanInterface() override;

  void send(const CanFrame& frame) override;
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout) override;

 private:
  /// File descriptor of the opened serial device (-1 when closed).
  int fd_ = -1;
  /// Serial device path (e.g. "/dev/ttyUSB0"), kept for error messages.
  std::string device_;
  /// Reassembles AT frames from the raw byte stream across Receive calls.
  at_serial::FrameParser parser_;
};

}  // namespace robstride
