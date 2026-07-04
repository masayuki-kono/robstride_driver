// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/at_serial_can_interface.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

// termios2 (BOTHER) is used instead of <termios.h> so that any baud rate,
// including the module's 921600 default, can be configured uniformly.
#include <asm/termbits.h>

#include <cerrno>
#include <cstring>
#include <system_error>

namespace robstride {

namespace at_serial {

namespace {

constexpr std::size_t kHeaderSize = 7;  // "AT" + 4-byte id + dlc
constexpr std::size_t kTailSize = 2;    // "\r\n"

}  // namespace

std::vector<std::uint8_t> EncodeFrame(const CanFrame& frame) {
  // Bit2 of the packed identifier marks an extended data frame; see the
  // worked example in the RS02 User Manual (3.3.5).
  const std::uint32_t packed = ((frame.id & 0x1FFFFFFFU) << 3) | 0x4U;

  std::vector<std::uint8_t> out;
  out.reserve(kHeaderSize + frame.dlc + kTailSize);
  out.push_back('A');
  out.push_back('T');
  out.push_back(static_cast<std::uint8_t>(packed >> 24));
  out.push_back(static_cast<std::uint8_t>(packed >> 16));
  out.push_back(static_cast<std::uint8_t>(packed >> 8));
  out.push_back(static_cast<std::uint8_t>(packed));
  out.push_back(frame.dlc);
  out.insert(out.end(), frame.data.begin(), frame.data.begin() + frame.dlc);
  out.push_back('\r');
  out.push_back('\n');
  return out;
}

void FrameParser::Push(const std::uint8_t* data, std::size_t size) {
  buffer_.insert(buffer_.end(), data, data + size);
}

std::optional<CanFrame> FrameParser::Poll() {
  while (buffer_.size() >= kHeaderSize + kTailSize) {
    if (buffer_[0] != 'A' || buffer_[1] != 'T') {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::uint8_t dlc = buffer_[6];
    if (dlc > 8) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t total = kHeaderSize + dlc + kTailSize;
    if (buffer_.size() < total) {
      return std::nullopt;  // wait for more bytes
    }
    if (buffer_[total - 2] != '\r' || buffer_[total - 1] != '\n') {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::uint32_t packed = (static_cast<std::uint32_t>(buffer_[2]) << 24) |
                                 (static_cast<std::uint32_t>(buffer_[3]) << 16) |
                                 (static_cast<std::uint32_t>(buffer_[4]) << 8) |
                                 static_cast<std::uint32_t>(buffer_[5]);
    CanFrame frame;
    frame.id = (packed >> 3) & 0x1FFFFFFFU;
    frame.dlc = dlc;
    std::memcpy(frame.data.data(), buffer_.data() + kHeaderSize, dlc);
    buffer_.erase(buffer_.begin(), buffer_.begin() + total);
    return frame;
  }
  return std::nullopt;
}

}  // namespace at_serial

namespace {

[[noreturn]] void ThrowErrno(const std::string& what) {
  throw std::system_error(errno, std::generic_category(), what);
}

}  // namespace

AtSerialCanInterface::AtSerialCanInterface(const std::string& device,
                                           int baud_rate)
    : device_(device) {
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    ThrowErrno("robstride: failed to open serial device '" + device + "'");
  }

  struct termios2 tio{};
  if (::ioctl(fd_, TCGETS2, &tio) < 0) {
    ::close(fd_);
    fd_ = -1;
    ThrowErrno("robstride: TCGETS2 failed on '" + device + "'");
  }
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL | BOTHER;
  tio.c_ispeed = static_cast<speed_t>(baud_rate);
  tio.c_ospeed = static_cast<speed_t>(baud_rate);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (::ioctl(fd_, TCSETS2, &tio) < 0) {
    ::close(fd_);
    fd_ = -1;
    ThrowErrno("robstride: TCSETS2 failed on '" + device + "'");
  }
  ::ioctl(fd_, TCFLSH, TCIOFLUSH);  // drop stale bytes from previous sessions
}

AtSerialCanInterface::~AtSerialCanInterface() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void AtSerialCanInterface::Send(const CanFrame& frame) {
  const auto packet = at_serial::EncodeFrame(frame);
  std::size_t sent = 0;
  while (sent < packet.size()) {
    const ssize_t written = ::write(fd_, packet.data() + sent,
                                    packet.size() - sent);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      ThrowErrno("robstride: failed to write to '" + device_ + "'");
    }
    sent += static_cast<std::size_t>(written);
  }
}

std::optional<CanFrame> AtSerialCanInterface::Receive(
    std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    if (auto frame = parser_.Poll()) {
      return frame;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return std::nullopt;
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    struct timeval tv{};
    tv.tv_sec = static_cast<time_t>(remaining.count() / 1000000);
    tv.tv_usec = static_cast<suseconds_t>(remaining.count() % 1000000);

    const int ready = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      ThrowErrno("robstride: select() failed on '" + device_ + "'");
    }
    if (ready == 0) {
      return std::nullopt;  // timeout
    }

    std::uint8_t chunk[256];
    const ssize_t received = ::read(fd_, chunk, sizeof(chunk));
    if (received < 0) {
      if (errno == EINTR) {
        continue;
      }
      ThrowErrno("robstride: failed to read from '" + device_ + "'");
    }
    if (received > 0) {
      parser_.Push(chunk, static_cast<std::size_t>(received));
    }
  }
}

}  // namespace robstride
