// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#include "robstride_driver/socket_can_interface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace robstride {

namespace {

[[noreturn]] void ThrowErrno(const std::string& what) {
  throw std::system_error(errno, std::generic_category(), what);
}

}  // namespace

SocketCanInterface::SocketCanInterface(const std::string& interface_name)
    : interface_name_(interface_name) {
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    ThrowErrno("robstride: failed to open CAN socket");
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    ThrowErrno("robstride: unknown CAN interface '" + interface_name + "'");
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr),
             sizeof(addr)) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    ThrowErrno("robstride: failed to bind CAN socket to '" + interface_name +
               "'");
  }
}

SocketCanInterface::~SocketCanInterface() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
}

// Mutates kernel-side socket state, so it stays non-const even though it
// does not touch any data member.
// NOLINTNEXTLINE(readability-make-member-function-const)
void SocketCanInterface::SetMotorIdFilter(std::uint8_t motor_id) {
  struct can_filter filter {};
  filter.can_id = (static_cast<canid_t>(motor_id) << 8) | CAN_EFF_FLAG;
  filter.can_mask = (0xFFU << 8) | CAN_EFF_FLAG;
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                   sizeof(filter)) < 0) {
    ThrowErrno("robstride: failed to set CAN filter");
  }
}

void SocketCanInterface::Send(const CanFrame& frame) {
  struct can_frame raw {};
  raw.can_id = (frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  raw.can_dlc = frame.dlc;
  std::memcpy(raw.data, frame.data.data(), frame.data.size());

  const ssize_t written = ::write(socket_fd_, &raw, sizeof(raw));
  if (std::cmp_not_equal(written, sizeof(raw))) {
    ThrowErrno("robstride: failed to send CAN frame on '" + interface_name_ +
               "'");
  }
}

std::optional<CanFrame> SocketCanInterface::Receive(
    std::chrono::milliseconds timeout) {
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);

  struct timeval tv {};
  tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);

  const int ready = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ready < 0) {
    ThrowErrno("robstride: select() failed on CAN socket");
  }
  if (ready == 0) {
    return std::nullopt;  // timeout
  }

  struct can_frame raw {};
  const ssize_t received = ::read(socket_fd_, &raw, sizeof(raw));
  if (received < 0) {
    ThrowErrno("robstride: failed to read CAN frame");
  }
  if (std::cmp_not_equal(received, sizeof(raw)) ||
      (raw.can_id & CAN_EFF_FLAG) == 0) {
    // Ignore short reads and non-extended frames.
    return std::nullopt;
  }

  CanFrame frame;
  frame.id = raw.can_id & CAN_EFF_MASK;
  frame.dlc = raw.can_dlc;
  std::memcpy(frame.data.data(), raw.data, frame.data.size());
  return frame;
}

}  // namespace robstride
