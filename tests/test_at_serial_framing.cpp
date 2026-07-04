// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Tests for the RobStride USB-CAN module serial (AT) framing.

#include <gtest/gtest.h>

#include <vector>

#include "robstride_driver/at_serial_can_interface.hpp"

namespace robstride::at_serial {
namespace {

// Worked example from the RS02 User Manual, section 3.3.5:
//   41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
// carries CAN id 0x1200FD01 (comm type 18, host 0xFD, motor 0x01).
const std::vector<std::uint8_t> kManualExample = {
    0x41, 0x54, 0x90, 0x07, 0xe8, 0x0c, 0x08, 0x05, 0x70,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};

TEST(AtSerialEncode, MatchesManualExample) {
  CanFrame frame;
  frame.id = 0x1200FD01;
  frame.dlc = 8;
  frame.data = {0x05, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

  EXPECT_EQ(EncodeFrame(frame), kManualExample);
}

TEST(AtSerialEncode, ShortDlc) {
  CanFrame frame;
  frame.id = 0x1;
  frame.dlc = 2;
  frame.data = {0xAA, 0xBB};

  const auto packet = EncodeFrame(frame);
  // "AT" + 4-byte id + dlc + 2 data bytes + "\r\n"
  ASSERT_EQ(packet.size(), 11U);
  EXPECT_EQ(packet[6], 2U);
  EXPECT_EQ(packet[7], 0xAA);
  EXPECT_EQ(packet[8], 0xBB);
}

TEST(AtSerialParse, DecodesManualExample) {
  FrameParser parser;
  parser.Push(kManualExample.data(), kManualExample.size());

  const auto frame = parser.Poll();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x1200FD01U);
  EXPECT_EQ(frame->dlc, 8U);
  EXPECT_EQ(frame->data[0], 0x05);
  EXPECT_EQ(frame->data[1], 0x70);
  EXPECT_FALSE(parser.Poll().has_value());
}

TEST(AtSerialParse, HandlesSplitDelivery) {
  FrameParser parser;
  // Feed the frame one byte at a time, as a serial port may deliver it.
  for (const std::uint8_t byte : kManualExample) {
    EXPECT_FALSE(parser.Poll().has_value());
    parser.Push(&byte, 1);
  }
  EXPECT_TRUE(parser.Poll().has_value());
}

TEST(AtSerialParse, SkipsGarbageAndCommandReplies) {
  FrameParser parser;
  const std::vector<std::uint8_t> noise = {'O', 'K', '\r', '\n', 0x00, 0xFF};
  parser.Push(noise.data(), noise.size());
  parser.Push(kManualExample.data(), kManualExample.size());

  const auto frame = parser.Poll();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x1200FD01U);
}

TEST(AtSerialParse, ResyncsAfterCorruptedTail) {
  FrameParser parser;
  auto corrupted = kManualExample;
  corrupted[15] = 0x00;  // break the '\r' of the tail
  parser.Push(corrupted.data(), corrupted.size());
  parser.Push(kManualExample.data(), kManualExample.size());

  const auto frame = parser.Poll();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x1200FD01U);
  EXPECT_FALSE(parser.Poll().has_value());
}

TEST(AtSerialParse, RoundTripDataContainingTailBytes) {
  // Payload bytes 0x0d 0x0a must not confuse the length-based parser.
  CanFrame frame;
  frame.id = 0x02017F01;
  frame.dlc = 8;
  frame.data = {0x0d, 0x0a, 0x0d, 0x0a, 0x41, 0x54, 0x0d, 0x0a};

  const auto packet = EncodeFrame(frame);
  FrameParser parser;
  parser.Push(packet.data(), packet.size());

  const auto decoded = parser.Poll();
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->id, frame.id);
  EXPECT_EQ(decoded->data, frame.data);
}

TEST(AtSerialParse, ParsesBackToBackFrames) {
  FrameParser parser;
  parser.Push(kManualExample.data(), kManualExample.size());
  parser.Push(kManualExample.data(), kManualExample.size());

  EXPECT_TRUE(parser.Poll().has_value());
  EXPECT_TRUE(parser.Poll().has_value());
  EXPECT_FALSE(parser.Poll().has_value());
}

}  // namespace
}  // namespace robstride::at_serial
