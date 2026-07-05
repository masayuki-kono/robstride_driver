// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Tests for the RobStride USB-CAN module serial (AT) framing.

#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "fixtures/at_serial_samples.hpp"
#include "robstride_driver/at_serial_can_interface.hpp"

namespace robstride::at_serial {
namespace {

using robstride::test_fixtures::at_serial_samples::manual_run_mode_frame;

TEST(AtSerialEncode, MatchesManualExample) {
  CanFrame frame;
  frame.id = 0x1200FD01;
  frame.dlc = 8;
  frame.data = {0x05, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

  EXPECT_EQ(encode_frame(frame),
            std::vector<std::uint8_t>(manual_run_mode_frame.begin(),
                                      manual_run_mode_frame.end()));
}

TEST(AtSerialEncode, ShortDlc) {
  CanFrame frame;
  frame.id = 0x1;
  frame.dlc = 2;
  frame.data = {0xAA, 0xBB};

  const auto packet = encode_frame(frame);
  // "AT" + 4-byte id + dlc + 2 data bytes + "\r\n"
  ASSERT_EQ(packet.size(), 11U);
  EXPECT_EQ(packet[6], 2U);
  EXPECT_EQ(packet[7], 0xAA);
  EXPECT_EQ(packet[8], 0xBB);
}

TEST(AtSerialParse, DecodesManualExample) {
  FrameParser parser;
  parser.push(manual_run_mode_frame.data(), manual_run_mode_frame.size());

  const auto frame = parser.poll();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x1200FD01U);
  EXPECT_EQ(frame->dlc, 8U);
  EXPECT_EQ(frame->data[0], 0x05);
  EXPECT_EQ(frame->data[1], 0x70);
  EXPECT_FALSE(parser.poll().has_value());
}

TEST(AtSerialParse, HandlesSplitDelivery) {
  FrameParser parser;
  // Feed the frame one byte at a time, as a serial port may deliver it.
  for (const std::uint8_t byte : manual_run_mode_frame) {
    EXPECT_FALSE(parser.poll().has_value());
    parser.push(&byte, 1);
  }
  EXPECT_TRUE(parser.poll().has_value());
}

TEST(AtSerialParse, SkipsGarbageAndCommandReplies) {
  FrameParser parser;
  const std::vector<std::uint8_t> noise = {'O', 'K', '\r', '\n', 0x00, 0xFF};
  parser.push(noise.data(), noise.size());
  parser.push(manual_run_mode_frame.data(), manual_run_mode_frame.size());

  const auto frame = parser.poll();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x1200FD01U);
}

TEST(AtSerialParse, ResyncsAfterCorruptedTail) {
  FrameParser parser;
  auto corrupted = manual_run_mode_frame;
  corrupted[15] = 0x00;  // break the '\r' of the tail
  parser.push(corrupted.data(), corrupted.size());
  parser.push(manual_run_mode_frame.data(), manual_run_mode_frame.size());

  const auto frame = parser.poll();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x1200FD01U);
  EXPECT_FALSE(parser.poll().has_value());
}

TEST(AtSerialParse, RoundTripDataContainingTailBytes) {
  // Payload bytes 0x0d 0x0a must not confuse the length-based parser.
  CanFrame frame;
  frame.id = 0x02017F01;
  frame.dlc = 8;
  frame.data = {0x0d, 0x0a, 0x0d, 0x0a, 0x41, 0x54, 0x0d, 0x0a};

  const auto packet = encode_frame(frame);
  FrameParser parser;
  parser.push(packet.data(), packet.size());

  const auto decoded = parser.poll();
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->id, frame.id);
  EXPECT_EQ(decoded->data, frame.data);
}

TEST(AtSerialParse, ParsesBackToBackFrames) {
  FrameParser parser;
  parser.push(manual_run_mode_frame.data(), manual_run_mode_frame.size());
  parser.push(manual_run_mode_frame.data(), manual_run_mode_frame.size());

  EXPECT_TRUE(parser.poll().has_value());
  EXPECT_TRUE(parser.poll().has_value());
  EXPECT_FALSE(parser.poll().has_value());
}

}  // namespace
}  // namespace robstride::at_serial
