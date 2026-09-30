#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "esp32_bridge/packet_codec.hpp"

namespace esp32_bridge
{

TEST(PacketCodec, EncodesBigEndianSignedFixedPoint)
{
  std::vector<std::uint8_t> data;
  ASSERT_TRUE(appendFixedInt16(data, -1.0, 0.001));
  EXPECT_EQ(data, (std::vector<std::uint8_t>{0xFC, 0x18}));
}

TEST(PacketCodec, RoundsToNearestInteger)
{
  std::vector<std::uint8_t> data;
  ASSERT_TRUE(appendFixedInt32(data, 1.2346, 0.001));
  EXPECT_EQ(data, (std::vector<std::uint8_t>{0x00, 0x00, 0x04, 0xD3}));
}

TEST(PacketCodec, EncodesQuaternionQ14)
{
  std::vector<std::uint8_t> data;
  ASSERT_TRUE(appendFixedInt16(data, 1.0, 1.0 / 16384.0));
  ASSERT_TRUE(appendFixedInt16(data, -1.0, 1.0 / 16384.0));
  EXPECT_EQ(data, (std::vector<std::uint8_t>{0x40, 0x00, 0xC0, 0x00}));
}

TEST(PacketCodec, SaturatesFiniteValuesAndRejectsNonFiniteValues)
{
  std::vector<std::uint8_t> data;
  ASSERT_TRUE(appendFixedInt16(data, 32.768, 0.001));
  ASSERT_TRUE(appendFixedInt16(data, -32.769, 0.001));
  ASSERT_TRUE(appendFixedUint16(data, 70000.0, 1.0));
  EXPECT_EQ(
    data,
    (std::vector<std::uint8_t>{0x7F, 0xFF, 0x80, 0x00, 0xFF, 0xFF}));

  EXPECT_FALSE(appendFixedInt16(
    data, std::numeric_limits<double>::infinity(), 0.001));
  EXPECT_FALSE(appendFixedInt16(
    data, std::numeric_limits<double>::quiet_NaN(), 0.001));
}

TEST(PacketCodec, EncodesControlCommandFrame)
{
  const std::vector<std::uint8_t> payload{
    0x01, 0x2A, 0x01, 0x02, 0x03, 0xE8, 0x00, 0x00};
  EXPECT_EQ(
    encodeFrame(MessageId::kControlCommand, payload),
    (std::vector<std::uint8_t>{
      0xAA, 0x55, 0x30, 0x08,
      0x01, 0x2A, 0x01, 0x02, 0x03, 0xE8, 0x00, 0x00}));
}

TEST(PacketCodec, RejectsPayloadLengthMismatch)
{
  EXPECT_THROW(
    encodeFrame(MessageId::kTorpedoOdometry, std::vector<std::uint8_t>(31)),
    std::invalid_argument);
}

TEST(PacketCodec, DecodesTorpedoActuator)
{
  const std::vector<std::uint8_t> payload{
    0x21, 0x01, 0x03, 0xE8, 0x01, 0xF4,
    0xFE, 0x0C, 0x00, 0x64, 0xFF, 0x9C};
  TorpedoActuator actuator;
  ASSERT_TRUE(decodeTorpedoActuator(payload, actuator));
  EXPECT_EQ(actuator.sequence, 0x21);
  EXPECT_EQ(actuator.state, 1);
  EXPECT_EQ(actuator.thrust, 1000);
  EXPECT_DOUBLE_EQ(actuator.fin_top, 0.5);
  EXPECT_DOUBLE_EQ(actuator.fin_bottom, -0.5);
  EXPECT_DOUBLE_EQ(actuator.fin_left, 0.1);
  EXPECT_DOUBLE_EQ(actuator.fin_right, -0.1);
}

TEST(PacketCodec, RejectsUnsafeActuatorRange)
{
  const std::vector<std::uint8_t> payload{
    0x00, 0x01, 0x03, 0xE9, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  TorpedoActuator actuator;
  EXPECT_FALSE(decodeTorpedoActuator(payload, actuator));
}

TEST(PacketCodec, DecodesControllerStatus)
{
  const std::vector<std::uint8_t> payload{
    0x01, 0x7B, 0x03, 0x02, 0x09, 0x04, 0x12, 0x34};
  ControllerStatus status;
  ASSERT_TRUE(decodeControllerStatus(payload, status));
  EXPECT_EQ(status.heartbeat_sequence, 0x7B);
  EXPECT_EQ(status.state, 3);
  EXPECT_EQ(status.mode, 2);
  EXPECT_EQ(status.flags, 0x09);
  EXPECT_EQ(status.last_error, 4);
  EXPECT_EQ(status.uptime_seconds_low16, 0x1234);
}

TEST(UartFrameParser, HandlesPartialAndCombinedFrames)
{
  const auto actuator = encodeFrame(
    MessageId::kTorpedoActuator, std::vector<std::uint8_t>(12, 0));
  const auto status = encodeFrame(
    MessageId::kControllerStatus,
    std::vector<std::uint8_t>{1, 0, 0, 0, 0, 0, 0, 0});
  UartFrameParser parser;

  EXPECT_TRUE(parser.feed(actuator.data(), 3, 100).empty());
  std::vector<std::uint8_t> remainder(actuator.begin() + 3, actuator.end());
  remainder.insert(remainder.end(), status.begin(), status.end());
  const auto frames = parser.feed(remainder.data(), remainder.size(), 110);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].message_id, MessageId::kTorpedoActuator);
  EXPECT_EQ(frames[1].message_id, MessageId::kControllerStatus);
}

TEST(UartFrameParser, KeepsSyncBytesInsidePayload)
{
  std::vector<std::uint8_t> payload(12, 0);
  payload[6] = 0xAA;
  payload[7] = 0x55;
  const auto bytes = encodeFrame(MessageId::kTorpedoActuator, payload);
  UartFrameParser parser;
  const auto frames = parser.feed(bytes.data(), bytes.size(), 100);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].payload, payload);
}

TEST(UartFrameParser, RecoversFromGarbageAndBadLength)
{
  const std::vector<std::uint8_t> garbage_and_bad{
    0x12, 0x34, 0xAA, 0x55, 0x80, 0x08, 0x99};
  const auto valid = encodeFrame(
    MessageId::kControllerStatus,
    std::vector<std::uint8_t>{1, 0, 0, 0, 0, 0, 0, 0});
  std::vector<std::uint8_t> bytes = garbage_and_bad;
  bytes.insert(bytes.end(), valid.begin(), valid.end());

  UartFrameParser parser;
  const auto frames = parser.feed(bytes.data(), bytes.size(), 100);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].message_id, MessageId::kControllerStatus);
  EXPECT_GE(parser.framingErrorCount(), 1U);
}

TEST(UartFrameParser, DropsIncompleteFrameAfterTimeout)
{
  const std::vector<std::uint8_t> partial{0xAA, 0x55, 0x80, 0x0C, 0x01};
  UartFrameParser parser;
  EXPECT_TRUE(parser.feed(partial.data(), partial.size(), 100).empty());
  parser.checkTimeout(150);
  EXPECT_EQ(parser.timeoutCount(), 1U);

  const auto valid = encodeFrame(
    MessageId::kControllerStatus,
    std::vector<std::uint8_t>{1, 0, 0, 0, 0, 0, 0, 0});
  const auto frames = parser.feed(valid.data(), valid.size(), 151);
  ASSERT_EQ(frames.size(), 1U);
}

}  // namespace esp32_bridge
