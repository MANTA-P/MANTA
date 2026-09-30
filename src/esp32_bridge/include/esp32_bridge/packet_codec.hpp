#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esp32_bridge
{

// Wire frame: AA 55 | MESSAGE_ID | PAYLOAD_LENGTH | PAYLOAD
enum class MessageId : std::uint8_t
{
  kBlueRovOdometry = 0x10,
  kTorpedoOdometry = 0x20,
  kControlCommand = 0x30,
  kTorpedoActuator = 0x80,
  kControllerStatus = 0x81,
};

struct UartFrame
{
  MessageId message_id{MessageId::kBlueRovOdometry};
  std::vector<std::uint8_t> payload;
};

struct TorpedoActuator
{
  std::uint8_t sequence{0};
  std::uint8_t state{0};
  std::uint16_t thrust{0};
  double fin_top{0.0};
  double fin_bottom{0.0};
  double fin_left{0.0};
  double fin_right{0.0};
};

struct ControllerStatus
{
  std::uint8_t version{0};
  std::uint8_t heartbeat_sequence{0};
  std::uint8_t state{0};
  std::uint8_t mode{0};
  std::uint8_t flags{0};
  std::uint8_t last_error{0};
  std::uint16_t uptime_seconds_low16{0};
};

constexpr std::uint8_t kSyncFirst = 0xAA;
constexpr std::uint8_t kSyncSecond = 0x55;
constexpr std::size_t kFrameHeaderSize = 4;
constexpr std::uint64_t kFrameTimeoutMs = 50;

std::size_t expectedPayloadSize(MessageId message_id);
std::vector<std::uint8_t> encodeFrame(
  MessageId message_id, const std::vector<std::uint8_t> & payload);

// Round to the nearest integer and saturate to the destination range. Return
// false only for a non-finite value or invalid scale.
bool appendFixedInt16(
  std::vector<std::uint8_t> & output, double value, double units_per_lsb);
bool appendFixedInt32(
  std::vector<std::uint8_t> & output, double value, double units_per_lsb);
bool appendFixedUint16(
  std::vector<std::uint8_t> & output, double value, double units_per_lsb);

bool decodeTorpedoActuator(
  const std::vector<std::uint8_t> & payload, TorpedoActuator & actuator);
bool decodeControllerStatus(
  const std::vector<std::uint8_t> & payload, ControllerStatus & status);

class UartFrameParser
{
public:
  std::vector<UartFrame> feed(
    const std::uint8_t * data, std::size_t size, std::uint64_t now_ms);
  void checkTimeout(std::uint64_t now_ms);

  std::uint64_t framingErrorCount() const;
  std::uint64_t timeoutCount() const;

private:
  std::vector<std::uint8_t> buffer_;
  std::uint64_t frame_start_ms_{0};
  bool frame_in_progress_{false};
  std::uint64_t framing_error_count_{0};
  std::uint64_t timeout_count_{0};
};

}  // namespace esp32_bridge
