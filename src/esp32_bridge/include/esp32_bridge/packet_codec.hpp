#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace esp32_bridge
{

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kMaximumPayloadSize = 512;

enum class MessageType : std::uint8_t
{
  kHeartbeat = 0x01,
  kBlueRovPosition = 0x10,
  kBlueRovVelocity = 0x11,
  kBlueRovAttitude = 0x12,
  kPressure = 0x13,
  kDvlVelocity = 0x14,
  kDepth = 0x15,
  kTorpedoState = 0x20,
  kMissionGoal = 0x21,
  kThrusterOutput = 0x80,
  kCommunicationStatus = 0xE0,
  kConfiguration = 0xF0,
};

struct Packet
{
  MessageType type{MessageType::kHeartbeat};
  std::uint8_t flags{0};
  std::uint16_t sequence{0};
  std::uint32_t timestamp_us{0};
  std::vector<std::uint8_t> payload;
};

std::uint16_t crc16CcittFalse(const std::uint8_t * data, std::size_t size);
std::vector<std::uint8_t> encodePacket(const Packet & packet);

void appendFloat32BigEndian(std::vector<std::uint8_t> & output, float value);
bool readFloat32BigEndian(
  const std::vector<std::uint8_t> & input, std::size_t offset, float & value);

class PacketParser
{
public:
  std::vector<Packet> feed(const std::uint8_t * data, std::size_t size);
  std::uint64_t crcErrorCount() const;
  std::uint64_t framingErrorCount() const;

private:
  std::vector<std::uint8_t> buffer_;
  std::uint64_t crc_error_count_{0};
  std::uint64_t framing_error_count_{0};
};

}  // namespace esp32_bridge
