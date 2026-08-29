#include "esp32_bridge/packet_codec.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace esp32_bridge
{
namespace
{
constexpr std::uint8_t kMagicFirst = 0xAA;
constexpr std::uint8_t kMagicSecond = 0x55;
constexpr std::size_t kHeaderSize = 13;
constexpr std::size_t kCrcSize = 2;

void appendUint16(std::vector<std::uint8_t> & out, std::uint16_t value)
{
  out.push_back(static_cast<std::uint8_t>(value >> 8));
  out.push_back(static_cast<std::uint8_t>(value));
}

void appendUint32(std::vector<std::uint8_t> & out, std::uint32_t value)
{
  out.push_back(static_cast<std::uint8_t>(value >> 24));
  out.push_back(static_cast<std::uint8_t>(value >> 16));
  out.push_back(static_cast<std::uint8_t>(value >> 8));
  out.push_back(static_cast<std::uint8_t>(value));
}

std::uint16_t readUint16(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

std::uint32_t readUint32(const std::uint8_t * data)
{
  return (static_cast<std::uint32_t>(data[0]) << 24) |
         (static_cast<std::uint32_t>(data[1]) << 16) |
         (static_cast<std::uint32_t>(data[2]) << 8) | data[3];
}
}  // namespace

std::uint16_t crc16CcittFalse(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<std::uint8_t> encodePacket(const Packet & packet)
{
  if (packet.payload.size() > kMaximumPayloadSize) {
    throw std::invalid_argument("packet payload exceeds 512 bytes");
  }
  std::vector<std::uint8_t> out;
  out.reserve(kHeaderSize + packet.payload.size() + kCrcSize);
  out.insert(out.end(), {kMagicFirst, kMagicSecond, kProtocolVersion,
    static_cast<std::uint8_t>(packet.type), packet.flags});
  appendUint16(out, packet.sequence);
  appendUint16(out, static_cast<std::uint16_t>(packet.payload.size()));
  appendUint32(out, packet.timestamp_us);
  out.insert(out.end(), packet.payload.begin(), packet.payload.end());
  appendUint16(out, crc16CcittFalse(out.data(), out.size()));
  return out;
}

void appendFloat32BigEndian(std::vector<std::uint8_t> & out, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  appendUint32(out, bits);
}

bool readFloat32BigEndian(
  const std::vector<std::uint8_t> & input, std::size_t offset, float & value)
{
  if (offset + sizeof(std::uint32_t) > input.size()) {
    return false;
  }
  const std::uint32_t bits = readUint32(input.data() + offset);
  std::memcpy(&value, &bits, sizeof(value));
  return true;
}

std::vector<Packet> PacketParser::feed(const std::uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<Packet> packets;
  const std::array<std::uint8_t, 2> magic{kMagicFirst, kMagicSecond};

  while (true) {
    const auto position = std::search(
      buffer_.begin(), buffer_.end(), magic.begin(), magic.end());
    if (position == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kMagicFirst) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }
    buffer_.erase(buffer_.begin(), position);
    if (buffer_.size() < kHeaderSize) {
      break;
    }

    const std::uint16_t payload_size = readUint16(buffer_.data() + 7);
    if (buffer_[2] != kProtocolVersion || payload_size > kMaximumPayloadSize) {
      ++framing_error_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t packet_size = kHeaderSize + payload_size + kCrcSize;
    if (buffer_.size() < packet_size) {
      break;
    }
    const auto received_crc = readUint16(buffer_.data() + packet_size - kCrcSize);
    const auto expected_crc = crc16CcittFalse(buffer_.data(), packet_size - kCrcSize);
    if (received_crc != expected_crc) {
      ++crc_error_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    Packet packet;
    packet.type = static_cast<MessageType>(buffer_[3]);
    packet.flags = buffer_[4];
    packet.sequence = readUint16(buffer_.data() + 5);
    packet.timestamp_us = readUint32(buffer_.data() + 9);
    packet.payload.assign(buffer_.begin() + kHeaderSize,
      buffer_.begin() + kHeaderSize + payload_size);
    packets.push_back(std::move(packet));
    buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
  }
  return packets;
}

std::uint64_t PacketParser::crcErrorCount() const {return crc_error_count_;}
std::uint64_t PacketParser::framingErrorCount() const {return framing_error_count_;}
}  // namespace esp32_bridge
