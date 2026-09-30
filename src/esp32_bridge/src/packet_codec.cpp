#include "esp32_bridge/packet_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace esp32_bridge
{
namespace
{

template<typename IntegerT>
bool appendFixed(
  std::vector<std::uint8_t> & output, const double value, const double units_per_lsb)
{
  if (!std::isfinite(value) || !std::isfinite(units_per_lsb) || units_per_lsb <= 0.0) {
    return false;
  }

  const double rounded = std::round(value / units_per_lsb);
  const double saturated = std::clamp(
    rounded,
    static_cast<double>(std::numeric_limits<IntegerT>::lowest()),
    static_cast<double>(std::numeric_limits<IntegerT>::max()));
  using UnsignedT = std::make_unsigned_t<IntegerT>;
  const auto encoded = static_cast<UnsignedT>(static_cast<IntegerT>(saturated));
  for (std::size_t index = sizeof(IntegerT); index > 0; --index) {
    output.push_back(static_cast<std::uint8_t>(encoded >> ((index - 1U) * 8U)));
  }
  return true;
}

std::uint16_t readUint16(const std::vector<std::uint8_t> & input, const std::size_t offset)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(input[offset]) << 8U) |
    static_cast<std::uint16_t>(input[offset + 1U]));
}

std::int16_t readInt16(const std::vector<std::uint8_t> & input, const std::size_t offset)
{
  const auto raw = readUint16(input, offset);
  const auto signed_value = raw <= 0x7FFFU ?
    static_cast<std::int32_t>(raw) : static_cast<std::int32_t>(raw) - 0x10000;
  return static_cast<std::int16_t>(signed_value);
}

bool isKnownMessageId(const std::uint8_t value)
{
  switch (static_cast<MessageId>(value)) {
    case MessageId::kBlueRovOdometry:
    case MessageId::kTorpedoOdometry:
    case MessageId::kControlCommand:
    case MessageId::kTorpedoActuator:
    case MessageId::kControllerStatus:
      return true;
  }
  return false;
}

}  // namespace

std::size_t expectedPayloadSize(const MessageId message_id)
{
  switch (message_id) {
    case MessageId::kBlueRovOdometry: return 32;
    case MessageId::kTorpedoOdometry: return 32;
    case MessageId::kControlCommand: return 8;
    case MessageId::kTorpedoActuator: return 12;
    case MessageId::kControllerStatus: return 8;
  }
  throw std::invalid_argument("unknown UART message ID");
}

std::vector<std::uint8_t> encodeFrame(
  const MessageId message_id, const std::vector<std::uint8_t> & payload)
{
  if (payload.size() != expectedPayloadSize(message_id)) {
    throw std::invalid_argument("payload size does not match UART message ID");
  }

  std::vector<std::uint8_t> frame;
  frame.reserve(kFrameHeaderSize + payload.size());
  frame.push_back(kSyncFirst);
  frame.push_back(kSyncSecond);
  frame.push_back(static_cast<std::uint8_t>(message_id));
  frame.push_back(static_cast<std::uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  return frame;
}

bool appendFixedInt16(
  std::vector<std::uint8_t> & output, const double value, const double units_per_lsb)
{
  return appendFixed<std::int16_t>(output, value, units_per_lsb);
}

bool appendFixedInt32(
  std::vector<std::uint8_t> & output, const double value, const double units_per_lsb)
{
  return appendFixed<std::int32_t>(output, value, units_per_lsb);
}

bool appendFixedUint16(
  std::vector<std::uint8_t> & output, const double value, const double units_per_lsb)
{
  return appendFixed<std::uint16_t>(output, value, units_per_lsb);
}

bool decodeTorpedoActuator(
  const std::vector<std::uint8_t> & payload, TorpedoActuator & actuator)
{
  if (payload.size() != expectedPayloadSize(MessageId::kTorpedoActuator)) {
    return false;
  }
  actuator.sequence = payload[0];
  actuator.state = payload[1];
  actuator.thrust = readUint16(payload, 2);
  actuator.fin_top = static_cast<double>(readInt16(payload, 4)) * 0.001;
  actuator.fin_bottom = static_cast<double>(readInt16(payload, 6)) * 0.001;
  actuator.fin_left = static_cast<double>(readInt16(payload, 8)) * 0.001;
  actuator.fin_right = static_cast<double>(readInt16(payload, 10)) * 0.001;
  return actuator.state <= 4U && actuator.thrust <= 1000U &&
         std::abs(actuator.fin_top) <= 0.5 && std::abs(actuator.fin_bottom) <= 0.5 &&
         std::abs(actuator.fin_left) <= 0.5 && std::abs(actuator.fin_right) <= 0.5;
}

bool decodeControllerStatus(
  const std::vector<std::uint8_t> & payload, ControllerStatus & status)
{
  if (payload.size() != expectedPayloadSize(MessageId::kControllerStatus)) {
    return false;
  }
  status.version = payload[0];
  status.heartbeat_sequence = payload[1];
  status.state = payload[2];
  status.mode = payload[3];
  status.flags = payload[4];
  status.last_error = payload[5];
  status.uptime_seconds_low16 = readUint16(payload, 6);
  return status.version == 1U && status.state <= 4U && status.mode <= 2U;
}

void UartFrameParser::checkTimeout(const std::uint64_t now_ms)
{
  if (frame_in_progress_ && now_ms - frame_start_ms_ >= kFrameTimeoutMs) {
    buffer_.clear();
    frame_in_progress_ = false;
    ++timeout_count_;
  }
}

std::vector<UartFrame> UartFrameParser::feed(
  const std::uint8_t * data, const std::size_t size, const std::uint64_t now_ms)
{
  checkTimeout(now_ms);
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<UartFrame> frames;
  const std::array<std::uint8_t, 2> sync_bytes{kSyncFirst, kSyncSecond};
  while (true) {
    const auto sync = std::search(
      buffer_.begin(), buffer_.end(), sync_bytes.begin(), sync_bytes.end());
    if (sync == buffer_.end()) {
      const bool keep_first_sync = !buffer_.empty() && buffer_.back() == kSyncFirst;
      buffer_.clear();
      if (keep_first_sync) {
        buffer_.push_back(kSyncFirst);
        frame_start_ms_ = now_ms;
        frame_in_progress_ = true;
      } else {
        frame_in_progress_ = false;
      }
      break;
    }

    if (sync != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), sync);
      ++framing_error_count_;
    }
    if (!frame_in_progress_) {
      frame_start_ms_ = now_ms;
      frame_in_progress_ = true;
    }
    if (buffer_.size() < kFrameHeaderSize) {
      break;
    }

    const auto raw_id = buffer_[2];
    if (!isKnownMessageId(raw_id)) {
      buffer_.erase(buffer_.begin());
      frame_in_progress_ = false;
      ++framing_error_count_;
      continue;
    }
    const auto message_id = static_cast<MessageId>(raw_id);
    const auto payload_size = static_cast<std::size_t>(buffer_[3]);
    if (payload_size != expectedPayloadSize(message_id)) {
      buffer_.erase(buffer_.begin());
      frame_in_progress_ = false;
      ++framing_error_count_;
      continue;
    }

    const std::size_t frame_size = kFrameHeaderSize + payload_size;
    if (buffer_.size() < frame_size) {
      break;
    }
    UartFrame frame;
    frame.message_id = message_id;
    frame.payload.assign(
      buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameHeaderSize),
      buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    frames.push_back(std::move(frame));
    buffer_.erase(
      buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    frame_in_progress_ = false;
  }
  return frames;
}

std::uint64_t UartFrameParser::framingErrorCount() const
{
  return framing_error_count_;
}

std::uint64_t UartFrameParser::timeoutCount() const
{
  return timeout_count_;
}

}  // namespace esp32_bridge
