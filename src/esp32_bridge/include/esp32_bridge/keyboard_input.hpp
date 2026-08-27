#pragma once

#include <termios.h>

#include <cstdint>
#include <optional>

namespace esp32_bridge
{

// 터미널을 Enter 없이 한 글자씩 읽을 수 있는 모드로 전환한다.
// Ctrl+C 같은 터미널 시그널은 유지하며, 종료할 때 원래 설정을 복구한다.
class KeyboardInput
{
public:
  KeyboardInput();
  ~KeyboardInput();

  KeyboardInput(const KeyboardInput &) = delete;
  KeyboardInput & operator=(const KeyboardInput &) = delete;

  std::optional<std::uint8_t> readByte(int timeout_ms) const;

private:
  termios original_settings_{};
  bool restore_required_{false};
};

}  // namespace esp32_bridge
