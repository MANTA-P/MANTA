#include "esp32_bridge/keyboard_input.hpp"

#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace esp32_bridge
{

KeyboardInput::KeyboardInput()
{
  if (::isatty(STDIN_FILENO) == 0) {
    throw std::runtime_error("standard input is not a terminal");
  }
  if (::tcgetattr(STDIN_FILENO, &original_settings_) == -1) {
    throw std::system_error(errno, std::generic_category(), "tcgetattr failed");
  }

  termios settings = original_settings_;
  settings.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;

  if (::tcsetattr(STDIN_FILENO, TCSANOW, &settings) == -1) {
    throw std::system_error(errno, std::generic_category(), "tcsetattr failed");
  }
  restore_required_ = true;
}

KeyboardInput::~KeyboardInput()
{
  if (restore_required_) {
    ::tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_);
  }
}

std::optional<std::uint8_t> KeyboardInput::readByte(const int timeout_ms) const
{
  pollfd descriptor{};
  descriptor.fd = STDIN_FILENO;
  descriptor.events = POLLIN;

  const int result = ::poll(&descriptor, 1, timeout_ms);
  if (result == -1) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    throw std::system_error(errno, std::generic_category(), "keyboard poll failed");
  }
  if (result == 0 || (descriptor.revents & POLLIN) == 0) {
    return std::nullopt;
  }

  std::uint8_t byte{};
  const ssize_t bytes_read = ::read(STDIN_FILENO, &byte, 1);
  if (bytes_read == 1) {
    return byte;
  }
  if (bytes_read == -1 && errno != EINTR && errno != EAGAIN) {
    throw std::system_error(errno, std::generic_category(), "keyboard read failed");
  }
  return std::nullopt;
}

}  // namespace esp32_bridge
