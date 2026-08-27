#include "esp32_bridge/uart_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace esp32_bridge
{
namespace
{

speed_t toTermiosBaudRate(const int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
      throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud_rate));
  }
}

}  // namespace

UartPort::UartPort(const std::string & device, const int baud_rate)
{
  file_descriptor_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (file_descriptor_ == -1) {
    throw std::system_error(errno, std::generic_category(), "failed to open " + device);
  }

  try {
    termios settings{};
    if (::tcgetattr(file_descriptor_, &settings) == -1) {
      throw std::system_error(errno, std::generic_category(), "UART tcgetattr failed");
    }

    ::cfmakeraw(&settings);
    const speed_t speed = toTermiosBaudRate(baud_rate);
    if (::cfsetispeed(&settings, speed) == -1 || ::cfsetospeed(&settings, speed) == -1) {
      throw std::system_error(errno, std::generic_category(), "UART baud-rate setup failed");
    }
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    settings.c_cflag &= static_cast<tcflag_t>(~PARENB);
    settings.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    settings.c_cflag |= CS8;

    if (::tcsetattr(file_descriptor_, TCSANOW, &settings) == -1) {
      throw std::system_error(errno, std::generic_category(), "UART tcsetattr failed");
    }
    ::tcflush(file_descriptor_, TCIOFLUSH);
  } catch (...) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
    throw;
  }
}

UartPort::~UartPort()
{
  if (file_descriptor_ >= 0) {
    ::close(file_descriptor_);
  }
}

void UartPort::writeByte(const std::uint8_t byte) const
{
  ssize_t result;
  do {
    result = ::write(file_descriptor_, &byte, 1);
  } while (result == -1 && errno == EINTR);

  if (result != 1) {
    throw std::system_error(errno, std::generic_category(), "UART write failed");
  }
}

}  // namespace esp32_bridge
