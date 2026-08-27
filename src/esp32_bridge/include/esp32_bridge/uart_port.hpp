#pragma once

#include <cstdint>
#include <string>

namespace esp32_bridge
{

class UartPort
{
public:
  UartPort(const std::string & device, int baud_rate);
  ~UartPort();

  UartPort(const UartPort &) = delete;
  UartPort & operator=(const UartPort &) = delete;

  void writeByte(std::uint8_t byte) const;

private:
  int file_descriptor_{-1};
};

}  // namespace esp32_bridge
