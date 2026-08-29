#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace esp32_bridge
{

class UartPort
{
public:
  UartPort(const std::string & device, int baud_rate);
  ~UartPort();

  UartPort(const UartPort &) = delete;
  UartPort & operator=(const UartPort &) = delete;

  void writeAll(const std::vector<std::uint8_t> & data) const;
  std::vector<std::uint8_t> readAvailable(std::size_t maximum_size = 4096) const;

private:
  int file_descriptor_{-1};
};

}  // namespace esp32_bridge
