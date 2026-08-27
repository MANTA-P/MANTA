#include <rclcpp/rclcpp.hpp>

#include <cctype>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>

#include "esp32_bridge/keyboard_input.hpp"
#include "esp32_bridge/uart_port.hpp"

namespace esp32_bridge
{

class Esp32BridgeNode final : public rclcpp::Node
{
public:
  Esp32BridgeNode()
  : Node("esp32_bridge_node"),
    device_(declare_parameter<std::string>("device", "/dev/ttyACM0")),
    baud_rate_(declare_parameter<int>("baud_rate", 115200)),
    uart_(device_, baud_rate_)
  {
    RCLCPP_INFO(
      get_logger(), "UART ready: device=%s, baud=%d, format=8N1",
      device_.c_str(), baud_rate_);
    RCLCPP_INFO(get_logger(), "Press a key to send one byte. Press Ctrl+C to exit.");
  }

  void send(const std::uint8_t byte)
  {
    uart_.writeByte(byte);
    if (std::isprint(static_cast<unsigned char>(byte)) != 0) {
      RCLCPP_INFO(get_logger(), "TX: '%c' (0x%02X)", byte, byte);
    } else {
      RCLCPP_INFO(get_logger(), "TX: 0x%02X", byte);
    }
  }

private:
  std::string device_;
  int baud_rate_;
  UartPort uart_;
};

}  // namespace esp32_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<esp32_bridge::Esp32BridgeNode>();
    esp32_bridge::KeyboardInput keyboard;

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      const auto byte = keyboard.readByte(100);
      if (byte.has_value()) {
        node->send(*byte);
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("esp32_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
