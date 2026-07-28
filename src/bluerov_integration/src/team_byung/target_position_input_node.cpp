#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace bluerov_integration::team_byung
{

class TargetPositionInputNode final : public rclcpp::Node
{
public:
  TargetPositionInputNode()
  : Node("target_position_input_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    topic_ = declare_parameter<std::string>(
      "topic", "/mission/target_position");
    publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
      topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  void run()
  {
    std::cout << "Enter BlueROV mission goal: x y z\n"
              << "Enter q to quit.\n";
    std::string line;
    while (rclcpp::ok()) {
      std::cout << "target> " << std::flush;
      if (!std::getline(std::cin, line) || line == "q" || line == "Q") {
        break;
      }

      std::istringstream input(line);
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      char extra = '\0';
      if (!(input >> x >> y >> z) || (input >> extra) ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        std::cout << "Please enter exactly three finite numbers: x y z\n";
        continue;
      }

      geometry_msgs::msg::PointStamped message;
      message.header.stamp = now();
      message.header.frame_id = frame_id_;
      message.point.x = x;
      message.point.y = y;
      message.point.z = z;
      publisher_->publish(message);
      rclcpp::spin_some(shared_from_this());
      RCLCPP_INFO(
        get_logger(), "Published mission goal=(%.3f, %.3f, %.3f), frame='%s'",
        x, y, z, frame_id_.c_str());
    }
  }

private:
  std::string frame_id_;
  std::string topic_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};

}  // namespace bluerov_integration::team_byung

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<bluerov_integration::team_byung::TargetPositionInputNode>();
  node->run();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
