#include <dave_interfaces/msg/dvl.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "esp32_bridge/packet_codec.hpp"
#include "esp32_bridge/uart_port.hpp"

namespace esp32_bridge
{
namespace
{
void appendVector3(std::vector<std::uint8_t> & data, double x, double y, double z)
{
  appendFloat32BigEndian(data, static_cast<float>(x));
  appendFloat32BigEndian(data, static_cast<float>(y));
  appendFloat32BigEndian(data, static_cast<float>(z));
}
}  // namespace

class HilBridgeNode final : public rclcpp::Node
{
public:
  HilBridgeNode()
  : Node("esp32_hil_bridge_node"),
    device_(declare_parameter<std::string>("device", "/dev/ttyACM0")),
    baud_rate_(declare_parameter<int>("baud_rate", 115200)),
    uart_(device_, baud_rate_)
  {
    setupTopics();
    const double rx_hz = declare_parameter<double>("receive_rate_hz", 500.0);
    if (rx_hz <= 0.0) {
      throw std::invalid_argument("receive_rate_hz must be positive");
    }
    rx_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rx_hz),
      std::bind(&HilBridgeNode::receive, this));
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&HilBridgeNode::printStatus, this));
    RCLCPP_INFO(
      get_logger(), "HIL bridge ready: %s, protocol v1 big-endian",
      device_.c_str());
  }

private:
  void setupTopics()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto odometry_topic = declare_parameter<std::string>(
      "topics.bluerov_odometry", "/model/bluerov2/odometry");
    const auto imu_topic = declare_parameter<std::string>(
      "topics.imu", "/model/bluerov2/imu");
    const auto pressure_topic = declare_parameter<std::string>(
      "topics.pressure", "/model/bluerov2/pressure");
    const auto depth_topic = declare_parameter<std::string>(
      "topics.depth", "/model/bluerov2/Pressure_depth");
    const auto dvl_topic = declare_parameter<std::string>(
      "topics.dvl", "/dvl/velocity");
    const auto torpedo_topic = declare_parameter<std::string>(
      "topics.torpedo_odometry", "/torpedo/state/odometry");
    const auto goal_topic = declare_parameter<std::string>(
      "topics.mission_goal", "/mission/target_position");

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::vector<std::uint8_t> position;
        appendVector3(position, msg->pose.pose.position.x,
          msg->pose.pose.position.y, msg->pose.pose.position.z);
        send(MessageType::kBlueRovPosition, std::move(position));

        std::vector<std::uint8_t> velocity;
        appendVector3(velocity, msg->twist.twist.linear.x,
          msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        appendVector3(velocity, msg->twist.twist.angular.x,
          msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        send(MessageType::kBlueRovVelocity, std::move(velocity));
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        std::vector<std::uint8_t> data;
        appendFloat32BigEndian(data, static_cast<float>(msg->orientation.x));
        appendFloat32BigEndian(data, static_cast<float>(msg->orientation.y));
        appendFloat32BigEndian(data, static_cast<float>(msg->orientation.z));
        appendFloat32BigEndian(data, static_cast<float>(msg->orientation.w));
        appendVector3(data, msg->angular_velocity.x,
          msg->angular_velocity.y, msg->angular_velocity.z);
        appendVector3(data, msg->linear_acceleration.x,
          msg->linear_acceleration.y, msg->linear_acceleration.z);
        send(MessageType::kBlueRovAttitude, std::move(data));
      });
    pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      pressure_topic, sensor_qos,
      [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
        sendScalars(MessageType::kPressure, {msg->fluid_pressure});
      });
    depth_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      depth_topic, sensor_qos,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        std::vector<std::uint8_t> data;
        appendVector3(data, msg->point.x, msg->point.y, msg->point.z);
        send(MessageType::kDepth, std::move(data));
      });
    dvl_sub_ = create_subscription<dave_interfaces::msg::DVL>(
      dvl_topic, sensor_qos, [this](dave_interfaces::msg::DVL::ConstSharedPtr msg) {
        std::vector<std::uint8_t> data;
        appendVector3(data, msg->velocity.twist.linear.x,
          msg->velocity.twist.linear.y, msg->velocity.twist.linear.z);
        send(MessageType::kDvlVelocity, std::move(data));
      });
    torpedo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      torpedo_topic, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::vector<std::uint8_t> data;
        appendVector3(data, msg->pose.pose.position.x,
          msg->pose.pose.position.y, msg->pose.pose.position.z);
        appendVector3(data, msg->twist.twist.linear.x,
          msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        send(MessageType::kTorpedoState, std::move(data));
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        std::vector<std::uint8_t> data;
        appendVector3(data, msg->point.x, msg->point.y, msg->point.z);
        send(MessageType::kMissionGoal, std::move(data));
      });

    auto defaults = std::vector<std::string>{
      "/model/bluerov2/joint/thruster1_joint/cmd_thrust",
      "/model/bluerov2/joint/thruster2_joint/cmd_thrust",
      "/model/bluerov2/joint/thruster3_joint/cmd_thrust",
      "/model/bluerov2/joint/thruster4_joint/cmd_thrust",
      "/model/bluerov2/joint/thruster5_joint/cmd_thrust",
      "/model/bluerov2/joint/thruster6_joint/cmd_thrust"};
    const auto topics = declare_parameter<std::vector<std::string>>(
      "topics.thrusters", defaults);
    if (topics.size() != thruster_pubs_.size()) {
      throw std::invalid_argument("topics.thrusters must have 6 entries");
    }
    for (std::size_t i = 0; i < topics.size(); ++i) {
      thruster_pubs_[i] = create_publisher<std_msgs::msg::Float64>(topics[i], 10);
    }
  }

  void sendScalars(MessageType type, std::initializer_list<double> values)
  {
    std::vector<std::uint8_t> data;
    for (const double value : values) {
      appendFloat32BigEndian(data, static_cast<float>(value));
    }
    send(type, std::move(data));
  }

  void send(MessageType type, std::vector<std::uint8_t> payload)
  {
    Packet packet;
    packet.type = type;
    packet.sequence = sequence_[static_cast<std::uint8_t>(type)]++;
    packet.timestamp_us = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start_time_).count());
    packet.payload = std::move(payload);
    const auto bytes = encodePacket(packet);
    uart_.writeAll(bytes);
    ++tx_packets_;
    tx_bytes_ += bytes.size();
  }

  void receive()
  {
    const auto bytes = uart_.readAvailable();
    if (bytes.empty()) {
      return;
    }
    rx_bytes_ += bytes.size();
    for (const auto & packet : parser_.feed(bytes.data(), bytes.size())) {
      ++rx_packets_;
      handle(packet);
    }
  }

  void handle(const Packet & packet)
  {
    if (packet.type != MessageType::kThrusterOutput) {
      return;
    }
    if (packet.payload.size() != 6 * sizeof(float)) {
      ++invalid_packets_;
      return;
    }
    std::array<float, 6> commands{};
    for (std::size_t i = 0; i < commands.size(); ++i) {
      if (!readFloat32BigEndian(packet.payload, i * sizeof(float), commands[i]) ||
        !std::isfinite(commands[i]))
      {
        ++invalid_packets_;
        return;
      }
    }
    for (std::size_t i = 0; i < commands.size(); ++i) {
      std_msgs::msg::Float64 msg;
      msg.data = commands[i];
      thruster_pubs_[i]->publish(msg);
    }
  }

  void printStatus()
  {
    RCLCPP_INFO(get_logger(),
      "USB tx=%llu/%lluB rx=%llu/%lluB crc=%llu frame=%llu invalid=%llu",
      static_cast<unsigned long long>(tx_packets_),
      static_cast<unsigned long long>(tx_bytes_),
      static_cast<unsigned long long>(rx_packets_),
      static_cast<unsigned long long>(rx_bytes_),
      static_cast<unsigned long long>(parser_.crcErrorCount()),
      static_cast<unsigned long long>(parser_.framingErrorCount()),
      static_cast<unsigned long long>(invalid_packets_));
  }

  std::string device_;
  int baud_rate_;
  UartPort uart_;
  PacketParser parser_;
  const std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
  std::array<std::uint16_t, 256> sequence_{};
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 6> thruster_pubs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr depth_sub_;
  rclcpp::Subscription<dave_interfaces::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr torpedo_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::uint64_t tx_packets_{0};
  std::uint64_t tx_bytes_{0};
  std::uint64_t rx_packets_{0};
  std::uint64_t rx_bytes_{0};
  std::uint64_t invalid_packets_{0};
};
}  // namespace esp32_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<esp32_bridge::HilBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("esp32_hil_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
