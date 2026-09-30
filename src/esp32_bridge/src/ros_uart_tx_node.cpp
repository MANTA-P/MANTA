#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "esp32_bridge/packet_codec.hpp"
#include "esp32_bridge/uart_port.hpp"

namespace esp32_bridge
{
namespace
{

constexpr double kPositionUnit = 0.001;  // m/LSB
constexpr double kMotionUnit = 0.001;  // (m/s, rad/s)/LSB
constexpr double kQuaternionUnit = 1.0 / 16384.0;
constexpr std::uint64_t kActuatorTimeoutMs = 200;
constexpr std::uint64_t kStatusTimeoutMs = 500;

std::uint64_t steadyNowMs()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool appendPosition(std::vector<std::uint8_t> & data, double x, double y, double z)
{
  return appendFixedInt32(data, x, kPositionUnit) &&
         appendFixedInt32(data, y, kPositionUnit) &&
         appendFixedInt32(data, z, kPositionUnit);
}

bool appendMotion(std::vector<std::uint8_t> & data, double x, double y, double z)
{
  return appendFixedInt16(data, x, kMotionUnit) &&
         appendFixedInt16(data, y, kMotionUnit) &&
         appendFixedInt16(data, z, kMotionUnit);
}

bool appendQuaternion(
  std::vector<std::uint8_t> & data, double x, double y, double z, double w)
{
  return appendFixedInt16(data, x, kQuaternionUnit) &&
         appendFixedInt16(data, y, kQuaternionUnit) &&
         appendFixedInt16(data, z, kQuaternionUnit) &&
         appendFixedInt16(data, w, kQuaternionUnit);
}

}  // namespace

class RosUartTxNode final : public rclcpp::Node
{
public:
  RosUartTxNode()
  : Node("esp32_torpedo_hil_bridge_node"),
    device_(declare_parameter<std::string>("device", "/dev/ttyACM0")),
    baud_rate_(declare_parameter<int>("baud_rate", 921600)),
    uart_(device_, baud_rate_)
  {
    setupRosInterfaces();
    declare_parameter<bool>("control.armed", false);
    declare_parameter<int>("control.mode", 0);
    declare_parameter<int>("control.target_thrust", 0);

    command_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RosUartTxNode::sendControlCommand, this));
    receive_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&RosUartTxNode::receive, this));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&RosUartTxNode::runWatchdogs, this));
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&RosUartTxNode::printStatistics, this));

    const auto now_ms = steadyNowMs();
    startup_ms_ = now_ms;
    last_actuator_ms_ = now_ms;
    last_status_ms_ = now_ms;
    RCLCPP_INFO(
      get_logger(),
      "Torpedo HIL UART bridge ready: device=%s baud=%d frame=AA55|ID|LEN|DATA",
      device_.c_str(), baud_rate_);
  }

private:
  void setupRosInterfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    bluerov_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>(
        "topics.bluerov_odometry", "/model/bluerov2/odometry"),
      sensor_qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        sendOdometry(MessageId::kBlueRovOdometry, *message);
      });
    torpedo_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>(
        "topics.torpedo_odometry", "/torpedo/state/odometry"),
      sensor_qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        sendOdometry(MessageId::kTorpedoOdometry, *message);
      });

    const std::array<std::string, 5> defaults{
      "/torpedo/actuators/thruster/command",
      "/torpedo/actuators/fins/top/command",
      "/torpedo/actuators/fins/bottom/command",
      "/torpedo/actuators/fins/left/command",
      "/torpedo/actuators/fins/right/command"};
    const std::array<std::string, 5> parameter_names{
      "topics.thrust", "topics.fin_top", "topics.fin_bottom",
      "topics.fin_left", "topics.fin_right"};
    for (std::size_t index = 0; index < actuator_publishers_.size(); ++index) {
      actuator_publishers_[index] = create_publisher<std_msgs::msg::Float64>(
        declare_parameter<std::string>(parameter_names[index], defaults[index]), 10);
    }
  }

  void sendOdometry(
    const MessageId message_id, const nav_msgs::msg::Odometry & message)
  {
    std::vector<std::uint8_t> payload;
    payload.reserve(32);
    if (appendPosition(
        payload, message.pose.pose.position.x, message.pose.pose.position.y,
        message.pose.pose.position.z) &&
      appendQuaternion(
        payload, message.pose.pose.orientation.x, message.pose.pose.orientation.y,
        message.pose.pose.orientation.z, message.pose.pose.orientation.w) &&
      appendMotion(
        payload, message.twist.twist.linear.x, message.twist.twist.linear.y,
        message.twist.twist.linear.z) &&
      appendMotion(
        payload, message.twist.twist.angular.x, message.twist.twist.angular.y,
        message.twist.twist.angular.z))
    {
      send(message_id, payload);
    } else {
      ++invalid_tx_values_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "UART odometry 0x%02X dropped: non-finite value",
        static_cast<unsigned int>(message_id));
    }
  }

  void sendControlCommand()
  {
    bool armed = get_parameter("control.armed").as_bool();
    const auto requested_mode = get_parameter("control.mode").as_int();
    const auto requested_thrust = get_parameter("control.target_thrust").as_int();

    std::int64_t mode = requested_mode;
    if (mode < 0 || mode > 2) {
      armed = false;
      mode = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Invalid control.mode=%ld; sending DISARMED/None",
        static_cast<long>(requested_mode));
    }
    const auto thrust = std::clamp<std::int64_t>(requested_thrust, 0, 1000);
    if (thrust != requested_thrust) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "control.target_thrust=%ld clamped to %ld",
        static_cast<long>(requested_thrust), static_cast<long>(thrust));
    }

    std::vector<std::uint8_t> payload;
    payload.reserve(8);
    payload.push_back(1U);
    payload.push_back(command_sequence_++);
    payload.push_back(armed ? 1U : 0U);
    payload.push_back(static_cast<std::uint8_t>(mode));
    appendFixedUint16(payload, static_cast<double>(thrust), 1.0);
    payload.push_back(0U);
    payload.push_back(0U);
    send(MessageId::kControlCommand, payload);
  }

  void send(const MessageId message_id, const std::vector<std::uint8_t> & payload)
  {
    const auto frame = encodeFrame(message_id, payload);
    uart_.writeAll(frame);
    ++tx_frames_;
    tx_bytes_ += frame.size();
  }

  void receive()
  {
    const auto now_ms = steadyNowMs();
    parser_.checkTimeout(now_ms);
    const auto bytes = uart_.readAvailable();
    if (bytes.empty()) {
      return;
    }
    rx_bytes_ += bytes.size();
    for (const auto & frame : parser_.feed(bytes.data(), bytes.size(), now_ms)) {
      ++rx_frames_;
      handleFrame(frame, now_ms);
    }
  }

  void handleFrame(const UartFrame & frame, const std::uint64_t now_ms)
  {
    if (frame.message_id == MessageId::kTorpedoActuator) {
      TorpedoActuator actuator;
      if (!decodeTorpedoActuator(frame.payload, actuator)) {
        ++invalid_rx_frames_;
        return;
      }
      last_actuator_ms_ = now_ms;
      actuator_received_ = true;
      actuator_watchdog_active_ = false;
      if (actuator.state == 1U) {
        publishActuator(
          actuator.thrust, actuator.fin_top, actuator.fin_bottom,
          actuator.fin_left, actuator.fin_right);
      } else {
        publishZeroActuator();
      }
      return;
    }

    if (frame.message_id == MessageId::kControllerStatus) {
      ControllerStatus status;
      if (!decodeControllerStatus(frame.payload, status)) {
        ++invalid_rx_frames_;
        return;
      }
      last_status_ms_ = now_ms;
      status_received_ = true;
      if (!have_last_status_ || status.state != last_status_.state ||
        status.mode != last_status_.mode || status.flags != last_status_.flags ||
        status.last_error != last_status_.last_error)
      {
        RCLCPP_INFO(
          get_logger(), "Controller status: state=%u mode=%u flags=0x%02X error=%u uptime=%u",
          status.state, status.mode, status.flags, status.last_error,
          status.uptime_seconds_low16);
      }
      last_status_ = status;
      have_last_status_ = true;
      return;
    }

    ++unexpected_rx_frames_;
  }

  void publishActuator(
    const double thrust, const double top, const double bottom,
    const double left, const double right)
  {
    const std::array<double, 5> values{thrust, top, bottom, left, right};
    for (std::size_t index = 0; index < values.size(); ++index) {
      std_msgs::msg::Float64 message;
      message.data = values[index];
      actuator_publishers_[index]->publish(message);
    }
  }

  void publishZeroActuator()
  {
    publishActuator(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  void runWatchdogs()
  {
    const auto now_ms = steadyNowMs();
    const auto actuator_reference_ms = actuator_received_ ? last_actuator_ms_ : startup_ms_;
    if (now_ms - actuator_reference_ms >= kActuatorTimeoutMs && !actuator_watchdog_active_) {
      actuator_watchdog_active_ = true;
      publishZeroActuator();
      RCLCPP_ERROR(get_logger(), "Actuator UART timeout: publishing safe zero output");
    }

    const auto status_reference_ms = status_received_ ? last_status_ms_ : startup_ms_;
    if (now_ms - status_reference_ms >= kStatusTimeoutMs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Controller status heartbeat timeout");
    }
  }

  void printStatistics()
  {
    RCLCPP_INFO(
      get_logger(),
      "UART tx=%llu/%lluB rx=%llu/%lluB frame_error=%llu timeout=%llu invalid=%llu unexpected=%llu",
      static_cast<unsigned long long>(tx_frames_),
      static_cast<unsigned long long>(tx_bytes_),
      static_cast<unsigned long long>(rx_frames_),
      static_cast<unsigned long long>(rx_bytes_),
      static_cast<unsigned long long>(parser_.framingErrorCount()),
      static_cast<unsigned long long>(parser_.timeoutCount()),
      static_cast<unsigned long long>(invalid_rx_frames_ + invalid_tx_values_),
      static_cast<unsigned long long>(unexpected_rx_frames_));
  }

  std::string device_;
  int baud_rate_;
  UartPort uart_;
  UartFrameParser parser_;
  std::uint8_t command_sequence_{0};
  std::uint64_t startup_ms_{0};
  std::uint64_t last_actuator_ms_{0};
  std::uint64_t last_status_ms_{0};
  bool actuator_received_{false};
  bool status_received_{false};
  bool actuator_watchdog_active_{false};
  bool have_last_status_{false};
  ControllerStatus last_status_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bluerov_odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr torpedo_odometry_sub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 5> actuator_publishers_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::uint64_t tx_frames_{0};
  std::uint64_t tx_bytes_{0};
  std::uint64_t rx_frames_{0};
  std::uint64_t rx_bytes_{0};
  std::uint64_t invalid_tx_values_{0};
  std::uint64_t invalid_rx_frames_{0};
  std::uint64_t unexpected_rx_frames_{0};
};

}  // namespace esp32_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<esp32_bridge::RosUartTxNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("esp32_torpedo_hil_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
