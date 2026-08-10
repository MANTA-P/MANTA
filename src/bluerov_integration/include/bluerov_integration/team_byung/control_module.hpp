#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "bluerov_integration/common/data_types.hpp"
#include "bluerov_integration/team_byung/ppid_controller.hpp"
#include "bluerov_integration/team_byung/ppid_logger.hpp"

namespace bluerov_integration::team_byung
{

struct ControlModuleConfig
{
  bool enabled{true};
  bool require_matching_target_frame{true};
  std::string velocity_source{"odometry"};
  double nominal_dt{0.05};
  double odometry_timeout_sec{0.5};
  double dvl_timeout_sec{0.5};
  double target_timeout_sec{0.0};
  std::array<std::string, 6> thruster_topics{
    "/model/bluerov2/joint/thruster1_joint/cmd_thrust",
    "/model/bluerov2/joint/thruster2_joint/cmd_thrust",
    "/model/bluerov2/joint/thruster3_joint/cmd_thrust",
    "/model/bluerov2/joint/thruster4_joint/cmd_thrust",
    "/model/bluerov2/joint/thruster5_joint/cmd_thrust",
    "/model/bluerov2/joint/thruster6_joint/cmd_thrust"};
  BlueRovControllerConfig controller{};
  PpidLoggerConfig logger{};
};

// team_byung 제어 기능의 ROS 연결부다.
// 센서를 직접 구독하지 않고 DataHub snapshot만 입력받는다.
class ControlModule
{
public:
  ControlModule(rclcpp::Node & node, ControlModuleConfig config);
  ~ControlModule();

  void update(
    const common::StateSnapshot & snapshot,
    const common::ControlTarget & tracking_target);
  void stop();
  std::uint64_t loggerOverflowCount() const;

private:
  bool targetFrameMatches(
    const geometry_msgs::msg::PointStamped & target,
    const nav_msgs::msg::Odometry & odometry) const;
  bool finiteTarget(const geometry_msgs::msg::PointStamped & target) const;
  void publish(const std::array<double, 6> & commands);
  void publishZero();
  void deactivate(const char * reason);

  ControlModuleConfig config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  BlueRovPPIDController controller_;
  std::unique_ptr<PpidLogger> ppid_logger_;
  std::array<
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 6> thruster_pubs_;

  std::uint64_t mission_sequence_{0};
  std::uint64_t tracking_sequence_{0};
  bool have_target_{false};
  bool active_{false};
  bool stopped_{false};
  bool have_previous_tick_{false};
  std::chrono::steady_clock::time_point previous_tick_;
};

}  // namespace bluerov_integration::team_byung
