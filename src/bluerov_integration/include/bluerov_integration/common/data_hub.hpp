#pragma once

#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/common/data_types.hpp"

namespace bluerov_integration::common
{

struct DataHubConfig
{
  std::string pressure_topic{"/model/bluerov2/pressure"};
  std::string depth_topic{"/model/bluerov2/Pressure_depth"};
  std::string imu_topic{"/model/bluerov2/imu"};
  std::string bluerov_odometry_topic{"/model/bluerov2/odometry"};
  std::string dvl_topic{"/dvl/velocity"};
  std::string torpedo_odometry_topic{"/torpedo/state/odometry"};
  std::string mission_goal_topic{"/mission/target_position"};
  std::string reference_path_topic{"/uuv/reference_path"};
};

// ROS 입력의 단일 진입점이다.
// 콜백에서는 메시지를 값으로 복사한 뒤 짧게 잠그고 최신 상태만 교체한다.
class DataHub
{
public:
  DataHub(rclcpp::Node & node, DataHubConfig config);

  StateSnapshot snapshot() const;

private:
  void pressureCallback(sensor_msgs::msg::FluidPressure::ConstSharedPtr message);
  void depthCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr message);
  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void bluerovOdometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void dvlCallback(dave_interfaces::msg::DVL::ConstSharedPtr message);
  void torpedoOdometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void missionGoalCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr message);
  void referencePathCallback(nav_msgs::msg::Path::ConstSharedPtr message);

  template<typename MessageT>
  void store(ReceivedSample<MessageT> & destination, MessageT message)
  {
    const auto received_ros_time = node_.now();
    const auto received_steady_time = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex_);
    destination.message = std::move(message);
    destination.metadata.valid = true;
    destination.metadata.sequence = ++state_.version;
    destination.metadata.received_ros_time = received_ros_time;
    destination.metadata.received_steady_time = received_steady_time;
  }

  rclcpp::Node & node_;
  DataHubConfig config_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bluerov_odometry_sub_;
  rclcpp::Subscription<dave_interfaces::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr torpedo_odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr mission_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_path_sub_;

  mutable std::mutex mutex_;
  StateSnapshot state_;
};

}  // namespace bluerov_integration::common
