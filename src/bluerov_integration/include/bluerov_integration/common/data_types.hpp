#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

#include <dave_interfaces/msg/dvl.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace bluerov_integration::common
{

struct SampleMetadata
{
  bool valid{false};
  std::uint64_t sequence{0};
  rclcpp::Time received_ros_time{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point received_steady_time{};
};

template<typename MessageT>
struct ReceivedSample
{
  MessageT message{};
  SampleMetadata metadata{};
};

// DataHub의 mutex 안에서 관리되는 전체 상태다.
// 각 소비자는 DataHub::snapshot()으로 값 복사본을 받아 사용한다.
struct StateSnapshot
{
  ReceivedSample<sensor_msgs::msg::FluidPressure> pressure;
  ReceivedSample<geometry_msgs::msg::PointStamped> depth;
  ReceivedSample<sensor_msgs::msg::Imu> imu;
  ReceivedSample<nav_msgs::msg::Odometry> bluerov_odometry;
  ReceivedSample<dave_interfaces::msg::DVL> dvl;
  ReceivedSample<nav_msgs::msg::Odometry> torpedo_odometry;
  ReceivedSample<geometry_msgs::msg::PointStamped> mission_goal;
  // Path는 클 수 있으므로 snapshot에서 본문 대신 불변 shared_ptr만 복사한다.
  ReceivedSample<nav_msgs::msg::Path::ConstSharedPtr> reference_path;
  std::uint64_t version{0};
};

// PathFollower가 만든 PPID 입력이다. mission_sequence는 PID 초기화 기준이고
// tracking_sequence는 waypoint 변경 감지에 사용한다.
struct ControlTarget
{
  geometry_msgs::msg::PointStamped message{};
  SampleMetadata source_metadata{};
  std::uint64_t mission_sequence{0};
  std::uint64_t path_sequence{0};
  std::uint64_t tracking_sequence{0};
  std::size_t waypoint_index{0};
  bool valid{false};
  bool follows_path{false};
  bool final_goal{false};
};

inline bool isFresh(
  const SampleMetadata & metadata,
  const std::chrono::steady_clock::time_point now,
  const double timeout_sec)
{
  if (!metadata.valid) {
    return false;
  }
  if (timeout_sec <= 0.0) {
    return true;
  }

  const double age_sec =
    std::chrono::duration<double>(now - metadata.received_steady_time).count();
  return age_sec >= 0.0 && age_sec <= timeout_sec;
}

}  // namespace bluerov_integration::common
