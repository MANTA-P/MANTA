#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/common/data_types.hpp"

namespace bluerov_integration::integration
{

struct PathFollowerConfig
{
  bool enabled{true};
  bool stop_without_valid_path{true};
  bool require_matching_frame{true};
  double lookahead_distance{2.0};
  double waypoint_reached_radius{0.75};
  std::string tracking_target_topic{"/ppid/tracking_target"};
};

// team_min 경로와 현재 위치를 team_byung PPID가 사용할 한 점으로 바꾼다.
class PathFollower
{
public:
  PathFollower(rclcpp::Node & node, PathFollowerConfig config);

  common::ControlTarget selectTarget(const common::StateSnapshot & snapshot);

private:
  static bool finitePoint(const geometry_msgs::msg::Point & point);
  static bool framesCompatible(
    const std::string & first,
    const std::string & second);
  static double distance(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second);

  common::ControlTarget invalidTarget(
    const std::string & reason,
    const common::SampleMetadata * source_metadata = nullptr);
  common::ControlTarget makeTarget(
    geometry_msgs::msg::PointStamped message,
    const common::SampleMetadata & source_metadata,
    std::uint64_t mission_sequence,
    std::uint64_t path_sequence,
    std::size_t waypoint_index,
    bool follows_path,
    bool final_goal);

  PathFollowerConfig config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;

  std::uint64_t last_mission_sequence_{0};
  std::uint64_t last_path_sequence_{0};
  std::uint64_t tracking_sequence_{0};
  std::size_t progress_index_{0};
  std::size_t last_waypoint_index_{0};
  geometry_msgs::msg::Point last_target_point_{};
  bool last_follows_path_{false};
  bool last_final_goal_{false};
  bool have_selection_{false};
  std::string last_failure_;
};

}  // namespace bluerov_integration::integration
