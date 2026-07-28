#include "bluerov_integration/integration/path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bluerov_integration::integration
{

PathFollower::PathFollower(rclcpp::Node & node, PathFollowerConfig config)
: config_(std::move(config)),
  logger_(node.get_logger()),
  clock_(node.get_clock())
{
  if (config_.lookahead_distance <= 0.0 ||
    config_.waypoint_reached_radius <= 0.0)
  {
    throw std::invalid_argument(
            "Path follower distances must be positive");
  }

  target_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.tracking_target_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  RCLCPP_INFO(
    logger_, "PathFollower %s: lookahead=%.2f m, reached=%.2f m",
    config_.enabled ? "enabled" : "bypass",
    config_.lookahead_distance,
    config_.waypoint_reached_radius);
}

bool PathFollower::finitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool PathFollower::framesCompatible(
  const std::string & first,
  const std::string & second)
{
  return first.empty() || second.empty() || first == second;
}

double PathFollower::distance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  return std::hypot(
    std::hypot(first.x - second.x, first.y - second.y),
    first.z - second.z);
}

common::ControlTarget PathFollower::invalidTarget(
  const std::string & reason,
  const common::SampleMetadata * source_metadata)
{
  if (config_.enabled && reason != last_failure_) {
    RCLCPP_WARN(logger_, "PathFollower idle: %s", reason.c_str());
  }
  last_failure_ = reason;
  have_selection_ = false;

  common::ControlTarget target;
  if (source_metadata != nullptr) {
    target.source_metadata = *source_metadata;
  }
  return target;
}

common::ControlTarget PathFollower::makeTarget(
  geometry_msgs::msg::PointStamped message,
  const common::SampleMetadata & source_metadata,
  const std::uint64_t mission_sequence,
  const std::uint64_t path_sequence,
  const std::size_t waypoint_index,
  const bool follows_path,
  const bool final_goal)
{
  message.header.stamp = clock_->now();
  const bool changed =
    !have_selection_ ||
    mission_sequence != last_mission_sequence_ ||
    path_sequence != last_path_sequence_ ||
    waypoint_index != last_waypoint_index_ ||
    follows_path != last_follows_path_ ||
    final_goal != last_final_goal_ ||
    distance(message.point, last_target_point_) > 1.0e-6;

  if (changed) {
    ++tracking_sequence_;
    target_pub_->publish(message);
    RCLCPP_INFO(
      logger_,
      "tracking target=(%.2f %.2f %.2f), waypoint=%zu, final=%d",
      message.point.x, message.point.y, message.point.z,
      waypoint_index, final_goal);
  }

  last_mission_sequence_ = mission_sequence;
  last_path_sequence_ = path_sequence;
  last_waypoint_index_ = waypoint_index;
  last_target_point_ = message.point;
  last_follows_path_ = follows_path;
  last_final_goal_ = final_goal;
  have_selection_ = true;
  last_failure_.clear();

  common::ControlTarget target;
  target.message = std::move(message);
  target.source_metadata = source_metadata;
  target.mission_sequence = mission_sequence;
  target.path_sequence = path_sequence;
  target.tracking_sequence = tracking_sequence_;
  target.waypoint_index = waypoint_index;
  target.valid = true;
  target.follows_path = follows_path;
  target.final_goal = final_goal;
  return target;
}

common::ControlTarget PathFollower::selectTarget(
  const common::StateSnapshot & snapshot)
{
  const auto & mission = snapshot.mission_goal;
  if (!mission.metadata.valid) {
    return invalidTarget("mission goal missing");
  }
  if (!finitePoint(mission.message.point)) {
    return invalidTarget("mission goal contains non-finite value", &mission.metadata);
  }

  if (!config_.enabled) {
    return makeTarget(
      mission.message,
      mission.metadata,
      mission.metadata.sequence,
      0,
      0,
      false,
      true);
  }

  if (!snapshot.bluerov_odometry.metadata.valid) {
    return invalidTarget("BlueROV odometry missing", &mission.metadata);
  }

  const auto & odometry = snapshot.bluerov_odometry.message;
  if (config_.require_matching_frame &&
    !framesCompatible(
      mission.message.header.frame_id, odometry.header.frame_id))
  {
    return invalidTarget(
      "mission goal and odometry frames differ", &mission.metadata);
  }

  const auto & path_sample = snapshot.reference_path;
  if (!path_sample.metadata.valid || path_sample.message == nullptr ||
    path_sample.message->poses.empty())
  {
    if (!config_.stop_without_valid_path) {
      return makeTarget(
        mission.message,
        mission.metadata,
        mission.metadata.sequence,
        0,
        0,
        false,
        true);
    }
    return invalidTarget("A* path missing or empty", &mission.metadata);
  }

  // 새 mission goal보다 먼저 만들어진 경로는 따라가지 않는다.
  if (path_sample.metadata.received_steady_time <
    mission.metadata.received_steady_time &&
    distance(
      path_sample.message->poses.back().pose.position,
      mission.message.point) > config_.waypoint_reached_radius)
  {
    return invalidTarget(
      "waiting for A* path for the new mission goal", &mission.metadata);
  }

  const auto & path = *path_sample.message;
  if (config_.require_matching_frame &&
    (!framesCompatible(path.header.frame_id, odometry.header.frame_id) ||
    !framesCompatible(path.header.frame_id, mission.message.header.frame_id)))
  {
    return invalidTarget("A* path frame differs", &mission.metadata);
  }
  if (!std::all_of(
      path.poses.begin(), path.poses.end(),
      [](const auto & pose) {
        return PathFollower::finitePoint(pose.pose.position);
      }))
  {
    return invalidTarget("A* path contains non-finite waypoint", &mission.metadata);
  }

  if (path_sample.metadata.sequence != last_path_sequence_ ||
    mission.metadata.sequence != last_mission_sequence_)
  {
    progress_index_ = 0;
  }

  const auto & current = odometry.pose.pose.position;
  const std::size_t first_index =
    std::min(progress_index_, path.poses.size() - 1U);
  std::size_t closest_index = first_index;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = first_index; index < path.poses.size(); ++index) {
    const double candidate_distance =
      distance(current, path.poses[index].pose.position);
    if (candidate_distance < closest_distance) {
      closest_distance = candidate_distance;
      closest_index = index;
    }
  }
  progress_index_ = closest_index;

  while (progress_index_ + 1U < path.poses.size() &&
    distance(current, path.poses[progress_index_].pose.position) <=
    config_.waypoint_reached_radius)
  {
    ++progress_index_;
  }

  std::size_t selected_index = progress_index_;
  double lookahead = 0.0;
  while (selected_index + 1U < path.poses.size() &&
    lookahead < config_.lookahead_distance)
  {
    lookahead += distance(
      path.poses[selected_index].pose.position,
      path.poses[selected_index + 1U].pose.position);
    ++selected_index;
  }

  geometry_msgs::msg::PointStamped target;
  target.header.frame_id = path.header.frame_id.empty() ?
    mission.message.header.frame_id : path.header.frame_id;
  const bool final_goal =
    selected_index + 1U >= path.poses.size() ||
    distance(current, mission.message.point) <=
    config_.waypoint_reached_radius;
  target.point = final_goal ?
    mission.message.point : path.poses[selected_index].pose.position;

  return makeTarget(
    std::move(target),
    mission.metadata,
    mission.metadata.sequence,
    path_sample.metadata.sequence,
    selected_index,
    true,
    final_goal);
}

}  // namespace bluerov_integration::integration
