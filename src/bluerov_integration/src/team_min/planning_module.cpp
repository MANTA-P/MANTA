#include "bluerov_integration/team_min/planning_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bluerov_integration::team_min
{

PlanningModule::PlanningModule(rclcpp::Node & node, PlanningConfig config)
: config_(std::move(config)),
  logger_(node.get_logger()),
  clock_(node.get_clock())
{
  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  path_pub_ = node.create_publisher<nav_msgs::msg::Path>(
    config_.path_topic, latched_qos);
  current_point_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.current_point_topic, latched_qos);
  goal_point_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.goal_point_topic, latched_qos);
  torpedo_point_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.torpedo_point_topic, latched_qos);
  visualizer_ = std::make_unique<RvizVisualizer>(node, config_.marker_topic);

  if (!config_.enabled) {
    RCLCPP_INFO(
      logger_,
      "team_min A* disabled; BlueROV and torpedo RViz tracking remains active");
    return;
  }
  if (!config_.use_target_topic_for_goal &&
    config_.goal_offset_x == 0.0 && config_.goal_offset_y == 0.0 &&
    config_.goal_offset_z == 0.0)
  {
    throw std::invalid_argument("Planning goal offset cannot be all zero");
  }
  if (config_.robot_replan_distance < 0.0 ||
    config_.torpedo_replan_distance < 0.0 ||
    config_.goal_replan_distance < 0.0)
  {
    throw std::invalid_argument("Planning replan distances must be non-negative");
  }

  running_.store(true);
  worker_ = std::thread(&PlanningModule::workerLoop, this);
  RCLCPP_INFO(logger_, "team_min A* worker started");
}

PlanningModule::~PlanningModule()
{
  stop();
}

void PlanningModule::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  request_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

Point3D PlanningModule::positionOf(const nav_msgs::msg::Odometry & odometry)
{
  return {
    odometry.pose.pose.position.x,
    odometry.pose.pose.position.y,
    odometry.pose.pose.position.z};
}

bool PlanningModule::framesCompatible(
  const std::string & first,
  const std::string & second)
{
  return first.empty() || second.empty() || first == second;
}

bool PlanningModule::needsReplan(const PlanningRequest & request) const
{
  if (!last_requested_) {
    return true;
  }
  return distance3D(request.start, last_requested_->start) >=
           config_.robot_replan_distance ||
         distance3D(request.torpedo, last_requested_->torpedo) >=
           config_.torpedo_replan_distance ||
         distance3D(request.goal, last_requested_->goal) >=
           config_.goal_replan_distance ||
         request.frame_id != last_requested_->frame_id;
}

void PlanningModule::update(const common::StateSnapshot & snapshot)
{
  if (snapshot.bluerov_odometry.metadata.valid) {
    visualizer_->publishBlueRov(snapshot.bluerov_odometry.message);
  }
  if (snapshot.torpedo_odometry.metadata.valid) {
    BoxObstacle live_barrier = config_.torpedo_barrier;
    live_barrier.center = positionOf(snapshot.torpedo_odometry.message);
    visualizer_->publishTorpedo(
      snapshot.torpedo_odometry.message, live_barrier, config_.enabled);
  }
  visualizer_->publishTelemetry(snapshot);

  if (!config_.enabled ||
    !snapshot.bluerov_odometry.metadata.valid ||
    !snapshot.torpedo_odometry.metadata.valid)
  {
    return;
  }

  const auto & bluerov = snapshot.bluerov_odometry.message;
  const auto & torpedo = snapshot.torpedo_odometry.message;
  const std::string frame_id =
    bluerov.header.frame_id.empty() ? "map" : bluerov.header.frame_id;
  if (!framesCompatible(frame_id, torpedo.header.frame_id)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "team_min frame mismatch: BlueROV='%s', torpedo='%s'",
      frame_id.c_str(), torpedo.header.frame_id.c_str());
    return;
  }

  const Point3D start = positionOf(bluerov);
  const Point3D torpedo_position = positionOf(torpedo);
  Point3D goal;
  if (config_.use_target_topic_for_goal) {
    if (!snapshot.mission_goal.metadata.valid) {
      return;
    }
    const auto & target = snapshot.mission_goal.message;
    if (!framesCompatible(frame_id, target.header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min target frame mismatch: planning='%s', target='%s'",
        frame_id.c_str(), target.header.frame_id.c_str());
      return;
    }
    goal = {target.point.x, target.point.y, target.point.z};
  } else {
    if (!fixed_goal_) {
      fixed_goal_ = Point3D{
        start.x + config_.goal_offset_x,
        start.y + config_.goal_offset_y,
        start.z + config_.goal_offset_z};
    }
    goal = *fixed_goal_;
  }

  const PlanningRequest request{start, goal, torpedo_position, frame_id};
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (!needsReplan(request)) {
      return;
    }
    pending_request_ = request;
    last_requested_ = request;
  }
  request_cv_.notify_one();
}

GridMapConfig PlanningModule::planningMap(
  const PlanningRequest & request) const
{
  if (!config_.use_dynamic_map) {
    return config_.fixed_map;
  }

  GridMapConfig map = config_.fixed_map;
  map.min_x = std::min(request.start.x, request.goal.x) - config_.map_padding_x;
  map.max_x = std::max(request.start.x, request.goal.x) + config_.map_padding_x;
  map.min_y = std::min(request.start.y, request.goal.y) - config_.map_padding_y;
  map.max_y = std::max(request.start.y, request.goal.y) + config_.map_padding_y;
  map.min_z = std::min(request.start.z, request.goal.z) - config_.map_padding_z;
  map.max_z = std::max(request.start.z, request.goal.z) + config_.map_padding_z;
  return map;
}

void PlanningModule::workerLoop()
{
  while (running_.load()) {
    std::optional<PlanningRequest> request;
    {
      std::unique_lock<std::mutex> lock(request_mutex_);
      request_cv_.wait(lock, [this]() {
        return !running_.load() || pending_request_.has_value();
      });
      if (!running_.load() && !pending_request_) {
        break;
      }
      request = std::move(pending_request_);
      pending_request_.reset();
    }
    if (request) {
      execute(*request);
    }
  }
}

void PlanningModule::execute(const PlanningRequest & request)
{
  const auto calculation_start = std::chrono::steady_clock::now();
  try {
    BoxObstacle barrier = config_.torpedo_barrier;
    barrier.center = request.torpedo;
    const auto path = runEnhancedAStar3D(
      request.start,
      request.goal,
      planningMap(request),
      std::vector<BoxObstacle>{barrier},
      config_.astar);
    if (path.empty()) {
      RCLCPP_WARN(logger_, "team_min A* found no path");
      return;
    }

    nav_msgs::msg::Path path_message;
    path_message.header.stamp = clock_->now();
    path_message.header.frame_id = request.frame_id;
    path_message.poses.reserve(path.size());
    for (const auto & waypoint : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_message.header;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.position.z = waypoint.z;
      pose.pose.orientation.w = 1.0;
      path_message.poses.push_back(std::move(pose));
    }
    path_pub_->publish(path_message);
    publishPoint(current_point_pub_, request.start, request.frame_id);
    publishPoint(goal_point_pub_, request.goal, request.frame_id);
    publishPoint(torpedo_point_pub_, request.torpedo, request.frame_id);
    visualizer_->publishScene(
      request.frame_id, request.start, request.goal, barrier, path);

    const double calculation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - calculation_start).count();
    RCLCPP_INFO(
      logger_, "team_min A* time=%.3f ms, waypoints=%zu",
      calculation_ms, path.size());
  } catch (const std::exception & error) {
    const double calculation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - calculation_start).count();
    RCLCPP_ERROR(
      logger_, "team_min A* failed after %.3f ms: %s",
      calculation_ms, error.what());
  }
}

void PlanningModule::publishPoint(
  const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
  const Point3D & point,
  const std::string & frame_id)
{
  geometry_msgs::msg::PointStamped message;
  message.header.stamp = clock_->now();
  message.header.frame_id = frame_id;
  message.point.x = point.x;
  message.point.y = point.y;
  message.point.z = point.z;
  publisher->publish(message);
}

}  // namespace bluerov_integration::team_min
