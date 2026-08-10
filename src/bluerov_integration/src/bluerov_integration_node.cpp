#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "bluerov_integration/common/data_hub.hpp"
#include "bluerov_integration/integration/path_follower.hpp"
#include "bluerov_integration/team_byung/control_module.hpp"
#include "bluerov_integration/team_min/planning_module.hpp"

namespace bluerov_integration
{
namespace
{

template<typename T, std::size_t Size>
std::vector<T> toVector(const std::array<T, Size> & values)
{
  return std::vector<T>(values.begin(), values.end());
}

template<typename T, std::size_t Size>
std::array<T, Size> checkedArray(
  const std::vector<T> & values,
  const std::string & parameter_name)
{
  if (values.size() != Size) {
    throw std::invalid_argument(
            parameter_name + " must contain exactly " + std::to_string(Size) +
            " values");
  }
  std::array<T, Size> result;
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

}  // namespace

class BlueRovIntegrationNode final : public rclcpp::Node
{
public:
  BlueRovIntegrationNode()
  : Node("bluerov_integration_node")
  {
    const double control_rate_hz =
      declare_parameter<double>("control.rate_hz", 20.0);
    const double planning_rate_hz =
      declare_parameter<double>("planning.check_rate_hz", 5.0);
    const double status_rate_hz =
      declare_parameter<double>("status.rate_hz", 1.0);
    if (control_rate_hz <= 0.0 || planning_rate_hz <= 0.0 ||
      status_rate_hz <= 0.0)
    {
      throw std::invalid_argument("All timer rates must be positive");
    }

    auto hub_config = loadHubConfig();
    auto planning_config = loadPlanningConfig();
    hub_config.reference_path_topic = planning_config.path_topic;
    auto path_follower_config = loadPathFollowerConfig();
    auto control_config = loadControlConfig(1.0 / control_rate_hz);
    if (path_follower_config.enabled &&
      !planning_config.use_target_topic_for_goal)
    {
      throw std::invalid_argument(
              "Path following requires planning.use_target_topic_for_goal=true");
    }
    if (path_follower_config.enabled &&
      control_config.enabled && !planning_config.enabled)
    {
      throw std::invalid_argument(
              "Control with path following requires planning.enabled=true");
    }

    hub_ = std::make_unique<common::DataHub>(*this, std::move(hub_config));
    planning_module_ = std::make_unique<team_min::PlanningModule>(
      *this, std::move(planning_config));
    path_follower_ = std::make_unique<integration::PathFollower>(
      *this, std::move(path_follower_config));
    control_module_ = std::make_unique<team_byung::ControlModule>(
      *this, std::move(control_config));

    planning_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    planning_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / planning_rate_hz),
      [this]() {
        const auto local_snapshot = hub_->snapshot();
        planning_module_->update(local_snapshot);
      },
      planning_group_);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz),
      [this]() {
        const auto local_snapshot = hub_->snapshot();
        const auto tracking_target =
          path_follower_->selectTarget(local_snapshot);
        control_module_->update(local_snapshot, tracking_target);
      },
      control_group_);
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_rate_hz),
      std::bind(&BlueRovIntegrationNode::printStatus, this),
      status_group_);

    RCLCPP_INFO(
      get_logger(),
      "BlueROV ready: Hub -> A* -> PathFollower -> PPID");
  }

  ~BlueRovIntegrationNode() override
  {
    if (planning_module_) {
      planning_module_->stop();
    }
    if (control_module_) {
      control_module_->stop();
    }
  }

private:
  common::DataHubConfig loadHubConfig()
  {
    common::DataHubConfig config;
    config.pressure_topic = declare_parameter<std::string>(
      "topics.pressure", config.pressure_topic);
    config.depth_topic = declare_parameter<std::string>(
      "topics.depth", config.depth_topic);
    config.imu_topic = declare_parameter<std::string>(
      "topics.imu", config.imu_topic);
    config.bluerov_odometry_topic = declare_parameter<std::string>(
      "topics.bluerov_odometry", config.bluerov_odometry_topic);
    config.dvl_topic = declare_parameter<std::string>(
      "topics.dvl", config.dvl_topic);
    config.torpedo_odometry_topic = declare_parameter<std::string>(
      "topics.torpedo_odometry", config.torpedo_odometry_topic);
    config.mission_goal_topic = declare_parameter<std::string>(
      "topics.mission_goal", config.mission_goal_topic);
    return config;
  }

  team_min::PlanningConfig loadPlanningConfig()
  {
    team_min::PlanningConfig config;
    config.enabled = declare_parameter<bool>(
      "planning.enabled", config.enabled);
    config.use_dynamic_map = declare_parameter<bool>(
      "planning.use_dynamic_map", config.use_dynamic_map);
    config.use_target_topic_for_goal = declare_parameter<bool>(
      "planning.use_target_topic_for_goal",
      config.use_target_topic_for_goal);
    config.goal_offset_x = declare_parameter<double>(
      "planning.goal_offset_x", config.goal_offset_x);
    config.goal_offset_y = declare_parameter<double>(
      "planning.goal_offset_y", config.goal_offset_y);
    config.goal_offset_z = declare_parameter<double>(
      "planning.goal_offset_z", config.goal_offset_z);
    config.map_padding_x = declare_parameter<double>(
      "planning.map_padding_x", config.map_padding_x);
    config.map_padding_y = declare_parameter<double>(
      "planning.map_padding_y", config.map_padding_y);
    config.map_padding_z = declare_parameter<double>(
      "planning.map_padding_z", config.map_padding_z);
    config.robot_replan_distance = declare_parameter<double>(
      "planning.robot_replan_distance", config.robot_replan_distance);
    config.torpedo_replan_distance = declare_parameter<double>(
      "planning.torpedo_replan_distance", config.torpedo_replan_distance);
    config.goal_replan_distance = declare_parameter<double>(
      "planning.goal_replan_distance", config.goal_replan_distance);

    config.fixed_map.min_x = declare_parameter<double>(
      "planning.map.min_x", -1000.0);
    config.fixed_map.max_x = declare_parameter<double>(
      "planning.map.max_x", 1000.0);
    config.fixed_map.min_y = declare_parameter<double>(
      "planning.map.min_y", -1000.0);
    config.fixed_map.max_y = declare_parameter<double>(
      "planning.map.max_y", 1000.0);
    config.fixed_map.min_z = declare_parameter<double>(
      "planning.map.min_z", -100.0);
    config.fixed_map.max_z = declare_parameter<double>(
      "planning.map.max_z", 100.0);
    config.fixed_map.resolution = declare_parameter<double>(
      "planning.map.resolution", config.fixed_map.resolution);
    const std::int64_t max_cells = declare_parameter<std::int64_t>(
      "planning.map.max_cells",
      static_cast<std::int64_t>(config.fixed_map.max_cells));
    if (max_cells <= 0) {
      throw std::invalid_argument("planning.map.max_cells must be positive");
    }
    config.fixed_map.max_cells = static_cast<std::size_t>(max_cells);

    config.astar.allow_diagonal = declare_parameter<bool>(
      "planning.astar.allow_diagonal", config.astar.allow_diagonal);
    config.astar.heuristic_weight = declare_parameter<double>(
      "planning.astar.heuristic_weight", config.astar.heuristic_weight);
    config.astar.tie_break_offset = declare_parameter<double>(
      "planning.astar.tie_break_offset", config.astar.tie_break_offset);
    config.astar.safety_margin = declare_parameter<double>(
      "planning.astar.safety_margin", config.astar.safety_margin);
    config.torpedo_barrier.size_x = declare_parameter<double>(
      "planning.barrier.size_x", config.torpedo_barrier.size_x);
    config.torpedo_barrier.size_y = declare_parameter<double>(
      "planning.barrier.size_y", config.torpedo_barrier.size_y);
    config.torpedo_barrier.size_z = declare_parameter<double>(
      "planning.barrier.size_z", config.torpedo_barrier.size_z);

    config.path_topic = declare_parameter<std::string>(
      "planning.topics.path", config.path_topic);
    config.current_point_topic = declare_parameter<std::string>(
      "planning.topics.current_point", config.current_point_topic);
    config.goal_point_topic = declare_parameter<std::string>(
      "planning.topics.goal_point", config.goal_point_topic);
    config.torpedo_point_topic = declare_parameter<std::string>(
      "planning.topics.torpedo_point", config.torpedo_point_topic);
    config.marker_topic = declare_parameter<std::string>(
      "planning.topics.markers", config.marker_topic);
    return config;
  }

  integration::PathFollowerConfig loadPathFollowerConfig()
  {
    integration::PathFollowerConfig config;
    config.enabled = declare_parameter<bool>(
      "path_following.enabled", config.enabled);
    config.stop_without_valid_path = declare_parameter<bool>(
      "path_following.stop_without_valid_path",
      config.stop_without_valid_path);
    config.require_matching_frame = declare_parameter<bool>(
      "path_following.require_matching_frame",
      config.require_matching_frame);
    config.lookahead_distance = declare_parameter<double>(
      "path_following.lookahead_distance",
      config.lookahead_distance);
    config.waypoint_reached_radius = declare_parameter<double>(
      "path_following.waypoint_reached_radius",
      config.waypoint_reached_radius);
    config.tracking_target_topic = declare_parameter<std::string>(
      "path_following.tracking_target_topic",
      config.tracking_target_topic);
    return config;
  }

  team_byung::PIDGains loadPid(
    const std::string & prefix,
    const team_byung::PIDGains & defaults)
  {
    team_byung::PIDGains gains;
    gains.kp = declare_parameter<double>(prefix + ".kp", defaults.kp);
    gains.ki = declare_parameter<double>(prefix + ".ki", defaults.ki);
    gains.kd = declare_parameter<double>(prefix + ".kd", defaults.kd);
    gains.integral_limit = declare_parameter<double>(
      prefix + ".integral_limit", defaults.integral_limit);
    gains.output_limit = declare_parameter<double>(
      prefix + ".output_limit", defaults.output_limit);
    return gains;
  }

  std::array<double, 6> loadMixerArray(
    const std::string & name,
    const std::array<double, 6> & defaults)
  {
    return checkedArray<double, 6>(
      declare_parameter<std::vector<double>>(name, toVector(defaults)), name);
  }

  team_byung::ControlModuleConfig loadControlConfig(const double nominal_dt)
  {
    team_byung::ControlModuleConfig config;
    config.enabled = declare_parameter<bool>(
      "control.enabled", config.enabled);
    config.require_matching_target_frame = declare_parameter<bool>(
      "control.require_matching_target_frame",
      config.require_matching_target_frame);
    config.velocity_source = declare_parameter<std::string>(
      "control.velocity_source", config.velocity_source);
    config.nominal_dt = nominal_dt;
    config.odometry_timeout_sec = declare_parameter<double>(
      "control.odometry_timeout_sec", config.odometry_timeout_sec);
    config.dvl_timeout_sec = declare_parameter<double>(
      "control.dvl_timeout_sec", config.dvl_timeout_sec);
    config.target_timeout_sec = declare_parameter<double>(
      "control.target_timeout_sec", config.target_timeout_sec);
    config.thruster_topics = checkedArray<std::string, 6>(
      declare_parameter<std::vector<std::string>>(
        "control.thruster_topics", toVector(config.thruster_topics)),
      "control.thruster_topics");

    auto & controller = config.controller;
    controller.position_gain.x = declare_parameter<double>(
      "control.position_gain.x", controller.position_gain.x);
    controller.position_gain.y = declare_parameter<double>(
      "control.position_gain.y", controller.position_gain.y);
    controller.position_gain.z = declare_parameter<double>(
      "control.position_gain.z", controller.position_gain.z);
    controller.max_horizontal_speed = declare_parameter<double>(
      "control.max_horizontal_speed", controller.max_horizontal_speed);
    controller.max_vertical_speed = declare_parameter<double>(
      "control.max_vertical_speed", controller.max_vertical_speed);

    controller.velocity_gains[0] = loadPid(
      "control.velocity_pid.x", controller.velocity_gains[0]);
    controller.velocity_gains[1] = loadPid(
      "control.velocity_pid.y", controller.velocity_gains[1]);
    controller.velocity_gains[2] = loadPid(
      "control.velocity_pid.z", controller.velocity_gains[2]);
    controller.velocity_output_limit.x = declare_parameter<double>(
      "control.velocity_output_limit.x", controller.velocity_output_limit.x);
    controller.velocity_output_limit.y = declare_parameter<double>(
      "control.velocity_output_limit.y", controller.velocity_output_limit.y);
    controller.velocity_output_limit.z = declare_parameter<double>(
      "control.velocity_output_limit.z", controller.velocity_output_limit.z);
    controller.velocity_feedforward.x = declare_parameter<double>(
      "control.velocity_feedforward.x", controller.velocity_feedforward.x);
    controller.velocity_feedforward.y = declare_parameter<double>(
      "control.velocity_feedforward.y", controller.velocity_feedforward.y);
    controller.velocity_feedforward.z = declare_parameter<double>(
      "control.velocity_feedforward.z", controller.velocity_feedforward.z);
    controller.yaw_gains = loadPid(
      "control.yaw_pid", controller.yaw_gains);
    controller.yaw_command_limit = declare_parameter<double>(
      "control.yaw_command_limit", controller.yaw_command_limit);
    controller.heave_trim = declare_parameter<double>(
      "control.heave_trim", controller.heave_trim);

    controller.mixer.command_limit = declare_parameter<double>(
      "control.mixer.command_limit", controller.mixer.command_limit);
    controller.mixer.x_coefficients = loadMixerArray(
      "control.mixer.x", controller.mixer.x_coefficients);
    controller.mixer.y_coefficients = loadMixerArray(
      "control.mixer.y", controller.mixer.y_coefficients);
    controller.mixer.z_coefficients = loadMixerArray(
      "control.mixer.z", controller.mixer.z_coefficients);
    controller.mixer.yaw_coefficients = loadMixerArray(
      "control.mixer.yaw", controller.mixer.yaw_coefficients);

    config.logger.enabled = declare_parameter<bool>(
      "control.logging.enabled", config.logger.enabled);
    config.logger.generate_html = declare_parameter<bool>(
      "control.logging.generate_html", config.logger.generate_html);
    config.logger.directory = std::filesystem::path(
      declare_parameter<std::string>(
        "control.logging.directory", config.logger.directory.string()));
    const std::int64_t queue_limit = declare_parameter<std::int64_t>(
      "control.logging.queue_limit",
      static_cast<std::int64_t>(config.logger.queue_limit));
    if (queue_limit <= 0) {
      throw std::invalid_argument(
              "control.logging.queue_limit must be positive");
    }
    config.logger.queue_limit = static_cast<std::size_t>(queue_limit);
    return config;
  }

  void printStatus()
  {
    const auto snapshot = hub_->snapshot();
    RCLCPP_INFO(
      get_logger(),
      "[HUB] version=%llu pressure=%d depth=%d imu=%d odom=%d dvl=%d "
      "torpedo=%d mission=%d path=%d logger_overflow=%llu",
      static_cast<unsigned long long>(snapshot.version),
      snapshot.pressure.metadata.valid,
      snapshot.depth.metadata.valid,
      snapshot.imu.metadata.valid,
      snapshot.bluerov_odometry.metadata.valid,
      snapshot.dvl.metadata.valid,
      snapshot.torpedo_odometry.metadata.valid,
      snapshot.mission_goal.metadata.valid,
      snapshot.reference_path.metadata.valid,
      static_cast<unsigned long long>(
        control_module_->loggerOverflowCount()));
  }

  std::unique_ptr<common::DataHub> hub_;
  std::unique_ptr<team_min::PlanningModule> planning_module_;
  std::unique_ptr<integration::PathFollower> path_follower_;
  std::unique_ptr<team_byung::ControlModule> control_module_;
  rclcpp::CallbackGroup::SharedPtr planning_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace bluerov_integration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node =
      std::make_shared<bluerov_integration::BlueRovIntegrationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 4U);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    std::cerr << "BlueROV integration startup failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
