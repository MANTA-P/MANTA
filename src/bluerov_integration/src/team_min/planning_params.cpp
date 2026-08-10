#include "bluerov_integration/team_min/planning_params.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace bluerov_integration::team_min
{
namespace
{

// 예측 파라미터는 team_min 소유이므로 통합 노드(loadPlanningConfig)를
// 수정하지 않고 여기서 직접 선언한다.
PredictionConfig loadPredictionParameters(
  rclcpp::Node & node,
  PredictionConfig defaults)
{
  defaults.enabled = node.declare_parameter<bool>(
    "planning.prediction.enabled", defaults.enabled);
  defaults.horizon_sec = node.declare_parameter<double>(
    "planning.prediction.horizon_sec", defaults.horizon_sec);
  defaults.spacing = node.declare_parameter<double>(
    "planning.prediction.spacing", defaults.spacing);
  const std::int64_t max_boxes = node.declare_parameter<std::int64_t>(
    "planning.prediction.max_boxes",
    static_cast<std::int64_t>(defaults.max_boxes));
  if (max_boxes < 0 || max_boxes > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(
            "planning.prediction.max_boxes is out of range");
  }
  defaults.max_boxes = static_cast<int>(max_boxes);
  defaults.min_speed = node.declare_parameter<double>(
    "planning.prediction.min_speed", defaults.min_speed);
  defaults.velocity_alpha = node.declare_parameter<double>(
    "planning.prediction.velocity_alpha", defaults.velocity_alpha);
  defaults.start_clearance = node.declare_parameter<double>(
    "planning.prediction.start_clearance", defaults.start_clearance);
  return defaults;
}

ReplanPolicyConfig loadReplanParameters(
  rclcpp::Node & node,
  ReplanPolicyConfig defaults)
{
  defaults.collision_only = node.declare_parameter<bool>(
    "planning.replan.collision_only", defaults.collision_only);
  defaults.path_deviation_distance = node.declare_parameter<double>(
    "planning.replan.path_deviation_distance",
    defaults.path_deviation_distance);
  defaults.collision_margin = node.declare_parameter<double>(
    "planning.replan.collision_margin", defaults.collision_margin);
  defaults.min_interval_sec = node.declare_parameter<double>(
    "planning.replan.min_interval_sec", defaults.min_interval_sec);
  return defaults;
}

AvoidConfig loadAvoidParameters(
  rclcpp::Node & node,
  AvoidConfig defaults)
{
  defaults.torpedo_timeout_sec = node.declare_parameter<double>(
    "planning.avoid.torpedo_timeout_sec", defaults.torpedo_timeout_sec);
  defaults.engage_radius = node.declare_parameter<double>(
    "planning.avoid.engage_radius", defaults.engage_radius);
  defaults.hit_radius = node.declare_parameter<double>(
    "planning.avoid.hit_radius", defaults.hit_radius);
  return defaults;
}

// Dynamic VO: parameters are declared by the team_min adapter.
DynamicVOOptions loadDynamicVOParameters(
  rclcpp::Node & node,
  DynamicVOOptions defaults)
{
  defaults.prediction_horizon = node.declare_parameter<double>(
    "planning.dynamic_vo.prediction_horizon", defaults.prediction_horizon);
  defaults.rollout_step = node.declare_parameter<double>(
    "planning.dynamic_vo.rollout_step", defaults.rollout_step);
  const auto rollout_steps = node.declare_parameter<std::int64_t>(
    "planning.dynamic_vo.rollout_steps",
    static_cast<std::int64_t>(defaults.rollout_steps));
  if (rollout_steps <= 0) {
    throw std::invalid_argument("planning.dynamic_vo.rollout_steps must be positive");
  }
  defaults.rollout_steps = static_cast<std::size_t>(rollout_steps);
  defaults.max_horizontal_speed = node.declare_parameter<double>(
    "planning.dynamic_vo.max_horizontal_speed", defaults.max_horizontal_speed);
  defaults.max_vertical_speed = node.declare_parameter<double>(
    "planning.dynamic_vo.max_vertical_speed", defaults.max_vertical_speed);
  defaults.robot_radius = node.declare_parameter<double>(
    "planning.dynamic_vo.robot_radius", defaults.robot_radius);
  defaults.safety_margin = node.declare_parameter<double>(
    "planning.dynamic_vo.safety_margin", defaults.safety_margin);
  defaults.goal_tolerance = node.declare_parameter<double>(
    "planning.dynamic_vo.goal_tolerance", defaults.goal_tolerance);
  defaults.path_lookahead = node.declare_parameter<double>(
    "planning.dynamic_vo.path_lookahead", defaults.path_lookahead);
  return defaults;
}

}  // namespace

void loadTeamMinParameters(rclcpp::Node & node, PlanningConfig & config)
{
  config.prediction = loadPredictionParameters(node, config.prediction);
  config.replan = loadReplanParameters(node, config.replan);
  config.avoid = loadAvoidParameters(node, config.avoid);
  // Dynamic VO: load defaults without changing the integration node.
  config.dynamic_vo = loadDynamicVOParameters(node, config.dynamic_vo);
}

PlanningCoreConfig toCoreConfig(const PlanningConfig & config)
{
  PlanningCoreConfig core;
  core.use_dynamic_map = config.use_dynamic_map;
  core.use_target_topic_for_goal = config.use_target_topic_for_goal;
  core.goal_offset_x = config.goal_offset_x;
  core.goal_offset_y = config.goal_offset_y;
  core.goal_offset_z = config.goal_offset_z;
  core.map_padding_x = config.map_padding_x;
  core.map_padding_y = config.map_padding_y;
  core.map_padding_z = config.map_padding_z;
  core.torpedo_replan_distance = config.torpedo_replan_distance;
  core.robot_replan_distance = config.robot_replan_distance;
  core.goal_replan_distance = config.goal_replan_distance;
  core.fixed_map = config.fixed_map;
  core.astar = config.astar;
  core.torpedo_barrier = config.torpedo_barrier;
  core.prediction = config.prediction;
  core.replan = config.replan;
  core.avoid = config.avoid;
  return core;
}

std::optional<PlannerType> parsePlanner(const std::string & name)
{
  if (name == "astar") {
    return PlannerType::kAStar;
  }
  if (name == "dvo") {
    return PlannerType::kDynamicVO;
  }
  if (name == "hybrid") {
    return PlannerType::kHybrid;
  }
  return std::nullopt;
}

}  // namespace bluerov_integration::team_min
