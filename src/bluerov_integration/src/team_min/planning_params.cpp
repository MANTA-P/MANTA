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
  defaults.guidance_enabled = node.declare_parameter<bool>(
    "planning.prediction.guidance.enabled", defaults.guidance_enabled);
  defaults.guidance_horizon_sec = node.declare_parameter<double>(
    "planning.prediction.guidance.horizon_sec", defaults.guidance_horizon_sec);
  defaults.guidance_step_sec = node.declare_parameter<double>(
    "planning.prediction.guidance.step_sec", defaults.guidance_step_sec);
  defaults.pn_navigation_constant = node.declare_parameter<double>(
    "planning.prediction.guidance.pn_navigation_constant",
    defaults.pn_navigation_constant);
  defaults.pursuit_turn_rate = node.declare_parameter<double>(
    "planning.prediction.guidance.pursuit_turn_rate",
    defaults.pursuit_turn_rate);
  defaults.guidance_max_lateral_acceleration = node.declare_parameter<double>(
    "planning.prediction.guidance.max_lateral_acceleration",
    defaults.guidance_max_lateral_acceleration);
  defaults.classification_alpha = node.declare_parameter<double>(
    "planning.prediction.guidance.classification_alpha",
    defaults.classification_alpha);
  const auto min_samples = node.declare_parameter<std::int64_t>(
    "planning.prediction.guidance.classification_min_samples",
    static_cast<std::int64_t>(defaults.classification_min_samples));
  if (min_samples <= 0) {
    throw std::invalid_argument("classification_min_samples must be positive");
  }
  defaults.classification_min_samples = static_cast<std::size_t>(min_samples);
  defaults.classification_ratio = node.declare_parameter<double>(
    "planning.prediction.guidance.classification_ratio",
    defaults.classification_ratio);

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

// 시공간 A* 파라미터. 분기비율(max_speed*dt/resolution)이 2 미만이면
// 코어가 예외를 던지므로, 셋을 함께 바꿔야 한다.
void loadSpaceTimeParameters(rclcpp::Node & node, SpaceTimeOptions & options,
  double & lookahead)
{
  options.resolution = node.declare_parameter<double>(
    "planning.spacetime.resolution", options.resolution);
  options.dt = node.declare_parameter<double>(
    "planning.spacetime.dt", options.dt);
  options.horizon_sec = node.declare_parameter<double>(
    "planning.spacetime.horizon_sec", options.horizon_sec);
  options.half_extent_xy = node.declare_parameter<double>(
    "planning.spacetime.half_extent_xy", options.half_extent_xy);
  options.half_extent_z = node.declare_parameter<double>(
    "planning.spacetime.half_extent_z", options.half_extent_z);
  options.max_speed = node.declare_parameter<double>(
    "planning.spacetime.max_speed", options.max_speed);
  options.max_vertical_speed = node.declare_parameter<double>(
    "planning.spacetime.max_vertical_speed", options.max_vertical_speed);
  options.safety_margin = node.declare_parameter<double>(
    "planning.spacetime.safety_margin", options.safety_margin);
  lookahead = node.declare_parameter<double>(
    "planning.spacetime.lookahead", lookahead);
}

// TTC 국면 전환 파라미터. 기본값의 근거는 planning_types.hpp 주석에 있다
// (Zarchan 3~5τ 규칙 / DVO 예측지평 / arXiv 2506.20311 C2 조건).
void loadTtcParameters(rclcpp::Node & node, TtcSwitchConfig & ttc)
{
  ttc.enabled = node.declare_parameter<bool>(
    "planning.ttc.enabled", ttc.enabled);
  ttc.approach_sec = node.declare_parameter<double>(
    "planning.ttc.approach_sec", ttc.approach_sec);
  ttc.break_sec = node.declare_parameter<double>(
    "planning.ttc.break_sec", ttc.break_sec);
  ttc.reverse_sec = node.declare_parameter<double>(
    "planning.ttc.reverse_sec", ttc.reverse_sec);
  ttc.dwell_sec = node.declare_parameter<double>(
    "planning.ttc.dwell_sec", ttc.dwell_sec);
  ttc.minimum_closing = node.declare_parameter<double>(
    "planning.ttc.minimum_closing", ttc.minimum_closing);
}


void loadTeamMinParameters(rclcpp::Node & node, PlanningConfig & config)
{
  loadTtcParameters(node, config.ttc);
  config.prediction = loadPredictionParameters(node, config.prediction);
  config.replan = loadReplanParameters(node, config.replan);
  loadSpaceTimeParameters(node, config.spacetime, config.spacetime_lookahead);
  config.avoid = loadAvoidParameters(node, config.avoid);
  // Dynamic VO: load defaults without changing the integration node.
  config.dynamic_vo = loadDynamicVOParameters(node, config.dynamic_vo);
}

}  // namespace bluerov_integration::team_min
