#include "bluerov_integration/team_min/planning_config.hpp"

namespace bluerov_integration::team_min
{

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
  core.ttc = config.ttc;
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
  if (name == "spacetime") {
    return PlannerType::kSpaceTime;
  }
  return std::nullopt;
}

}  // namespace bluerov_integration::team_min
