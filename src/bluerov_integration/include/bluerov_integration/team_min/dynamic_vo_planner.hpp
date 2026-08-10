#pragma once

#include <cstddef>
#include <vector>

#include "bluerov_integration/team_min/astar_planner.hpp"

namespace bluerov_integration::team_min
{

// Dynamic VO: planner inputs use the same world frame as Point3D.
struct MovingObstacle
{
  BoxObstacle shape;
  Point3D velocity;
};

struct DynamicVOOptions
{
  double prediction_horizon{6.0};
  double rollout_step{0.5};
  std::size_t rollout_steps{20U};
  double max_horizontal_speed{3.0};
  double max_vertical_speed{1.5};
  double robot_radius{0.75};
  double safety_margin{0.50};
  double goal_tolerance{0.75};
  double path_lookahead{6.0};
  int heading_samples{36};
  int vertical_samples{7};
  int speed_samples{5};
  // true면 A*에 기대지 않고 단독으로 계획한다. 충돌 위험이 없어도 목표를
  // 향해 롤아웃하고, 목표에 못 닿아도 부분 경로를 성공으로 인정한다
  // (매 틱 다시 뽑는 receding-horizon). false면 기존 하이브리드 동작.
  bool standalone{false};
};

struct DynamicVOResult
{
  bool avoidance_required{false};
  bool success{false};
  Point3D selected_velocity;
  std::vector<Point3D> local_path;
};

// Dynamic VO: returns a short local path toward an A* lookahead point.
DynamicVOResult runDynamicVO3D(
  const Point3D & robot_position,
  const Point3D & robot_velocity,
  const Point3D & local_goal,
  const GridMapConfig & map_config,
  const std::vector<MovingObstacle> & obstacles,
  const DynamicVOOptions & options = DynamicVOOptions{});

}  // namespace bluerov_integration::team_min
