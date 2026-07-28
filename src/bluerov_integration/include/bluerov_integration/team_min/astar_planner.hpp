#pragma once

#include <cstddef>
#include <vector>

namespace bluerov_integration::team_min
{

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct BoxObstacle
{
  Point3D center;
  double size_x{3.0};
  double size_y{3.0};
  double size_z{3.0};
};

struct GridMapConfig
{
  double min_x{-20.0};
  double max_x{20.0};
  double min_y{-20.0};
  double max_y{20.0};
  double min_z{-10.0};
  double max_z{5.0};
  double resolution{1.0};
  std::size_t max_cells{50'000'000U};
};

struct AStarOptions
{
  bool allow_diagonal{true};
  double heuristic_weight{1.2};
  double tie_break_offset{0.001};
  double safety_margin{0.0};
};

double distance3D(const Point3D & first, const Point3D & second);

std::vector<Point3D> runEnhancedAStar3D(
  const Point3D & start_world,
  const Point3D & goal_world,
  const GridMapConfig & map_config,
  const std::vector<BoxObstacle> & obstacles,
  const AStarOptions & options = AStarOptions{});

}  // namespace bluerov_integration::team_min
