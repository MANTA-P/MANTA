#include "bluerov_integration/team_min/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bluerov_integration::team_min
{
namespace
{

struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & index) const noexcept
  {
    const std::size_t x = std::hash<int>{}(index.x);
    const std::size_t y = std::hash<int>{}(index.y);
    const std::size_t z = std::hash<int>{}(index.z);
    return x ^ (y << 1U) ^ (z << 2U);
  }
};

struct QueueNode
{
  GridIndex index;
  double score{0.0};

  bool operator>(const QueueNode & other) const
  {
    return score > other.score;
  }
};

class GridMap3D
{
public:
  explicit GridMap3D(const GridMapConfig & config)
  : config_(config)
  {
    validate();

    const double cells_x =
      std::ceil((config_.max_x - config_.min_x) / config_.resolution);
    const double cells_y =
      std::ceil((config_.max_y - config_.min_y) / config_.resolution);
    const double cells_z =
      std::ceil((config_.max_z - config_.min_z) / config_.resolution);
    const double int_max = static_cast<double>(std::numeric_limits<int>::max());
    if (cells_x > int_max || cells_y > int_max || cells_z > int_max) {
      throw std::runtime_error("Grid dimension exceeds integer index range");
    }

    size_x_ = static_cast<int>(cells_x);
    size_y_ = static_cast<int>(cells_y);
    size_z_ = static_cast<int>(cells_z);

    const std::size_t x = static_cast<std::size_t>(size_x_);
    const std::size_t y = static_cast<std::size_t>(size_y_);
    const std::size_t z = static_cast<std::size_t>(size_z_);
    if (x > std::numeric_limits<std::size_t>::max() / y ||
      x * y > std::numeric_limits<std::size_t>::max() / z)
    {
      throw std::runtime_error("Grid cell count overflow");
    }

    const std::size_t total = x * y * z;
    if (total > config_.max_cells) {
      std::ostringstream message;
      message << "Grid is too large: " << size_x_ << " x " << size_y_ << " x " <<
        size_z_ << " = " << total << ", limit=" << config_.max_cells;
      throw std::runtime_error(message.str());
    }
    occupancy_.assign(total, 0U);
  }

  bool isInside(const GridIndex & index) const
  {
    return index.x >= 0 && index.x < size_x_ &&
           index.y >= 0 && index.y < size_y_ &&
           index.z >= 0 && index.z < size_z_;
  }

  bool isFree(const GridIndex & index) const
  {
    return isInside(index) && occupancy_[linear(index)] == 0U;
  }

  GridIndex worldToGrid(const Point3D & point) const
  {
    return {
      static_cast<int>(std::floor((point.x - config_.min_x) / config_.resolution)),
      static_cast<int>(std::floor((point.y - config_.min_y) / config_.resolution)),
      static_cast<int>(std::floor((point.z - config_.min_z) / config_.resolution))};
  }

  Point3D gridToWorld(const GridIndex & index) const
  {
    return {
      config_.min_x + (static_cast<double>(index.x) + 0.5) * config_.resolution,
      config_.min_y + (static_cast<double>(index.y) + 0.5) * config_.resolution,
      config_.min_z + (static_cast<double>(index.z) + 0.5) * config_.resolution};
  }

  void addBox(const BoxObstacle & obstacle, const double safety_margin)
  {
    if (obstacle.size_x <= 0.0 || obstacle.size_y <= 0.0 ||
      obstacle.size_z <= 0.0 || safety_margin < 0.0)
    {
      throw std::invalid_argument("Invalid box obstacle");
    }

    const double half_x = obstacle.size_x * 0.5 + safety_margin;
    const double half_y = obstacle.size_y * 0.5 + safety_margin;
    const double half_z = obstacle.size_z * 0.5 + safety_margin;
    const GridIndex raw_min = worldToGrid(
      {obstacle.center.x - half_x, obstacle.center.y - half_y,
        obstacle.center.z - half_z});
    const GridIndex raw_max = worldToGrid(
      {obstacle.center.x + half_x, obstacle.center.y + half_y,
        obstacle.center.z + half_z});

    const int min_x = std::max(0, raw_min.x);
    const int min_y = std::max(0, raw_min.y);
    const int min_z = std::max(0, raw_min.z);
    const int max_x = std::min(size_x_ - 1, raw_max.x);
    const int max_y = std::min(size_y_ - 1, raw_max.y);
    const int max_z = std::min(size_z_ - 1, raw_max.z);
    if (min_x > max_x || min_y > max_y || min_z > max_z) {
      return;
    }

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        for (int z = min_z; z <= max_z; ++z) {
          occupancy_[linear({x, y, z})] = 1U;
        }
      }
    }
  }

  double diagonalLength() const
  {
    const double x = config_.max_x - config_.min_x;
    const double y = config_.max_y - config_.min_y;
    const double z = config_.max_z - config_.min_z;
    return std::sqrt(x * x + y * y + z * z);
  }

private:
  void validate() const
  {
    if (!std::isfinite(config_.min_x) || !std::isfinite(config_.max_x) ||
      !std::isfinite(config_.min_y) || !std::isfinite(config_.max_y) ||
      !std::isfinite(config_.min_z) || !std::isfinite(config_.max_z) ||
      !std::isfinite(config_.resolution) ||
      config_.max_x <= config_.min_x || config_.max_y <= config_.min_y ||
      config_.max_z <= config_.min_z || config_.resolution <= 0.0 ||
      config_.max_cells == 0U)
    {
      throw std::invalid_argument("Invalid 3D grid configuration");
    }
  }

  std::size_t linear(const GridIndex & index) const
  {
    return static_cast<std::size_t>(index.z) *
             static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_) +
           static_cast<std::size_t>(index.y) * static_cast<std::size_t>(size_x_) +
           static_cast<std::size_t>(index.x);
  }

  GridMapConfig config_;
  int size_x_{0};
  int size_y_{0};
  int size_z_{0};
  std::vector<std::uint8_t> occupancy_;
};

double enhancedScore(
  const double g_cost,
  const double h_cost,
  const GridMap3D & grid,
  const AStarOptions & options)
{
  const double diagonal = std::max(grid.diagonalLength(), 1e-9);
  const double normalized = h_cost / diagonal;
  const double weight = std::exp(
    normalized * (options.heuristic_weight + options.tie_break_offset));
  return g_cost + weight * h_cost;
}

bool diagonalTransitionIsFree(
  const GridMap3D & grid,
  const GridIndex & current,
  const int dx,
  const int dy,
  const int dz)
{
  int active_axes = 0;
  if (dx != 0) {active_axes |= 0x1;}
  if (dy != 0) {active_axes |= 0x2;}
  if (dz != 0) {active_axes |= 0x4;}
  if (active_axes == 0x1 || active_axes == 0x2 || active_axes == 0x4) {
    return true;
  }

  for (int subset = 1; subset < 0x8; ++subset) {
    if ((subset & ~active_axes) != 0 || subset == active_axes) {
      continue;
    }
    GridIndex intermediate = current;
    if ((subset & 0x1) != 0) {intermediate.x += dx;}
    if ((subset & 0x2) != 0) {intermediate.y += dy;}
    if ((subset & 0x4) != 0) {intermediate.z += dz;}
    if (!grid.isFree(intermediate)) {
      return false;
    }
  }
  return true;
}

std::vector<GridIndex> neighbors(
  const GridMap3D & grid,
  const GridIndex & current,
  const bool allow_diagonal)
{
  std::vector<GridIndex> result;
  result.reserve(allow_diagonal ? 26U : 6U);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const int changed = (dx != 0) + (dy != 0) + (dz != 0);
        if (changed == 0 || (!allow_diagonal && changed > 1)) {
          continue;
        }
        const GridIndex next{current.x + dx, current.y + dy, current.z + dz};
        if (grid.isFree(next) &&
          diagonalTransitionIsFree(grid, current, dx, dy, dz))
        {
          result.push_back(next);
        }
      }
    }
  }
  return result;
}

std::vector<Point3D> reconstruct(
  const GridMap3D & grid,
  const GridIndex & start,
  const GridIndex & goal,
  const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & parent,
  const Point3D & exact_start,
  const Point3D & exact_goal)
{
  std::vector<GridIndex> reversed{goal};
  GridIndex current = goal;
  while (!(current == start)) {
    const auto found = parent.find(current);
    if (found == parent.end()) {
      return {};
    }
    current = found->second;
    reversed.push_back(current);
  }
  std::reverse(reversed.begin(), reversed.end());

  std::vector<Point3D> path;
  path.reserve(reversed.size());
  for (const auto & index : reversed) {
    path.push_back(grid.gridToWorld(index));
  }
  path.front() = exact_start;
  path.back() = exact_goal;
  return path;
}

}  // namespace

double distance3D(const Point3D & first, const Point3D & second)
{
  const double x = first.x - second.x;
  const double y = first.y - second.y;
  const double z = first.z - second.z;
  return std::sqrt(x * x + y * y + z * z);
}

std::vector<Point3D> runEnhancedAStar3D(
  const Point3D & start_world,
  const Point3D & goal_world,
  const GridMapConfig & map_config,
  const std::vector<BoxObstacle> & obstacles,
  const AStarOptions & options)
{
  if (options.heuristic_weight < 0.0 || options.tie_break_offset < 0.0 ||
    options.safety_margin < 0.0)
  {
    throw std::invalid_argument("A* option values must be non-negative");
  }

  GridMap3D grid(map_config);
  for (const auto & obstacle : obstacles) {
    grid.addBox(obstacle, options.safety_margin);
  }

  const GridIndex start = grid.worldToGrid(start_world);
  const GridIndex goal = grid.worldToGrid(goal_world);
  if (!grid.isInside(start) || !grid.isInside(goal)) {
    throw std::runtime_error("A* start or goal is outside the grid");
  }
  if (!grid.isFree(start) || !grid.isFree(goal)) {
    throw std::runtime_error("A* start or goal is inside an obstacle");
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;
  std::unordered_set<GridIndex, GridIndexHash> closed;
  std::unordered_map<GridIndex, double, GridIndexHash> g_cost;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> parent;

  g_cost[start] = 0.0;
  const double start_h = distance3D(grid.gridToWorld(start), grid.gridToWorld(goal));
  open.push({start, enhancedScore(0.0, start_h, grid, options)});

  while (!open.empty()) {
    const GridIndex current = open.top().index;
    open.pop();
    if (closed.find(current) != closed.end()) {
      continue;
    }
    if (current == goal) {
      return reconstruct(grid, start, goal, parent, start_world, goal_world);
    }
    closed.insert(current);

    for (const auto & next : neighbors(grid, current, options.allow_diagonal)) {
      if (closed.find(next) != closed.end()) {
        continue;
      }
      const double step = distance3D(grid.gridToWorld(current), grid.gridToWorld(next));
      const double candidate = g_cost.at(current) + step;
      const auto previous = g_cost.find(next);
      if (previous == g_cost.end() || candidate < previous->second) {
        parent[next] = current;
        g_cost[next] = candidate;
        const double h = distance3D(grid.gridToWorld(next), grid.gridToWorld(goal));
        open.push({next, enhancedScore(candidate, h, grid, options)});
      }
    }
  }
  return {};
}

}  // namespace bluerov_integration::team_min
