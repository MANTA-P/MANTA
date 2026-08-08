// 3-D Dynamic Velocity Obstacle path planner.
//
// Deliberately implemented as a pure planner translation unit (no ROS node).
// It accepts the same inputs and returns the same path type as
// runEnhancedAStar3D(). Obstacle velocity is estimated across calls from the
// moving BoxObstacle centers, allowing the existing PlanningModule to invoke
// A* and DVO in parallel without changing its data acquisition architecture.

#include "bluerov_integration/team_min/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>

namespace bluerov_integration::team_min
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Change these defaults later through a public options struct if runtime
// configuration is required. Keeping them here preserves the A* call shape.
constexpr double kUpdatePeriodFallback = 0.2;
constexpr double kPredictionHorizon = 6.0; //예측시간[s]
constexpr double kRolloutStep = 0.5;
constexpr std::size_t kRolloutSteps = 40U; //최대 생성 단계
constexpr double kMaxHorizontalSpeed = 3.0; //uuv의 수평최대속도[m/s]
constexpr double kMaxVerticalSpeed = 1.5; //uuv의 수직최대속도[m/s]
constexpr double kRobotRadius = 0.75; //uuv의 반경[m]
constexpr double kSafetyMargin = 0.50; //기본 안전거리[m]
constexpr int kHeadingSamples = 36; //수평방향 후보
constexpr int kVerticalSamples = 7; //수직 후보
constexpr int kSpeedSamples = 5; //속력 후보
constexpr double kGoalWeight = 3.0;
constexpr double kVelocityWeight = 0.35;
constexpr double kClearanceWeight = 1.0;
constexpr double kGoalTolerance = 0.75; //목표 도착 거리[m]
constexpr double kVelocityFilterAlpha = 0.45; //장애물 속도 필터 계수 
constexpr double kMaximumEstimatorDt = 2.0;

struct Velocity3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Velocity3D operator+(const Velocity3D & first, const Velocity3D & second)
{
  return {first.x + second.x, first.y + second.y, first.z + second.z};
}

Velocity3D operator-(const Velocity3D & first, const Velocity3D & second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Velocity3D operator*(const Velocity3D & value, const double scale)
{
  return {value.x * scale, value.y * scale, value.z * scale};
}

Point3D operator+(const Point3D & point, const Velocity3D & offset)
{
  return {point.x + offset.x, point.y + offset.y, point.z + offset.z};
}

Velocity3D difference(const Point3D & first, const Point3D & second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

double dot(const Velocity3D & first, const Velocity3D & second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

double length(const Velocity3D & value)
{
  return std::sqrt(dot(value, value));
}

Velocity3D normalized(const Velocity3D & value)
{
  const double magnitude = length(value);
  return magnitude > 1e-9 ? value * (1.0 / magnitude) : Velocity3D{};
}

Velocity3D clampVelocity(const Velocity3D & value)
{
  Velocity3D result = value;
  const double horizontal = std::hypot(result.x, result.y);
  if (horizontal > kMaxHorizontalSpeed && horizontal > 1e-9) {
    const double scale = kMaxHorizontalSpeed / horizontal;
    result.x *= scale;
    result.y *= scale;
  }
  result.z = std::clamp(result.z, -kMaxVerticalSpeed, kMaxVerticalSpeed);
  return result;
}

double obstacleRadius(const BoxObstacle & obstacle)
{
  // A bounding sphere safely contains the complete axis-aligned box.
  return 0.5 * std::sqrt(
    obstacle.size_x * obstacle.size_x +
    obstacle.size_y * obstacle.size_y +
    obstacle.size_z * obstacle.size_z);
}

struct ObstacleMotion
{
  BoxObstacle obstacle;
  Velocity3D velocity;
};

struct EstimatorState
{
  std::vector<Point3D> previous_centers;
  std::vector<Velocity3D> filtered_velocities;
  std::chrono::steady_clock::time_point previous_time{};
  bool initialized{false};
};

std::mutex estimator_mutex;
EstimatorState estimator;

std::vector<ObstacleMotion> estimateObstacleMotion(
  const std::vector<BoxObstacle> & obstacles)
{
  const auto current_time = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(estimator_mutex);

  double dt = kUpdatePeriodFallback;
  if (estimator.initialized) {
    dt = std::chrono::duration<double>(current_time - estimator.previous_time).count();
  }
  const bool usable_history = estimator.initialized &&
    estimator.previous_centers.size() == obstacles.size() &&
    estimator.filtered_velocities.size() == obstacles.size() &&
    dt > 1e-4 && dt <= kMaximumEstimatorDt;

  std::vector<ObstacleMotion> result;
  result.reserve(obstacles.size());
  std::vector<Point3D> new_centers;
  std::vector<Velocity3D> new_velocities;
  new_centers.reserve(obstacles.size());
  new_velocities.reserve(obstacles.size());

  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    Velocity3D velocity{};
    if (usable_history) {
      const Velocity3D measured =
        difference(obstacles[index].center, estimator.previous_centers[index]) * (1.0 / dt);
      velocity = estimator.filtered_velocities[index] * (1.0 - kVelocityFilterAlpha) +
        measured * kVelocityFilterAlpha;
    }
    result.push_back({obstacles[index], velocity});
    new_centers.push_back(obstacles[index].center);
    new_velocities.push_back(velocity);
  }

  estimator.previous_centers = std::move(new_centers);
  estimator.filtered_velocities = std::move(new_velocities);
  estimator.previous_time = current_time;
  estimator.initialized = true;
  return result;
}

double predictedSeparation(
  const Point3D & robot_position,
  const Velocity3D & robot_velocity,
  const Point3D & obstacle_position,
  const Velocity3D & obstacle_velocity)
{
  const Velocity3D relative_position = difference(obstacle_position, robot_position);
  const Velocity3D relative_velocity = obstacle_velocity - robot_velocity;
  const double speed_squared = dot(relative_velocity, relative_velocity);
  double closest_time = 0.0;
  if (speed_squared > 1e-9) {
    closest_time = std::clamp(
      -dot(relative_position, relative_velocity) / speed_squared,
      0.0, kPredictionHorizon);
  }
  return length(relative_position + relative_velocity * closest_time);
}

bool insideMap(const Point3D & point, const GridMapConfig & map)
{
  return point.x >= map.min_x && point.x <= map.max_x &&
         point.y >= map.min_y && point.y <= map.max_y &&
         point.z >= map.min_z && point.z <= map.max_z;
}

std::vector<Velocity3D> candidateVelocities(
  const Velocity3D & preferred,
  const Velocity3D & previous)
{
  std::vector<Velocity3D> candidates;
  candidates.reserve(static_cast<std::size_t>(
    kHeadingSamples * kVerticalSamples * kSpeedSamples + 3));
  candidates.push_back({});
  candidates.push_back(clampVelocity(preferred));
  candidates.push_back(clampVelocity(previous));

  for (int speed_index = 1; speed_index <= kSpeedSamples; ++speed_index) {
    const double horizontal_speed = kMaxHorizontalSpeed *
      static_cast<double>(speed_index) / static_cast<double>(kSpeedSamples);
    for (int heading_index = 0; heading_index < kHeadingSamples; ++heading_index) {
      const double heading = 2.0 * kPi * static_cast<double>(heading_index) /
        static_cast<double>(kHeadingSamples);
      for (int vertical_index = 0; vertical_index < kVerticalSamples; ++vertical_index) {
        const double vertical_ratio = kVerticalSamples == 1 ? 0.0 :
          -1.0 + 2.0 * static_cast<double>(vertical_index) /
          static_cast<double>(kVerticalSamples - 1);
        candidates.push_back({
          horizontal_speed * std::cos(heading),
          horizontal_speed * std::sin(heading),
          kMaxVerticalSpeed * vertical_ratio});
      }
    }
  }
  return candidates;
}

std::optional<Velocity3D> chooseVelocity(
  const Point3D & position,
  const Point3D & goal,
  const Velocity3D & previous_velocity,
  const std::vector<ObstacleMotion> & obstacles,
  const GridMapConfig & map,
  const double elapsed,
  const double requested_safety_margin)
{
  const Velocity3D to_goal = difference(goal, position);
  Velocity3D preferred = normalized(to_goal) * kMaxHorizontalSpeed;
  preferred.z = std::clamp(to_goal.z / kRolloutStep, -kMaxVerticalSpeed, kMaxVerticalSpeed);
  preferred = clampVelocity(preferred);

  double best_score = std::numeric_limits<double>::infinity();
  std::optional<Velocity3D> best;
  for (const Velocity3D & candidate : candidateVelocities(preferred, previous_velocity)) {
    const Point3D next_position = position + candidate * kRolloutStep;
    if (!insideMap(next_position, map)) {
      continue;
    }

    bool safe = true;
    double minimum_margin = std::numeric_limits<double>::infinity();
    for (const auto & moving_obstacle : obstacles) {
      const Point3D predicted_center =
        moving_obstacle.obstacle.center + moving_obstacle.velocity * elapsed;
      const double required_clearance =
        kRobotRadius + obstacleRadius(moving_obstacle.obstacle) +
        kSafetyMargin + requested_safety_margin;
      const double separation = predictedSeparation(
        position, candidate, predicted_center, moving_obstacle.velocity);
      if (separation < required_clearance) {
        safe = false;
        break;
      }
      minimum_margin = std::min(minimum_margin, separation - required_clearance);
    }
    if (!safe) {
      continue;
    }

    const double goal_cost = distance3D(next_position, goal);
    const double velocity_cost =
      length(candidate - preferred) + 0.5 * length(candidate - previous_velocity);
    const double clearance_cost = obstacles.empty() ? 0.0 :
      1.0 / std::max(minimum_margin, 0.05);
    const double score = kGoalWeight * goal_cost +
      kVelocityWeight * velocity_cost + kClearanceWeight * clearance_cost;
    if (score < best_score) {
      best_score = score;
      best = candidate;
    }
  }
  return best;
}

void validateInput(
  const Point3D & start,
  const Point3D & goal,
  const GridMapConfig & map,
  const std::vector<BoxObstacle> & obstacles,
  const AStarOptions & options)
{
  const auto finitePoint = [](const Point3D & point) {
      return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
  if (!finitePoint(start) || !finitePoint(goal) ||
    !std::isfinite(map.min_x) || !std::isfinite(map.max_x) ||
    !std::isfinite(map.min_y) || !std::isfinite(map.max_y) ||
    !std::isfinite(map.min_z) || !std::isfinite(map.max_z) ||
    map.max_x <= map.min_x || map.max_y <= map.min_y || map.max_z <= map.min_z ||
    !insideMap(start, map) || !insideMap(goal, map) || options.safety_margin < 0.0)
  {
    throw std::invalid_argument("Invalid Dynamic VO input");
  }
  for (const auto & obstacle : obstacles) {
    if (!finitePoint(obstacle.center) || obstacle.size_x <= 0.0 ||
      obstacle.size_y <= 0.0 || obstacle.size_z <= 0.0)
    {
      throw std::invalid_argument("Invalid Dynamic VO obstacle");
    }
  }
}

}  // namespace

// Intentionally mirrors runEnhancedAStar3D() so the same PlanningRequest data
// can be dispatched to both algorithms. The AStarOptions safety_margin is
// added to DVO's internal clearance margin; the other DVO defaults are listed
// above until a dedicated public options type is introduced.
std::vector<Point3D> runDynamicVO3D(
  const Point3D & start_world,
  const Point3D & goal_world,
  const GridMapConfig & map_config,
  const std::vector<BoxObstacle> & obstacles,
  const AStarOptions & options)
{
  validateInput(start_world, goal_world, map_config, obstacles, options);
  if (distance3D(start_world, goal_world) <= kGoalTolerance) {
    return {start_world, goal_world};
  }

  const std::vector<ObstacleMotion> moving_obstacles = estimateObstacleMotion(obstacles);
  std::vector<Point3D> path;
  path.reserve(kRolloutSteps + 1U);
  path.push_back(start_world);

  Point3D position = start_world;
  Velocity3D velocity{};
  for (std::size_t step = 0; step < kRolloutSteps; ++step) {
    if (distance3D(position, goal_world) <= kGoalTolerance) {
      if (distance3D(path.back(), goal_world) > 1e-9) {
        path.push_back(goal_world);
      }
      return path;
    }

    const double elapsed = static_cast<double>(step) * kRolloutStep;
    const auto selected = chooseVelocity(
      position, goal_world, velocity, moving_obstacles, map_config, elapsed,
      options.safety_margin);
    if (!selected) {
      return {};
    }
    velocity = *selected;
    Point3D next = position + velocity * kRolloutStep;
    if (distance3D(position, next) >= distance3D(position, goal_world)) {
      next = goal_world;
    }
    if (distance3D(position, next) <= 1e-9) {
      return {};
    }
    position = next;
    path.push_back(position);
  }
  return path;
}

}  // namespace bluerov_integration::team_min
