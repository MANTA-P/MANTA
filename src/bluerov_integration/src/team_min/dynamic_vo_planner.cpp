#include "bluerov_integration/team_min/dynamic_vo_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace bluerov_integration::team_min
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGoalWeight = 3.0;
constexpr double kVelocityWeight = 0.35;
constexpr double kClearanceWeight = 1.0;

Point3D add(const Point3D & first, const Point3D & second)
{
  return {first.x + second.x, first.y + second.y, first.z + second.z};
}

Point3D subtract(const Point3D & first, const Point3D & second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Point3D scale(const Point3D & value, const double factor)
{
  return {value.x * factor, value.y * factor, value.z * factor};
}

double dot(const Point3D & first, const Point3D & second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

double length(const Point3D & value)
{
  return std::sqrt(dot(value, value));
}

Point3D normalized(const Point3D & value)
{
  const double magnitude = length(value);
  return magnitude > 1.0e-9 ? scale(value, 1.0 / magnitude) : Point3D{};
}

Point3D clampVelocity(const Point3D & value, const DynamicVOOptions & options)
{
  Point3D result = value;
  const double horizontal = std::hypot(result.x, result.y);
  if (horizontal > options.max_horizontal_speed && horizontal > 1.0e-9) {
    const double factor = options.max_horizontal_speed / horizontal;
    result.x *= factor;
    result.y *= factor;
  }
  result.z = std::clamp(
    result.z, -options.max_vertical_speed, options.max_vertical_speed);
  return result;
}

double obstacleRadius(const BoxObstacle & obstacle)
{
  return 0.5 * std::sqrt(
    obstacle.size_x * obstacle.size_x +
    obstacle.size_y * obstacle.size_y +
    obstacle.size_z * obstacle.size_z);
}

double predictedSeparation(
  const Point3D & robot_position,
  const Point3D & robot_velocity,
  const Point3D & obstacle_position,
  const Point3D & obstacle_velocity,
  const double horizon)
{
  const Point3D relative_position = subtract(obstacle_position, robot_position);
  const Point3D relative_velocity = subtract(obstacle_velocity, robot_velocity);
  const double speed_squared = dot(relative_velocity, relative_velocity);
  double closest_time = 0.0;
  if (speed_squared > 1.0e-9) {
    closest_time = std::clamp(
      -dot(relative_position, relative_velocity) / speed_squared, 0.0, horizon);
  }
  return length(add(relative_position, scale(relative_velocity, closest_time)));
}

bool insideMap(const Point3D & point, const GridMapConfig & map)
{
  return point.x >= map.min_x && point.x <= map.max_x &&
         point.y >= map.min_y && point.y <= map.max_y &&
         point.z >= map.min_z && point.z <= map.max_z;
}

double requiredClearance(
  const MovingObstacle & obstacle,
  const DynamicVOOptions & options)
{
  return options.robot_radius + obstacleRadius(obstacle.shape) + options.safety_margin;
}

bool collisionRisk(
  const Point3D & position,
  const Point3D & velocity,
  const std::vector<MovingObstacle> & obstacles,
  const DynamicVOOptions & options)
{
  for (const auto & obstacle : obstacles) {
    if (predictedSeparation(
        position, velocity, obstacle.shape.center, obstacle.velocity,
        options.prediction_horizon) < requiredClearance(obstacle, options))
    {
      return true;
    }
  }
  return false;
}

std::vector<Point3D> candidateVelocities(
  const Point3D & preferred,
  const Point3D & previous,
  const DynamicVOOptions & options)
{
  std::vector<Point3D> candidates;
  candidates.reserve(static_cast<std::size_t>(
    options.heading_samples * options.vertical_samples * options.speed_samples + 3));
  candidates.push_back({});
  candidates.push_back(clampVelocity(preferred, options));
  candidates.push_back(clampVelocity(previous, options));

  for (int speed_index = 1; speed_index <= options.speed_samples; ++speed_index) {
    const double horizontal_speed = options.max_horizontal_speed *
      static_cast<double>(speed_index) / static_cast<double>(options.speed_samples);
    for (int heading_index = 0; heading_index < options.heading_samples; ++heading_index) {
      const double heading = 2.0 * kPi * static_cast<double>(heading_index) /
        static_cast<double>(options.heading_samples);
      for (int vertical_index = 0; vertical_index < options.vertical_samples; ++vertical_index) {
        const double vertical_ratio = options.vertical_samples == 1 ? 0.0 :
          -1.0 + 2.0 * static_cast<double>(vertical_index) /
          static_cast<double>(options.vertical_samples - 1);
        candidates.push_back({
          horizontal_speed * std::cos(heading),
          horizontal_speed * std::sin(heading),
          options.max_vertical_speed * vertical_ratio});
      }
    }
  }
  return candidates;
}

std::optional<Point3D> chooseVelocity(
  const Point3D & position,
  const Point3D & goal,
  const Point3D & previous_velocity,
  const std::vector<MovingObstacle> & obstacles,
  const GridMapConfig & map,
  const DynamicVOOptions & options)
{
  const Point3D to_goal = subtract(goal, position);
  Point3D preferred = scale(normalized(to_goal), options.max_horizontal_speed);
  preferred.z = std::clamp(
    to_goal.z / options.rollout_step,
    -options.max_vertical_speed, options.max_vertical_speed);
  preferred = clampVelocity(preferred, options);

  double best_score = std::numeric_limits<double>::infinity();
  std::optional<Point3D> best;
  for (const Point3D & candidate :
    candidateVelocities(preferred, previous_velocity, options))
  {
    const Point3D next_position = add(position, scale(candidate, options.rollout_step));
    if (!insideMap(next_position, map) ||
      collisionRisk(position, candidate, obstacles, options))
    {
      continue;
    }

    double minimum_margin = std::numeric_limits<double>::infinity();
    for (const auto & obstacle : obstacles) {
      const double separation = predictedSeparation(
        position, candidate, obstacle.shape.center, obstacle.velocity,
        options.prediction_horizon);
      minimum_margin = std::min(
        minimum_margin, separation - requiredClearance(obstacle, options));
    }
    const double goal_cost = distance3D(next_position, goal);
    const double velocity_cost =
      length(subtract(candidate, preferred)) +
      0.5 * length(subtract(candidate, previous_velocity));
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
  const Point3D & position,
  const Point3D & velocity,
  const Point3D & goal,
  const GridMapConfig & map,
  const std::vector<MovingObstacle> & obstacles,
  const DynamicVOOptions & options)
{
  const auto finitePoint = [](const Point3D & point) {
      return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
  if (!finitePoint(position) || !finitePoint(velocity) || !finitePoint(goal) ||
    !insideMap(position, map) || !insideMap(goal, map) ||
    options.prediction_horizon <= 0.0 || options.rollout_step <= 0.0 ||
    options.rollout_steps == 0U || options.max_horizontal_speed <= 0.0 ||
    options.max_vertical_speed <= 0.0 || options.robot_radius < 0.0 ||
    options.safety_margin < 0.0 || options.goal_tolerance < 0.0 ||
    options.path_lookahead <= 0.0 ||
    options.heading_samples <= 0 || options.vertical_samples <= 0 ||
    options.speed_samples <= 0)
  {
    throw std::invalid_argument("Invalid Dynamic VO input");
  }
  for (const auto & obstacle : obstacles) {
    if (!finitePoint(obstacle.shape.center) || !finitePoint(obstacle.velocity) ||
      obstacle.shape.size_x <= 0.0 || obstacle.shape.size_y <= 0.0 ||
      obstacle.shape.size_z <= 0.0)
    {
      throw std::invalid_argument("Invalid Dynamic VO obstacle");
    }
  }
}

}  // namespace

// Dynamic VO: relative motion includes obstacles approaching from behind.
DynamicVOResult runDynamicVO3D(
  const Point3D & robot_position,
  const Point3D & robot_velocity,
  const Point3D & local_goal,
  const GridMapConfig & map_config,
  const std::vector<MovingObstacle> & obstacles,
  const DynamicVOOptions & options)
{
  validateInput(
    robot_position, robot_velocity, local_goal, map_config, obstacles, options);

  DynamicVOResult result;
  result.avoidance_required =
    collisionRisk(robot_position, robot_velocity, obstacles, options);
  // 단독 모드에서는 위험이 없어도 목표까지의 경로를 직접 만들어야 한다
  // (하이브리드는 위험이 없으면 A* 경로를 그대로 쓰므로 빈 경로 반환).
  if (!result.avoidance_required && !options.standalone) {
    result.success = true;
    return result;
  }

  result.local_path.reserve(options.rollout_steps + 1U);
  result.local_path.push_back(robot_position);
  Point3D position = robot_position;
  Point3D velocity = robot_velocity;
  for (std::size_t step = 0; step < options.rollout_steps; ++step) {
    if (distance3D(position, local_goal) <= options.goal_tolerance) {
      result.local_path.push_back(local_goal);
      result.success = true;
      return result;
    }
    // Dynamic VO: advance obstacles consistently with the local rollout time.
    std::vector<MovingObstacle> predicted_obstacles = obstacles;
    const double elapsed = static_cast<double>(step) * options.rollout_step;
    for (auto & obstacle : predicted_obstacles) {
      obstacle.shape.center = add(
        obstacle.shape.center, scale(obstacle.velocity, elapsed));
    }
    const auto selected = chooseVelocity(
      position, local_goal, velocity, predicted_obstacles, map_config, options);
    if (!selected) {
      return result;
    }
    velocity = *selected;
    const Point3D next = add(position, scale(velocity, options.rollout_step));
    if (distance3D(position, next) <= 1.0e-9) {
      return result;
    }
    position = next;
    result.local_path.push_back(position);
    if (step == 0U) {
      result.selected_velocity = velocity;
    }
  }
  // Dynamic VO: only reconnect to A* after reaching its selected waypoint.
  if (distance3D(position, local_goal) <= options.goal_tolerance) {
    if (distance3D(position, local_goal) > 1.0e-9) {
      result.local_path.push_back(local_goal);
    }
    result.success = true;
  } else if (options.standalone) {
    // 단독 모드는 먼 목표를 한 번에 못 간다(20스텝 x 0.5s x 3 m/s ~= 30 m).
    // 다음 틱에 다시 뽑으므로 부분 경로도 유효한 결과다.
    result.success = true;
  }
  return result;
}

}  // namespace bluerov_integration::team_min
