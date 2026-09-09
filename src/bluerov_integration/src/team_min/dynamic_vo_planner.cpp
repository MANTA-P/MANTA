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
// 가중치는 DynamicVOOptions 로 옮겼다(국면별로 바꿔야 하므로).
// 기본값이 예전 상수와 같아 평상시 동작은 그대로다.

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

// 시선(LOS)에 수직인 속도 성분이 클수록 낮은 비용.
//
// 비례항법의 소요 횡가속 a = N x Vc x lambda_dot 에서, 회피자가 키울 수
// 있는 것은 시선각속도 lambda_dot 뿐이다. 후보 속도에서 시선 성분을
// 빼면 남는 것이 시선각속도를 만드는 성분이다. 거리로 나누지 않는 이유는
// 급기동 국면 자체가 이미 근거리로 한정돼 있고, 나누면 후보 간 비교가
// 거리에 휘둘려 불안정해지기 때문이다.
//
// 최대속도로 정규화해 [0, 1] 로 만든다. 완전 횡기동이면 0, 시선 방향
// (접근이든 도주든)이면 1 이다.
double beamCost(
  const Point3D & position,
  const Point3D & candidate,
  const Point3D & previous_velocity,
  const std::vector<MovingObstacle> & obstacles,
  const DynamicVOOptions & options)
{
  // 가장 가까운 장애물을 위협으로 본다. 통로 박스 중 선두가 어뢰의
  // 현재 위치이므로, 최근접 박스가 곧 시선의 기준점이다.
  const MovingObstacle * threat = nullptr;
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : obstacles) {
    const double range = length(subtract(obstacle.shape.center, position));
    if (range < nearest) {
      nearest = range;
      threat = &obstacle;
    }
  }
  if (threat == nullptr || nearest < 1.0e-6) {
    return 0.0;
  }
  const Point3D los = normalized(subtract(threat->shape.center, position));
  const Point3D perpendicular =
    subtract(candidate, scale(los, dot(candidate, los)));
  const double reference = std::max(options.max_horizontal_speed, 1.0e-9);

  // 평상시 급기동: 시선에 수직인 성분이 크기만 하면 된다(방향 무관).
  if (!options.break_reverse) {
    return 1.0 - std::clamp(length(perpendicular) / reference, 0.0, 1.0);
  }

  // 반전(jink): 직전에 가던 횡방향의 '반대'로 갈수록 좋다.
  //
  // 한 방향으로만 계속 피하면 비례항법이 그 해를 수렴시킨다. 뒤집으면
  // 어뢰가 따라잡아야 할 횡속도 변화가 두 배(3.0 -> 6.0 m/s)가 된다.
  const Point3D previous_perpendicular = subtract(
    previous_velocity, scale(los, dot(previous_velocity, los)));
  const double previous_lateral = length(previous_perpendicular);
  if (previous_lateral < 0.5) {
    // 아직 횡방향으로 가고 있지 않다. 뒤집을 것이 없으므로 평상시와 같다.
    return 1.0 - std::clamp(length(perpendicular) / reference, 0.0, 1.0);
  }
  const Point3D previous_direction =
    scale(previous_perpendicular, 1.0 / previous_lateral);
  // 반대 방향이면 양수. 같은 방향이면 음수라 비용이 1 을 넘어 탈락한다.
  const double opposing = -dot(perpendicular, previous_direction);
  return 1.0 - std::clamp(opposing / reference, -1.0, 1.0);
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

  const std::vector<Point3D> candidates =
    candidateVelocities(preferred, previous_velocity, options);
  const DynamicVOWeights & weights =
    options.break_mode ? options.break_weights : options.weights;

  // 후보의 최소 여유(예측 이격 - 필요 이격). 음수면 충돌 예측이다.
  const auto marginOf = [&](const Point3D & candidate) {
      double minimum = std::numeric_limits<double>::infinity();
      for (const auto & obstacle : obstacles) {
        const double separation = predictedSeparation(
          position, candidate, obstacle.shape.center, obstacle.velocity,
          options.prediction_horizon);
        minimum = std::min(
          minimum, separation - requiredClearance(obstacle, options));
      }
      return minimum;
    };

  // 1차: 충돌이 예측되지 않는 후보 중 최선.
  double best_score = std::numeric_limits<double>::infinity();
  std::optional<Point3D> best;
  for (const Point3D & candidate : candidates) {
    const Point3D next_position =
      add(position, scale(candidate, options.rollout_step));
    if (!insideMap(next_position, map) ||
      collisionRisk(position, candidate, obstacles, options))
    {
      continue;
    }

    const double minimum_margin = marginOf(candidate);
    const double goal_cost = distance3D(next_position, goal);
    const double preferred_cost = length(subtract(candidate, preferred));
    const double continuity_cost =
      length(subtract(candidate, previous_velocity));
    const double clearance_cost = obstacles.empty() ? 0.0 :
      1.0 / std::max(minimum_margin, 0.05);
    // beam 이 0 이면(평상시) 계산 자체를 건너뛴다. 후보마다 도는
    // 루프라 불필요한 비용을 넣지 않는다.
    const double beam_cost = weights.beam == 0.0 ? 0.0 :
      beamCost(position, candidate, previous_velocity, obstacles, options);
    const double score = weights.goal * goal_cost +
      weights.preferred * preferred_cost +
      weights.continuity * continuity_cost +
      weights.clearance * clearance_cost +
      weights.beam * beam_cost;
    if (score < best_score) {
      best_score = score;
      best = candidate;
    }
  }
  if (best) {
    return best;
  }

  // 2차: 충돌을 피하는 후보가 하나도 없을 때.
  //
  // collisionRisk 는 예측 구간(prediction_horizon 6초) 안에 충돌하면
  // 후보를 버린다. 그런데 TTC 가 1~2초로 줄면 어떤 속도를 골라도 6초
  // 안에 충돌이 예측되므로 후보가 전멸한다. 그러면 호출자가 경로를
  // 못 만들어 ROV 를 정지시키는데, 유도어뢰 앞에서 정지는 가능한
  // 최악의 선택이다. 실측(hybrid 9판): 종말 구간에서 계획이 거의 못
  // 돈 4판은 전부 피격, 계획이 돈 5판은 전부 회피였다.
  //
  // 그래서 포기하지 않고 "가장 덜 나쁜" 속도를 고른다. 목표·선호·연속성
  // 항은 버린다 — 충돌이 확실한 상황에서 목표로 가려는 힘은 해롭다.
  // 남길 것은 여유(음수라도 큰 쪽이 낫다)와, 급기동이면 시선각속도다.
  double least_bad = -std::numeric_limits<double>::infinity();
  for (const Point3D & candidate : candidates) {
    const Point3D next_position =
      add(position, scale(candidate, options.rollout_step));
    if (!insideMap(next_position, map)) {
      continue;   // 맵 밖은 여전히 안 된다
    }
    // 여유는 클수록, 시선각속도는 클수록 좋다. beamCost 는 완전
    // 횡기동이 0, 시선 방향이 1 이므로 부호를 뒤집어 더한다.
    double utility = marginOf(candidate);
    if (weights.beam > 0.0) {
      utility += weights.beam *
        (1.0 - beamCost(
          position, candidate, previous_velocity, obstacles, options));
    }
    if (utility > least_bad) {
      least_bad = utility;
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
