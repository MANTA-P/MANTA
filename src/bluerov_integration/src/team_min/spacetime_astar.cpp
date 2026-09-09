#include "bluerov_integration/team_min/spacetime_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace bluerov_integration::team_min
{
namespace
{

// 격자 인덱스는 출발점을 원점으로 한 상대 좌표다. 절대 좌표를 쓰면
// 매 틱 원점이 바뀌어 인덱스가 커지므로, 창이 로봇을 따라다니게 한다.
struct Index4
{
  int x{0};
  int y{0};
  int z{0};
  int t{0};

  bool operator==(const Index4 & other) const
  {
    return x == other.x && y == other.y && z == other.z && t == other.t;
  }
};

struct Index4Hash
{
  std::size_t operator()(const Index4 & index) const noexcept
  {
    std::size_t hash = static_cast<std::size_t>(index.x) * 73856093U;
    hash ^= static_cast<std::size_t>(index.y) * 19349663U;
    hash ^= static_cast<std::size_t>(index.z) * 83492791U;
    hash ^= static_cast<std::size_t>(index.t) * 2654435761U;
    return hash;
  }
};

struct QueueItem
{
  double score{0.0};
  Index4 index{};

  bool operator>(const QueueItem & other) const
  {
    return score > other.score;
  }
};

// 한 스텝에 갈 수 있는 변위만 이웃으로 삼는다. 이것이 곧 속도 제약이라,
// 기존 방식처럼 계획한 뒤에 "낼 수 있는 속도인가"를 따로 검사할 필요가 없다.
std::vector<std::array<int, 3>> buildMoves(const SpaceTimeOptions & options)
{
  const double horizontal_step = options.max_speed * options.dt;
  const double vertical_step = options.max_vertical_speed * options.dt;
  const int reach_xy =
    static_cast<int>(std::floor(horizontal_step / options.resolution));
  const int reach_z =
    static_cast<int>(std::floor(vertical_step / options.resolution));

  std::vector<std::array<int, 3>> moves;
  for (int dx = -reach_xy; dx <= reach_xy; ++dx) {
    for (int dy = -reach_xy; dy <= reach_xy; ++dy) {
      for (int dz = -reach_z; dz <= reach_z; ++dz) {
        const double horizontal = std::sqrt(
          static_cast<double>(dx * dx + dy * dy)) * options.resolution;
        const double vertical =
          std::abs(static_cast<double>(dz)) * options.resolution;
        if (horizontal <= horizontal_step + 1.0e-9 &&
          vertical <= vertical_step + 1.0e-9)
        {
          // dx=dy=dz=0 은 "제자리 대기"다. 어뢰가 지나갈 때까지 기다렸다
          // 가는 해를 표현하려면 반드시 있어야 한다.
          moves.push_back({dx, dy, dz});
        }
      }
    }
  }
  return moves;
}

bool blockedAt(
  const Point3D & world,
  const double time,
  const std::vector<MovingBox> & obstacles,
  const double margin)
{
  for (const auto & obstacle : obstacles) {
    const Point3D center{
      obstacle.center.x + obstacle.velocity.x * time,
      obstacle.center.y + obstacle.velocity.y * time,
      obstacle.center.z + obstacle.velocity.z * time};
    if (std::abs(world.x - center.x) <= obstacle.half_x + margin &&
      std::abs(world.y - center.y) <= obstacle.half_y + margin &&
      std::abs(world.z - center.z) <= obstacle.half_z + margin)
    {
      return true;
    }
  }
  return false;
}

// 이동의 "회피 방향" 성분. 목표를 향하는 축은 빼고 옆으로 벗어난 양만 본다.
// 목표 방향으로 가는 건 회피가 아니라 전진이므로 관성 대상이 아니다.
Point3D lateralPart(const Point3D & displacement, const Point3D & to_goal)
{
  const double length = std::sqrt(
    to_goal.x * to_goal.x + to_goal.y * to_goal.y + to_goal.z * to_goal.z);
  if (length < 1.0e-9) {
    return displacement;
  }
  const Point3D unit{to_goal.x / length, to_goal.y / length,
    to_goal.z / length};
  const double along = displacement.x * unit.x + displacement.y * unit.y +
    displacement.z * unit.z;
  return {
    displacement.x - unit.x * along,
    displacement.y - unit.y * along,
    displacement.z - unit.z * along};
}

double norm(const Point3D & vector)
{
  return std::sqrt(
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

void validate(const SpaceTimeOptions & options)
{
  if (options.resolution <= 0.0 || options.dt <= 0.0 ||
    options.horizon_sec <= 0.0 || options.max_speed <= 0.0 ||
    options.max_vertical_speed < 0.0 || options.heuristic_weight < 0.0 ||
    options.safety_margin < 0.0 || options.half_extent_xy <= 0.0 ||
    options.half_extent_z <= 0.0 || options.bias_penalty < 0.0)
  {
    throw std::invalid_argument("Space-time A* options are invalid");
  }
  // 분기비율이 2 미만이면 대각선이 잘려 축 이동만 남는다. 그 설정은
  // 측정상 20배 이상 느리고 목표에 못 닿는 경우가 많아 아예 막는다.
  const double ratio = options.max_speed * options.dt / options.resolution;
  if (ratio < 2.0) {
    throw std::invalid_argument(
            "Space-time A* needs max_speed*dt/resolution >= 2 "
            "(diagonal moves get clipped otherwise)");
  }
}

}  // namespace

std::vector<Point3D> stripTimes(const std::vector<TimedPoint> & path)
{
  std::vector<Point3D> points;
  points.reserve(path.size());
  for (const auto & timed : path) {
    points.push_back(timed.point);
  }
  return points;
}

SpaceTimeResult runSpaceTimeAStar3D(
  const Point3D & start,
  const Point3D & local_goal,
  const std::vector<MovingBox> & obstacles,
  const SpaceTimeOptions & options)
{
  validate(options);
  SpaceTimeResult result;

  // 도달 가능성 선검사. 시간창 안에 못 닿는 목표를 주면 A*가 창 전체를
  // 뒤지므로(측정: 172 ms), 탐색 전에 잘라낸다.
  const double reachable =
    options.max_speed * options.horizon_sec * options.reachable_fraction;
  if (distance3D(start, local_goal) > reachable) {
    result.goal_unreachable = true;
    return result;
  }

  const int layers =
    static_cast<int>(std::round(options.horizon_sec / options.dt));
  const double resolution = options.resolution;
  const auto moves = buildMoves(options);

  // 회피 방향 관성 준비. previous_bias가 있으면 그 반대로 가는 이동에
  // 벌점을 매겨, 매 재계획마다 수직<->수평으로 축이 바뀌는 걸 막는다.
  const Point3D to_goal{
    local_goal.x - start.x, local_goal.y - start.y, local_goal.z - start.z};
  const double bias_length = norm(options.previous_bias);
  const bool bias_active =
    bias_length > 1.0e-9 && options.bias_penalty > 0.0;
  const Point3D bias_unit = bias_active ?
    Point3D{options.previous_bias.x / bias_length,
    options.previous_bias.y / bias_length,
    options.previous_bias.z / bias_length} : Point3D{};

  const auto toWorld = [&](const Index4 & index) {
      return Point3D{
      start.x + index.x * resolution,
      start.y + index.y * resolution,
      start.z + index.z * resolution};
    };
  const auto inWindow = [&](const Index4 & index) {
      return std::abs(index.x * resolution) <= options.half_extent_xy &&
             std::abs(index.y * resolution) <= options.half_extent_xy &&
             std::abs(index.z * resolution) <= options.half_extent_z &&
             index.t >= 0 && index.t <= layers;
    };
  // 비용이 시간이므로 휴리스틱도 시간이어야 단위가 맞는다.
  // 남은 거리를 최대속도로 나눈 값은 항상 실제 소요시간 이하라 허용적이다.
  const auto heuristic = [&](const Index4 & index) {
      return distance3D(toWorld(index), local_goal) / options.max_speed;
    };

  // 전형적인 확장 수는 10개 안팎이지만(측정), 최악에는 창 전체가 된다.
  // 미리 잡아두면 그 경우의 재해시 반복을 피한다.
  constexpr std::size_t kReserve = 1024U;
  std::vector<QueueItem> open_storage;
  open_storage.reserve(kReserve);
  std::priority_queue<QueueItem, std::vector<QueueItem>,
    std::greater<QueueItem>> open(std::greater<QueueItem>{},
    std::move(open_storage));
  std::unordered_map<Index4, double, Index4Hash> g_cost;
  std::unordered_map<Index4, Index4, Index4Hash> parent;
  std::unordered_set<Index4, Index4Hash> closed;
  g_cost.reserve(kReserve);
  parent.reserve(kReserve);
  closed.reserve(kReserve);

  const Index4 start_index{0, 0, 0, 0};
  g_cost[start_index] = 0.0;
  open.push({options.heuristic_weight * heuristic(start_index), start_index});

  Index4 reached{};
  bool found = false;

  while (!open.empty()) {
    const Index4 current = open.top().index;
    open.pop();
    if (closed.find(current) != closed.end()) {
      continue;
    }
    closed.insert(current);
    ++result.expanded;

    if (distance3D(toWorld(current), local_goal) <= resolution) {
      reached = current;
      found = true;
      break;
    }
    if (current.t >= layers) {
      continue;   // 시간창을 다 썼다
    }

    // 이웃마다 g_cost.at(current)를 다시 찾지 않도록 한 번만 읽는다.
    const double current_cost = g_cost.at(current);
    const double next_cost = current_cost + options.dt;
    // 이웃은 전부 t+1이라 예측 시각도 하나뿐이다.
    const double next_time = (current.t + 1) * options.dt;

    for (const auto & move : moves) {
      const Index4 next{
        current.x + move[0], current.y + move[1], current.z + move[2],
        current.t + 1};
      if (!inWindow(next) || closed.find(next) != closed.end()) {
        continue;
      }
      if (blockedAt(toWorld(next), next_time, obstacles,
        options.safety_margin))
      {
        continue;
      }

      if (g_cost.find(next) != g_cost.end()) {
        continue;
      }
      // 비용은 경과 시간이다. 대기(제자리)도 dt만큼 비용이 들어
      // 불필요하게 멈추지 않는다.
      //
      // 여기에 회피 방향 관성 벌점을 더한다. 직전 계획이 택한 방향과
      // 반대로 횡이동하면 그만큼 비싸져, 축이 뒤집히지 않는다.
      // 벌점은 g_cost에 넣지 않고 우선순위에만 반영한다 — 시간 비용의
      // 의미(경과 초)를 유지해야 휴리스틱이 허용적이다.
      double penalty = 0.0;
      if (bias_active) {
        const Point3D step{
          static_cast<double>(move[0]) * resolution,
          static_cast<double>(move[1]) * resolution,
          static_cast<double>(move[2]) * resolution};
        const Point3D lateral = lateralPart(step, to_goal);
        const double agreement = lateral.x * bias_unit.x +
          lateral.y * bias_unit.y + lateral.z * bias_unit.z;
        if (agreement < 0.0) {
          penalty = options.bias_penalty * (-agreement) / resolution;
        }
      }
      g_cost.emplace(next, next_cost);
      parent.emplace(next, current);
      open.push(
        {next_cost + penalty + options.heuristic_weight * heuristic(next),
          next});
    }
  }

  if (!found) {
    return result;
  }

  std::vector<TimedPoint> path;
  path.reserve(static_cast<std::size_t>(reached.t) + 1U);
  Index4 node = reached;
  while (!(node == start_index)) {
    path.push_back({toWorld(node), node.t * options.dt});
    node = parent.at(node);
  }
  path.push_back({start, 0.0});
  std::reverse(path.begin(), path.end());
  result.path = std::move(path);
  result.success = true;

  // 이번 경로가 목표선에서 옆으로 벗어난 순 방향을 다음 호출용으로 남긴다.
  const Point3D net{
    result.path.back().point.x - start.x,
    result.path.back().point.y - start.y,
    result.path.back().point.z - start.z};
  Point3D maximum{};
  double maximum_length = 0.0;
  for (const auto & timed : result.path) {
    const Point3D offset{
      timed.point.x - start.x, timed.point.y - start.y,
      timed.point.z - start.z};
    const Point3D lateral = lateralPart(offset, to_goal);
    const double length = norm(lateral);
    if (length > maximum_length) {
      maximum_length = length;
      maximum = lateral;
    }
  }
  (void)net;
  if (maximum_length > 1.0e-6) {
    result.bias = {maximum.x / maximum_length, maximum.y / maximum_length,
      maximum.z / maximum_length};
  }
  return result;
}

}  // namespace bluerov_integration::team_min
