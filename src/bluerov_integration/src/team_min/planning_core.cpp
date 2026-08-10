#include "bluerov_integration/team_min/planning_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bluerov_integration::team_min
{

bool framesCompatible(const std::string & first, const std::string & second)
{
  return first.empty() || second.empty() || first == second;
}

bool boxContains(
  const BoxObstacle & box,
  const Point3D & point,
  const double margin)
{
  return std::abs(point.x - box.center.x) <= box.size_x * 0.5 + margin &&
         std::abs(point.y - box.center.y) <= box.size_y * 0.5 + margin &&
         std::abs(point.z - box.center.z) <= box.size_z * 0.5 + margin;
}

bool sampleFresh(
  const VehicleState & sample,
  const double now_sec,
  const double timeout_sec)
{
  if (!sample.valid) {
    return false;
  }
  const double age_sec = now_sec - sample.received_sec;
  return age_sec >= 0.0 && age_sec <= timeout_sec;
}

// 어뢰의 진행방향 앞에 겹치는 예측 박스를 깔아 통로(corridor)를 만든다.
// A*가 미래 궤적 전체를 한 번에 우회하므로 어뢰가 예측대로 움직이는 동안
// 경로가 흔들리지 않는다. obstacles[0]은 항상 현재 위치 박스다.
std::vector<BoxObstacle> buildTorpedoObstacles(
  const PlanRequest & request,
  const PlanningCoreConfig & config)
{
  std::vector<BoxObstacle> obstacles;
  if (!request.torpedo_valid) {
    return obstacles;  // NORMAL 모드: 장애물 없이 계획한다.
  }
  BoxObstacle current = config.torpedo_barrier;
  current.center = request.torpedo;
  obstacles.push_back(current);

  const PredictionConfig & prediction = config.prediction;
  const double speed = distance3D(request.torpedo_velocity, Point3D{});
  // min_speed를 0으로 낮춰도 0 나누기가 없도록 하한을 둔다.
  if (!prediction.enabled || speed < prediction.min_speed || speed < 1.0e-6) {
    return obstacles;
  }

  const Point3D unit{
    request.torpedo_velocity.x / speed,
    request.torpedo_velocity.y / speed,
    request.torpedo_velocity.z / speed};
  const double travel = std::min(
    speed * prediction.horizon_sec,
    prediction.spacing * static_cast<double>(prediction.max_boxes));
  const int count = std::min(
    prediction.max_boxes,
    static_cast<int>(std::ceil(travel / prediction.spacing)));
  const double guard_margin =
    config.astar.safety_margin + prediction.start_clearance;
  for (int i = 1; i <= count; ++i) {
    BoxObstacle box = config.torpedo_barrier;
    const double offset = prediction.spacing * static_cast<double>(i);
    box.center = {
      request.torpedo.x + unit.x * offset,
      request.torpedo.y + unit.y * offset,
      request.torpedo.z + unit.z * offset};
    // 예측 박스가 로봇이나 목표를 덮으면 A*가 start/goal inside obstacle로
    // 실패해 경로 없음(추력 0 정지)이 되므로 그 박스는 건너뛴다.
    if (boxContains(box, request.start, guard_margin) ||
      boxContains(box, request.goal, guard_margin))
    {
      continue;
    }
    obstacles.push_back(box);
  }
  return obstacles;
}

GridMapConfig planningMap(
  const PlanRequest & request,
  const std::vector<BoxObstacle> & obstacles,
  const PlanningCoreConfig & config)
{
  if (!config.use_dynamic_map) {
    return config.fixed_map;
  }

  GridMapConfig map = config.fixed_map;
  double min_x = std::min(request.start.x, request.goal.x);
  double max_x = std::max(request.start.x, request.goal.x);
  double min_y = std::min(request.start.y, request.goal.y);
  double max_y = std::max(request.start.y, request.goal.y);
  double min_z = std::min(request.start.z, request.goal.z);
  double max_z = std::max(request.start.z, request.goal.z);
  // 예측 통로 박스가 맵 경계에서 잘려 못 막는 일이 없도록 범위에 포함한다.
  for (const auto & obstacle : obstacles) {
    const double half_x = obstacle.size_x * 0.5 + config.astar.safety_margin;
    const double half_y = obstacle.size_y * 0.5 + config.astar.safety_margin;
    const double half_z = obstacle.size_z * 0.5 + config.astar.safety_margin;
    min_x = std::min(min_x, obstacle.center.x - half_x);
    max_x = std::max(max_x, obstacle.center.x + half_x);
    min_y = std::min(min_y, obstacle.center.y - half_y);
    max_y = std::max(max_y, obstacle.center.y + half_y);
    min_z = std::min(min_z, obstacle.center.z - half_z);
    max_z = std::max(max_z, obstacle.center.z + half_z);
  }
  map.min_x = min_x - config.map_padding_x;
  map.max_x = max_x + config.map_padding_x;
  map.min_y = min_y - config.map_padding_y;
  map.max_y = max_y + config.map_padding_y;
  map.min_z = min_z - config.map_padding_z;
  map.max_z = max_z + config.map_padding_z;
  return map;
}

// Dynamic VO: choose a forward A* waypoint instead of the mission goal.
LocalTarget selectLocalTarget(
  const Point3D & position,
  const std::vector<Point3D> & path,
  const double lookahead)
{
  std::size_t closest = 0U;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double candidate = distance3D(position, path[index]);
    if (candidate < closest_distance) {
      closest_distance = candidate;
      closest = index;
    }
  }
  double accumulated = 0.0;
  std::size_t target = closest;
  while (target + 1U < path.size() && accumulated < lookahead) {
    accumulated += distance3D(path[target], path[target + 1U]);
    ++target;
  }
  return {path[target], target};
}

// Dynamic VO: reconnect the local avoidance path to the untouched A* suffix.
std::vector<Point3D> mergePaths(
  std::vector<Point3D> local_path,
  const std::vector<Point3D> & global_path,
  const std::size_t reconnect_index)
{
  if (local_path.empty()) {
    return global_path;
  }
  std::size_t suffix = reconnect_index;
  if (suffix < global_path.size() &&
    distance3D(local_path.back(), global_path[suffix]) <= 1.0e-6)
  {
    ++suffix;
  }
  local_path.insert(local_path.end(), global_path.begin() + suffix, global_path.end());
  return local_path;
}

PlanningCore::PlanningCore(PlanningCoreConfig config)
: config_(std::move(config))
{
  if (!config_.use_target_topic_for_goal &&
    config_.goal_offset_x == 0.0 && config_.goal_offset_y == 0.0 &&
    config_.goal_offset_z == 0.0)
  {
    throw std::invalid_argument("Planning goal offset cannot be all zero");
  }
  if (config_.robot_replan_distance < 0.0 ||
    config_.torpedo_replan_distance < 0.0 ||
    config_.goal_replan_distance < 0.0)
  {
    throw std::invalid_argument("Planning replan distances must be non-negative");
  }
  const auto & prediction = config_.prediction;
  if (prediction.spacing <= 0.0 || prediction.horizon_sec < 0.0 ||
    prediction.max_boxes < 0 || prediction.min_speed < 0.0 ||
    prediction.velocity_alpha <= 0.0 || prediction.velocity_alpha > 1.0 ||
    prediction.start_clearance < 0.0)
  {
    throw std::invalid_argument("Planning prediction parameters are invalid");
  }
  if (config_.replan.path_deviation_distance <= 0.0 ||
    config_.replan.collision_margin < 0.0 ||
    config_.replan.min_interval_sec < 0.0)
  {
    throw std::invalid_argument("Planning replan policy parameters are invalid");
  }
  if (config_.avoid.torpedo_timeout_sec <= 0.0 ||
    config_.avoid.hit_radius < 0.0 ||
    config_.avoid.engage_radius <= config_.avoid.hit_radius)
  {
    throw std::invalid_argument("Planning avoid parameters are invalid");
  }
}

// 5Hz 틱 사이(12 m/s 어뢰면 2.4 m 이동)를 등속으로 보간해 두 이동점의
// 최근접 거리를 닫힌형으로 구한다. 순간 거리만 보면 틱 사이에 스쳐
// 지나가는 피격을 놓친다.
double PlanningCore::closestApproachDistance(
  const Point3D & rov_previous,
  const Point3D & rov_current,
  const Point3D & torpedo_previous,
  const Point3D & torpedo_current)
{
  const Point3D relative_start{
    torpedo_previous.x - rov_previous.x,
    torpedo_previous.y - rov_previous.y,
    torpedo_previous.z - rov_previous.z};
  const Point3D relative_delta{
    (torpedo_current.x - rov_current.x) - relative_start.x,
    (torpedo_current.y - rov_current.y) - relative_start.y,
    (torpedo_current.z - rov_current.z) - relative_start.z};
  const double delta_squared =
    relative_delta.x * relative_delta.x +
    relative_delta.y * relative_delta.y +
    relative_delta.z * relative_delta.z;
  double t = 0.0;
  if (delta_squared > 1.0e-9) {
    const double projection =
      relative_start.x * relative_delta.x +
      relative_start.y * relative_delta.y +
      relative_start.z * relative_delta.z;
    t = std::clamp(-projection / delta_squared, 0.0, 1.0);
  }
  const Point3D closest{
    relative_start.x + relative_delta.x * t,
    relative_start.y + relative_delta.y * t,
    relative_start.z + relative_delta.z * t};
  return distance3D(closest, Point3D{});
}

// 어뢰 twist는 body 프레임일 수 있어 map 프레임 위치 차분으로 속도를
// 추정한다. dt는 메시지 스탬프(sim time) 차분 우선, 스탬프가 없거나
// 역행하면 수신 시각 차분으로 폴백한다(RTF != 1 대응).
void PlanningCore::updateTorpedoVelocity(const VehicleState & torpedo)
{
  if (!torpedo.valid) {
    return;
  }
  if (have_torpedo_history_ && torpedo.sequence == last_torpedo_sequence_) {
    return;  // planning 타이머가 같은 snapshot을 다시 본 경우
  }

  if (have_torpedo_history_ && torpedo.frame_id == last_torpedo_frame_) {
    double dt = torpedo.received_sec - last_torpedo_received_sec_;
    if (torpedo.stamp_sec > 0.0 && last_torpedo_stamp_sec_ > 0.0) {
      const double message_dt = torpedo.stamp_sec - last_torpedo_stamp_sec_;
      if (message_dt > 0.0) {
        dt = message_dt;
      }
    }
    if (dt > 1.0e-3 && dt < 2.0) {
      const Point3D raw{
        (torpedo.position.x - last_torpedo_position_.x) / dt,
        (torpedo.position.y - last_torpedo_position_.y) / dt,
        (torpedo.position.z - last_torpedo_position_.z) / dt};
      const double alpha = config_.prediction.velocity_alpha;
      if (have_torpedo_velocity_) {
        torpedo_velocity_ = {
          alpha * raw.x + (1.0 - alpha) * torpedo_velocity_.x,
          alpha * raw.y + (1.0 - alpha) * torpedo_velocity_.y,
          alpha * raw.z + (1.0 - alpha) * torpedo_velocity_.z};
      } else {
        torpedo_velocity_ = raw;
        have_torpedo_velocity_ = true;
      }
    } else if (dt >= 2.0) {
      have_torpedo_velocity_ = false;  // 수신 공백: 추정을 다시 시작한다
    }
  } else {
    have_torpedo_velocity_ = false;  // 첫 샘플 또는 frame 변경
  }

  last_torpedo_position_ = torpedo.position;
  last_torpedo_received_sec_ = torpedo.received_sec;
  last_torpedo_stamp_sec_ = torpedo.stamp_sec;
  last_torpedo_frame_ = torpedo.frame_id;
  last_torpedo_sequence_ = torpedo.sequence;
  have_torpedo_history_ = true;
}

// Dynamic VO: position differencing avoids assuming odometry twist frame semantics.
void PlanningCore::updateRobotVelocity(const VehicleState & bluerov)
{
  if (!bluerov.valid ||
    (have_robot_history_ && bluerov.sequence == last_robot_sequence_))
  {
    return;
  }

  if (have_robot_history_ && bluerov.frame_id == last_robot_frame_) {
    double dt = bluerov.received_sec - last_robot_received_sec_;
    if (bluerov.stamp_sec > 0.0 && last_robot_stamp_sec_ > 0.0) {
      const double message_dt = bluerov.stamp_sec - last_robot_stamp_sec_;
      if (message_dt > 0.0) {
        dt = message_dt;
      }
    }
    if (dt > 1.0e-3 && dt < 2.0) {
      robot_velocity_ = {
        (bluerov.position.x - last_robot_position_.x) / dt,
        (bluerov.position.y - last_robot_position_.y) / dt,
        (bluerov.position.z - last_robot_position_.z) / dt};
      have_robot_velocity_ = true;
    } else if (dt >= 2.0) {
      have_robot_velocity_ = false;
    }
  } else {
    have_robot_velocity_ = false;
  }

  last_robot_position_ = bluerov.position;
  last_robot_received_sec_ = bluerov.received_sec;
  last_robot_stamp_sec_ = bluerov.stamp_sec;
  last_robot_frame_ = bluerov.frame_id;
  last_robot_sequence_ = bluerov.sequence;
  have_robot_history_ = true;
}

// 교전 상태기계: IDLE -> ENGAGED -> (HIT | AVOIDED) -> IDLE.
// HIT은 새 미션 목표가 올 때까지 래치되어 계획을 중단시키고,
// AVOIDED는 회차별(자동추적 U턴 재공격 포함)로 판정된다.
// 판단만 하고 행동(로그·정지 경로 발행)은 어댑터가 Decision을 보고 한다.
void PlanningCore::updateEngagement(
  const PlanningInput & input,
  const bool torpedo_detected,
  const Point3D & rov_position,
  Decision & decision)
{
  if (!torpedo_detected) {
    if (engaged_) {
      // 교전 중 어뢰 소실: 피격 없이 끝났으므로 회피 성공으로 마감한다.
      engaged_ = false;
      have_avoided_event_ = true;
      avoided_min_distance_ = engagement_min_distance_;
      avoided_event_sec_ = input.now_sec;
      decision.torpedo_lost_avoided = true;
    }
    have_engagement_history_ = false;
    return;
  }

  const Point3D torpedo_position = input.torpedo.position;
  const double distance = distance3D(rov_position, torpedo_position);
  double closest = distance;
  if (have_engagement_history_) {
    closest = std::min(
      closest,
      closestApproachDistance(
        previous_rov_position_, rov_position,
        previous_torpedo_position_, torpedo_position));
  }
  previous_rov_position_ = rov_position;
  previous_torpedo_position_ = torpedo_position;
  have_engagement_history_ = true;

  if (!engaged_) {
    if (distance < config_.avoid.engage_radius) {
      engaged_ = true;
      engagement_min_distance_ = closest;
      decision.engagement_started = true;
      decision.engagement_distance = distance;
    }
    return;
  }

  engagement_min_distance_ = std::min(engagement_min_distance_, closest);

  if (!hit_latched_ && engagement_min_distance_ < config_.avoid.hit_radius) {
    engaged_ = false;
    hit_distance_ = engagement_min_distance_;
    hit_mission_sequence_ = input.goal.sequence;
    hit_latched_ = true;
    // 정지 후 재시도가 즉시 되도록 재계획 기준도 초기화한다.
    last_requested_.reset();
    decision.hit = true;
    decision.stop_requested = true;
    return;
  }

  // 경계에서 교전이 켜졌다 꺼졌다 하지 않도록 이탈 판정에 10% 여유를 둔다.
  if (distance > config_.avoid.engage_radius * 1.1) {
    engaged_ = false;
    have_avoided_event_ = true;
    avoided_min_distance_ = engagement_min_distance_;
    avoided_event_sec_ = input.now_sec;
    decision.avoided = true;
  }
}

bool PlanningCore::needsReplan(
  const PlanRequest & request,
  const std::vector<BoxObstacle> & obstacles,
  const std::vector<Point3D> & last_path,
  const double now_sec) const
{
  if (!last_requested_) {
    return true;
  }
  if (request.frame_id != last_requested_->frame_id) {
    return true;
  }

  if (!config_.replan.collision_only) {
    // 기존 거리 기반 정책이다. 어뢰 유무(모드)가 바뀌면 즉시 재계획한다.
    if (request.torpedo_valid != last_requested_->torpedo_valid) {
      return true;
    }
    const bool torpedo_moved = request.torpedo_valid &&
      distance3D(request.torpedo, last_requested_->torpedo) >=
      config_.torpedo_replan_distance;
    return torpedo_moved ||
           distance3D(request.start, last_requested_->start) >=
             config_.robot_replan_distance ||
           distance3D(request.goal, last_requested_->goal) >=
             config_.goal_replan_distance;
  }

  // 히스테리시스 정책: 충돌·이탈·목표 변경 때만 재계획한다.
  // 계획 실패가 반복돼도 5Hz로 폭주하지 않도록 요청 간격을 제한한다.
  if (now_sec - last_replan_request_sec_ < config_.replan.min_interval_sec) {
    return false;
  }

  if (last_path.empty()) {
    return true;  // 유효한 경로가 아직 없다(직전 계획 실패 포함).
  }
  // 경로 끝점이 현재 목표와 다르면 목표가 바뀌었거나 직전 계획이
  // 실패한 채 남은 옛 경로라는 뜻이므로 다시 계획한다.
  if (distance3D(request.goal, last_path.back()) >=
    config_.goal_replan_distance)
  {
    return true;
  }

  // 로봇에서 가장 가까운 waypoint를 찾아 경로 이탈 여부를 본다.
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < last_path.size(); ++index) {
    const double candidate = distance3D(request.start, last_path[index]);
    if (candidate < closest_distance) {
      closest_distance = candidate;
      closest_index = index;
    }
  }
  if (closest_distance > config_.replan.path_deviation_distance) {
    return true;
  }

  // 이미 지나온 구간은 제외하고, 남은 경로가 새 어뢰 통로와 충돌하면
  // 재계획한다. 충돌이 없으면 어뢰가 움직여도 경로를 그대로 유지한다.
  const double margin =
    config_.astar.safety_margin + config_.replan.collision_margin;
  for (std::size_t index = closest_index; index < last_path.size(); ++index) {
    for (const auto & obstacle : obstacles) {
      if (boxContains(obstacle, last_path[index], margin)) {
        return true;
      }
    }
  }
  return false;
}

Decision PlanningCore::update(
  const PlanningInput & input,
  const std::vector<Point3D> & last_path)
{
  Decision decision;

  // 미션 목표가 아직 없어도 어뢰 속도 추정은 계속 워밍업한다.
  updateTorpedoVelocity(input.torpedo);
  // Dynamic VO: keep ROV velocity warm even before a mission goal arrives.
  updateRobotVelocity(input.bluerov);

  // 어뢰 탐지: 수신 신선도로 NORMAL/AVOID 모드를 가른다.
  const bool torpedo_detected =
    sampleFresh(input.torpedo, input.now_sec, config_.avoid.torpedo_timeout_sec);

  if (!input.bluerov.valid) {
    return decision;
  }

  const std::string frame_id =
    input.bluerov.frame_id.empty() ? "map" : input.bluerov.frame_id;
  decision.frame_id = frame_id;
  const Point3D start = input.bluerov.position;

  if (torpedo_detected != avoid_mode_) {
    avoid_mode_ = torpedo_detected;
    decision.mode_changed = true;
  }
  decision.avoid_mode = avoid_mode_;

  // 피격 래치는 새 미션 목표가 도착하면 해제하고 계획을 재개한다.
  if (hit_latched_ && input.goal.valid &&
    input.goal.sequence != hit_mission_sequence_)
  {
    hit_latched_ = false;
    engaged_ = false;
    decision.reset = true;
  }

  updateEngagement(input, torpedo_detected, start, decision);

  decision.hit_latched = hit_latched_;
  decision.hit_distance = hit_distance_;
  decision.show_avoided = have_avoided_event_ &&
    (input.now_sec - avoided_event_sec_) < 5.0;
  decision.avoided_min_distance = avoided_min_distance_;

  if (hit_latched_) {
    decision.skip = Decision::Skip::kHitLatched;
    return decision;
  }

  if (torpedo_detected &&
    !framesCompatible(frame_id, input.torpedo.frame_id))
  {
    decision.skip = Decision::Skip::kTorpedoFrameMismatch;
    return decision;
  }

  Point3D goal;
  if (config_.use_target_topic_for_goal) {
    if (!input.goal.valid) {
      return decision;
    }
    if (!framesCompatible(frame_id, input.goal.frame_id)) {
      decision.skip = Decision::Skip::kGoalFrameMismatch;
      return decision;
    }
    goal = input.goal.point;
  } else {
    if (!fixed_goal_) {
      fixed_goal_ = Point3D{
        start.x + config_.goal_offset_x,
        start.y + config_.goal_offset_y,
        start.z + config_.goal_offset_z};
    }
    goal = *fixed_goal_;
  }

  const PlanRequest request{
    start,
    have_robot_velocity_ ? robot_velocity_ : Point3D{},
    goal,
    torpedo_detected ? input.torpedo.position : Point3D{},
    (torpedo_detected && have_torpedo_velocity_) ?
    torpedo_velocity_ : Point3D{},
    torpedo_detected, input.planner, frame_id};
  // 충돌 검사용 통로는 worker의 execute()와 같은 함수로 만들어 판정을
  // 일치시킨다.
  const auto obstacles = buildTorpedoObstacles(request, config_);
  if (needsReplan(request, obstacles, last_path, input.now_sec)) {
    last_requested_ = request;
    last_replan_request_sec_ = input.now_sec;
    decision.global_replan_required = true;
  }
  // 계획을 언제 요청할지는 알고리즘 성격에 따라 다르다.
  switch (input.planner) {
    case PlannerType::kAStar:
      // 격자 탐색은 비싸고 경로가 안정적이라 히스테리시스대로만 돈다.
      if (decision.global_replan_required) {
        decision.plan_request = request;
      }
      break;
    case PlannerType::kDynamicVO:
      // 반응형이라 매 틱 새로 뽑는다(receding-horizon).
      decision.plan_request = request;
      break;
    case PlannerType::kHybrid:
      // A*는 필요할 때만, DVO는 어뢰가 보이는 동안 매 틱.
      if (torpedo_detected || decision.global_replan_required) {
        decision.plan_request = request;
      }
      break;
  }
  return decision;
}

}  // namespace bluerov_integration::team_min
