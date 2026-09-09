#include "bluerov_integration/team_min/planning_engine.hpp"

#include <utility>
#include <vector>

namespace bluerov_integration::team_min
{
namespace
{

// DVO 단독 경로가 이보다 짧으면 PathFollower가 경로 끝에 도달했다고 보고
// 미션 목표로 직행할 수 있다. 한 구간은 최대 1.5 m이므로 5점은 약 6 m다.
constexpr std::size_t kMinimumDynamicVOWaypoints = 5U;

}  // namespace

PlanningEngine::PlanningEngine(PlanningEngineConfig config)
: config_(std::move(config))
{
}

void PlanningEngine::reset()
{
  global_path_.clear();
}

PlanningEngineOutput PlanningEngine::plan(const PlanningWork & work)
{
  const PlanRequest & request = work.request;
  PlanningEngineOutput output;
  PlanResult & result = output.result;

  // DVO 단독은 실제 움직이는 장애물을 직접 사용한다. 나머지 플래너는
  // A* 전역 경로를 만들기 위해 어뢰 예측 통로를 정적 박스로 변환한다.
  if (request.planner != PlannerType::kDynamicVO) {
    result.obstacles = buildTorpedoObstacles(request, config_.core);
  }
  const auto & obstacles = result.obstacles;

  if (!obstacles.empty()) {
    const auto & live_barrier = obstacles.front();
    if (boxContains(
        live_barrier, request.goal, config_.core.astar.safety_margin))
    {
      result.failure = PlanFailure::kGoalInsideBarrier;
      return output;
    }
    if (boxContains(
        live_barrier, request.start, config_.core.astar.safety_margin))
    {
      result.failure = PlanFailure::kStartInsideBarrier;
      return output;
    }
  }

  const GridMapConfig map = planningMap(request, obstacles, config_.core);

  if (request.planner == PlannerType::kDynamicVO) {
    DynamicVOOptions options = config_.dynamic_vo;
    options.standalone = true;
    std::vector<MovingObstacle> moving_obstacles;
    if (request.torpedo_valid) {
      BoxObstacle live_torpedo = config_.core.torpedo_barrier;
      live_torpedo.center = request.torpedo;
      moving_obstacles.push_back({live_torpedo, request.torpedo_velocity});
    }
    const DynamicVOResult vo = runDynamicVO3D(
      request.start, request.robot_velocity, request.goal, map,
      moving_obstacles, options);
    if (!vo.success || vo.local_path.size() < kMinimumDynamicVOWaypoints) {
      result.failure = PlanFailure::kNoSafeLocalPath;
      result.stop_requested = true;
      return output;
    }
    result.path = vo.local_path;
    result.vo_active = vo.avoidance_required;
    result.valid = true;
    return output;
  }

  if (work.global_replan_required || global_path_.empty()) {
    auto new_global_path = runEnhancedAStar3D(
      request.start, request.goal, map, obstacles, config_.core.astar);
    if (new_global_path.empty()) {
      result.failure = PlanFailure::kNoAStarPath;
      return output;
    }
    global_path_ = std::move(new_global_path);
    output.replan_reference_path = global_path_;
  }

  result.path = global_path_;
  result.valid = !result.path.empty();
  if (request.planner == PlannerType::kAStar) {
    return output;
  }

  if (request.planner == PlannerType::kSpaceTime) {
    if (!request.torpedo_valid || global_path_.empty()) {
      return output;
    }
    const LocalTarget target = selectLocalTarget(
      request.start, global_path_, config_.spacetime_lookahead);
    MovingBox torpedo;
    torpedo.center = request.torpedo;
    torpedo.velocity = request.torpedo_velocity;
    torpedo.half_x = config_.core.torpedo_barrier.size_x * 0.5;
    torpedo.half_y = config_.core.torpedo_barrier.size_y * 0.5;
    torpedo.half_z = config_.core.torpedo_barrier.size_z * 0.5;

    const SpaceTimeResult local = runSpaceTimeAStar3D(
      request.start, target.point, {torpedo}, config_.spacetime);
    if (local.goal_unreachable) {
      result.failure = PlanFailure::kSpaceTimeUnreachable;
      return output;
    }
    if (!local.success) {
      result.failure = PlanFailure::kNoSpaceTimePath;
      result.valid = false;
      result.stop_requested = true;
      return output;
    }
    result.path = mergePaths(
      stripTimes(local.path), global_path_, target.index);
    result.vo_active = true;
    result.expanded_nodes = local.expanded;
    return output;
  }

  // Hybrid: Cruise는 전역 A*를 그대로 사용하고 Approach/Break에서만
  // 움직이는 실제 어뢰 하나를 대상으로 DVO 국소 계획을 수행한다.
  if (request.torpedo_valid && !global_path_.empty() &&
    work.phase != EngagementPhase::kCruise)
  {
    const bool breaking = work.phase == EngagementPhase::kBreak;
    const LocalTarget target = selectLocalTarget(
      request.start, global_path_, config_.dynamic_vo.path_lookahead);
    BoxObstacle live_torpedo = config_.core.torpedo_barrier;
    live_torpedo.center = request.torpedo;
    const std::vector<MovingObstacle> moving_obstacles{{
      live_torpedo, request.torpedo_velocity}};
    DynamicVOOptions options = config_.dynamic_vo;
    options.break_mode = breaking;
    options.break_reverse = breaking &&
      config_.core.ttc.reverse_sec > 0.0 &&
      work.ttc_sec < config_.core.ttc.reverse_sec;
    output.jink_active = options.break_reverse;
    options.standalone = true;

    const DynamicVOResult vo = runDynamicVO3D(
      request.start, request.robot_velocity, target.point, map,
      moving_obstacles, options);
    result.vo_active = vo.avoidance_required || breaking;
    if (vo.avoidance_required) {
      if (!vo.success || vo.local_path.empty()) {
        result.failure = PlanFailure::kNoSafeLocalPath;
        result.valid = false;
        result.stop_requested = true;
        return output;
      }
      result.path = mergePaths(vo.local_path, global_path_, target.index);
    }
  }
  return output;
}

}  // namespace bluerov_integration::team_min
