#include "bluerov_integration/team_min/planning_module.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "bluerov_integration/team_min/planning_params.hpp"

namespace bluerov_integration::team_min
{
namespace
{

// DVO 단독 모드가 내보낼 최소 waypoint 수다. 경로가 이보다 짧으면
// PathFollower가 lookahead(기본 2 m)로 경로 끝을 넘어서고, 그 순간
// "최종 목표 도달"로 판정해 미션 목표로 직행한다(회피 무시).
// 롤아웃 한 구간이 최대 1.5 m(0.5 s x 3 m/s)이므로 5점이면 약 6 m다.
constexpr std::size_t kMinimumDynamicVOWaypoints = 5U;

double toSeconds(const std::chrono::steady_clock::time_point time_point)
{
  return std::chrono::duration<double>(time_point.time_since_epoch()).count();
}

}  // namespace

PlanningModule::PlanningModule(rclcpp::Node & node, PlanningConfig config)
: config_(std::move(config)),
  logger_(node.get_logger()),
  clock_(node.get_clock())
{
  // 파라미터 선언·로딩은 planning_params.cpp가 담당한다.
  loadTeamMinParameters(node, config_);
  core_config_ = toCoreConfig(config_);

  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  path_pub_ = node.create_publisher<nav_msgs::msg::Path>(
    config_.path_topic, latched_qos);
  current_point_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.current_point_topic, latched_qos);
  goal_point_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.goal_point_topic, latched_qos);
  torpedo_point_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
    config_.torpedo_point_topic, latched_qos);
  visualizer_ = std::make_unique<RvizVisualizer>(node, config_.marker_topic);

  if (!config_.enabled) {
    RCLCPP_INFO(
      logger_,
      "team_min A* disabled; BlueROV and torpedo RViz tracking remains active");
    return;
  }

  // 실험용 알고리즘 선택. 런타임에 ros2 param set으로 바꿀 수 있다.
  const std::string planner_name =
    node.declare_parameter<std::string>("planning.planner", "hybrid");
  const auto parsed_planner = parsePlanner(planner_name);
  if (!parsed_planner) {
    throw std::invalid_argument(
            "planning.planner must be one of: astar, dvo, hybrid");
  }
  planner_.store(*parsed_planner);
  RCLCPP_INFO(
    logger_, "team_min planner: %s (ros2 param set으로 전환 가능)",
    plannerName(*parsed_planner));

  parameter_callback_ = node.add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() != "planning.planner") {
          continue;
        }
        const auto requested = parsePlanner(parameter.as_string());
        if (!requested) {
          result.successful = false;
          result.reason = "planning.planner must be astar, dvo, or hybrid";
          continue;
        }
        const PlannerType previous = planner_.exchange(*requested);
        if (previous != *requested) {
          // 이전 알고리즘이 남긴 경로는 버리고 새 플래너가 즉시 계획하게 한다.
          {
            std::lock_guard<std::mutex> lock(request_mutex_);
            last_path_.clear();
            global_path_.clear();
            pending_request_.reset();
          }
          RCLCPP_INFO(
            logger_, "team_min planner switched: %s -> %s",
            plannerName(previous), plannerName(*requested));
        }
      }
      return result;
    });

  // 설정 검증은 코어 생성자가 한다(잘못되면 std::invalid_argument).
  core_ = std::make_unique<PlanningCore>(core_config_);

  running_.store(true);
  worker_ = std::thread(&PlanningModule::workerLoop, this);
  RCLCPP_INFO(logger_, "team_min A* worker started");
}

PlanningModule::~PlanningModule()
{
  stop();
}

void PlanningModule::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  request_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

Point3D PlanningModule::positionOf(const nav_msgs::msg::Odometry & odometry)
{
  return {
    odometry.pose.pose.position.x,
    odometry.pose.pose.position.y,
    odometry.pose.pose.position.z};
}

VehicleState PlanningModule::toVehicleState(
  const common::ReceivedSample<nav_msgs::msg::Odometry> & sample)
{
  VehicleState state;
  state.valid = sample.metadata.valid;
  if (!state.valid) {
    return state;
  }
  state.position = positionOf(sample.message);
  state.frame_id = sample.message.header.frame_id;
  state.sequence = sample.metadata.sequence;
  // sim time 스탬프. 없으면 0으로 남아 코어가 수신 시각 폴백을 쓴다.
  state.stamp_sec =
    rclcpp::Time(sample.message.header.stamp, RCL_ROS_TIME).seconds();
  state.received_sec = toSeconds(sample.metadata.received_steady_time);
  return state;
}

// ROS/DataHub 타입을 만지는 유일한 입력 변환 지점이다.
PlanningInput PlanningModule::makePlanningInput(
  const common::StateSnapshot & snapshot) const
{
  PlanningInput input;
  input.now_sec = toSeconds(std::chrono::steady_clock::now());
  input.planner = planner_.load();
  input.bluerov = toVehicleState(snapshot.bluerov_odometry);
  input.torpedo = toVehicleState(snapshot.torpedo_odometry);
  const auto & goal = snapshot.mission_goal;
  input.goal.valid = goal.metadata.valid;
  if (goal.metadata.valid) {
    input.goal.point = {
      goal.message.point.x, goal.message.point.y, goal.message.point.z};
    input.goal.frame_id = goal.message.header.frame_id;
    input.goal.sequence = goal.metadata.sequence;
  }
  return input;
}

void PlanningModule::update(const common::StateSnapshot & snapshot)
{
  const PlanningInput input = makePlanningInput(snapshot);
  // 시각화용 어뢰 탐지 판정은 코어와 같은 식(sampleFresh)을 쓴다.
  const bool torpedo_detected = sampleFresh(
    input.torpedo, input.now_sec, config_.avoid.torpedo_timeout_sec);

  if (snapshot.bluerov_odometry.metadata.valid) {
    visualizer_->publishBlueRov(snapshot.bluerov_odometry.message);
  }
  if (snapshot.torpedo_odometry.metadata.valid) {
    BoxObstacle live_barrier = config_.torpedo_barrier;
    live_barrier.center = positionOf(snapshot.torpedo_odometry.message);
    visualizer_->publishTorpedo(
      snapshot.torpedo_odometry.message, live_barrier,
      config_.enabled && torpedo_detected);
  }
  visualizer_->publishTelemetry(snapshot);

  if (!core_) {
    return;  // planning.enabled=false
  }

  // 코어에는 최신 경로를 복사로 넘긴다(코어는 mutex를 모른다).
  std::vector<Point3D> last_path_copy;
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    last_path_copy = last_path_;
  }
  const Decision decision = core_->update(input, last_path_copy);

  if (!input.bluerov.valid) {
    return;
  }
  hit_latched_.store(decision.hit_latched);
  handleDecision(decision, input);
}

// 판단은 코어가 했고, 여기서는 행동(로그·정지 경로·마커·worker 요청)만
// 한다. 로그 문자열은 분리 전과 동일하게 유지한다.
void PlanningModule::handleDecision(
  const Decision & decision,
  const PlanningInput & input)
{
  if (decision.mode_changed) {
    if (decision.avoid_mode) {
      RCLCPP_INFO(logger_, "team_min mode: AVOID (torpedo detected)");
    } else {
      RCLCPP_INFO(logger_, "team_min mode: NORMAL (no torpedo)");
    }
  }
  if (decision.reset) {
    RCLCPP_INFO(
      logger_, "team_min [RESET] new mission goal received; resuming");
  }
  if (decision.engagement_started) {
    RCLCPP_INFO(
      logger_, "team_min engagement started (torpedo %.1f m away)",
      decision.engagement_distance);
  }
  if (decision.hit) {
    RCLCPP_ERROR(
      logger_,
      "team_min [HIT] torpedo hit the BlueROV (closest %.2f m); "
      "stopping until a new mission goal arrives",
      decision.hit_distance);
  }
  if (decision.stop_requested) {
    publishStopPath(decision.frame_id);
  }
  if (decision.avoided) {
    RCLCPP_INFO(
      logger_,
      "team_min [AVOIDED] torpedo pass survived (min distance %.1f m)",
      decision.avoided_min_distance);
  }
  if (decision.torpedo_lost_avoided) {
    RCLCPP_INFO(
      logger_,
      "team_min [AVOIDED] torpedo lost during engagement "
      "(min distance %.1f m)",
      decision.avoided_min_distance);
  }

  visualizer_->publishStatus(
    decision.frame_id, input.bluerov.position, plannerName(input.planner),
    decision.avoid_mode, decision.hit_latched, decision.hit_distance,
    decision.show_avoided, decision.avoided_min_distance);

  switch (decision.skip) {
    case Decision::Skip::kHitLatched:
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "team_min HIT latched; enter a new mission goal to resume");
      return;
    case Decision::Skip::kTorpedoFrameMismatch:
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min frame mismatch: BlueROV='%s', torpedo='%s'",
        decision.frame_id.c_str(), input.torpedo.frame_id.c_str());
      return;
    case Decision::Skip::kGoalFrameMismatch:
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min target frame mismatch: planning='%s', target='%s'",
        decision.frame_id.c_str(), input.goal.frame_id.c_str());
      return;
    case Decision::Skip::kNone:
      break;
  }

  if (decision.plan_request) {
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      // Dynamic VO: newest state replaces stale local-planning work.
      const bool preserve_global_replan = decision.global_replan_required ||
        (pending_request_ && pending_request_->global_replan_required);
      pending_request_ = PlanningWork{
        *decision.plan_request, preserve_global_replan};
    }
    request_cv_.notify_one();
  }
}

// 빈 경로를 발행하면 PathFollower(stop_without_valid_path=true)가 목표를
// 무효화하고 제어가 추력 0으로 내려간다. team_min 밖 코드를 수정하지 않는
// 정지 방법이다.
void PlanningModule::publishStopPath(const std::string & frame_id)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = clock_->now();
  empty_path.header.frame_id = frame_id;
  path_pub_->publish(empty_path);

  std::lock_guard<std::mutex> lock(request_mutex_);
  last_path_.clear();
  pending_request_.reset();
}

void PlanningModule::workerLoop()
{
  while (running_.load()) {
    std::optional<PlanningWork> request;
    {
      std::unique_lock<std::mutex> lock(request_mutex_);
      request_cv_.wait(lock, [this]() {
        return !running_.load() || pending_request_.has_value();
      });
      if (!running_.load() && !pending_request_) {
        break;
      }
      request = std::move(pending_request_);
      pending_request_.reset();
    }
    if (request) {
      execute(*request);
    }
  }
}

// 알고리즘 분기는 이 함수 안에만 있다. A*든 DVO든 결과는 PlanResult 하나로
// 나오므로 호출부(execute/publishPlan)는 무슨 플래너였는지 몰라도 된다.
PlanResult PlanningModule::runPlanner(const PlanningWork & work)
{
  const PlanRequest & request = work.request;
  PlanResult result;
  // 예측 통로는 A*가 쓰는 정적 장애물이다. DVO 단독 모드는 어뢰를 움직이는
  // 물체로 직접 다루므로 통로를 만들지 않는다.
  if (request.planner != PlannerType::kDynamicVO) {
    result.obstacles = buildTorpedoObstacles(request, core_config_);
  }
  const auto & obstacles = result.obstacles;

  // 어뢰 현재 위치 박스가 start/goal을 덮으면 A*는 성공할 수 없다.
  // 일반 예외 대신 원인을 명시하고, 어뢰가 지나간 뒤의 재시도(경로가
  // 없으므로 needsReplan이 계속 true)에 맡긴다.
  if (!obstacles.empty()) {
    const auto & live_barrier = obstacles.front();
    if (boxContains(
        live_barrier, request.goal, core_config_.astar.safety_margin))
    {
      result.failure = PlanFailure::kGoalInsideBarrier;
      return result;
    }
    if (boxContains(
        live_barrier, request.start, core_config_.astar.safety_margin))
    {
      result.failure = PlanFailure::kStartInsideBarrier;
      return result;
    }
  }

  const GridMapConfig map = planningMap(request, obstacles, core_config_);

  // DVO 단독: A*를 전혀 쓰지 않고 미션 목표를 향해 직접 롤아웃한다.
  if (request.planner == PlannerType::kDynamicVO) {
    DynamicVOOptions options = config_.dynamic_vo;
    options.standalone = true;
    std::vector<MovingObstacle> moving_obstacles;
    if (request.torpedo_valid) {
      BoxObstacle live_torpedo = core_config_.torpedo_barrier;
      live_torpedo.center = request.torpedo;
      moving_obstacles.push_back({live_torpedo, request.torpedo_velocity});
    }
    const DynamicVOResult vo = runDynamicVO3D(
      request.start, request.robot_velocity, request.goal, map,
      moving_obstacles, options);
    if (!vo.success || vo.local_path.size() < kMinimumDynamicVOWaypoints) {
      // 경로가 너무 짧으면 PathFollower가 "끝에 도달"로 보고 미션 목표로
      // 직행해 회피를 무시한다. 그런 경로는 아예 내보내지 않고 정지시킨다.
      result.failure = PlanFailure::kNoSafeLocalPath;
      result.stop_requested = true;
      return result;
    }
    result.path = vo.local_path;
    result.vo_active = vo.avoidance_required;
    result.valid = true;
    return result;
  }

  // A* 단독 / 하이브리드: 전역 경로를 먼저 확보한다.
  if (work.global_replan_required || global_path_.empty()) {
    auto new_global_path = runEnhancedAStar3D(
      request.start, request.goal, map, obstacles, core_config_.astar);
    if (new_global_path.empty()) {
      result.failure = PlanFailure::kNoAStarPath;
      return result;
    }
    global_path_ = std::move(new_global_path);
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      // Dynamic VO: replan policy evaluates the stable A* path, not local detours.
      last_path_ = global_path_;
    }
  }

  result.path = global_path_;
  result.valid = !result.path.empty();
  if (request.planner == PlannerType::kAStar) {
    return result;  // A* 단독은 국소 회피를 하지 않는다.
  }

  // 하이브리드: A* 경로를 따라가다 위험하면 DVO가 국소로 비켜갔다 복귀한다.
  if (request.torpedo_valid && !global_path_.empty()) {
    const LocalTarget target = selectLocalTarget(
      request.start, global_path_, config_.dynamic_vo.path_lookahead);
    BoxObstacle live_torpedo = core_config_.torpedo_barrier;
    live_torpedo.center = request.torpedo;
    // Dynamic VO: use one real moving obstacle, not A* prediction boxes.
    const std::vector<MovingObstacle> moving_obstacles{{
      live_torpedo, request.torpedo_velocity}};
    const DynamicVOResult vo = runDynamicVO3D(
      request.start, request.robot_velocity, target.point, map,
      moving_obstacles, config_.dynamic_vo);
    result.vo_active = vo.avoidance_required;
    if (vo.avoidance_required) {
      if (!vo.success || vo.local_path.empty()) {
        result.failure = PlanFailure::kNoSafeLocalPath;
        result.valid = false;
        result.stop_requested = true;  // 병합 전 하이브리드와 같은 동작이다.
        return result;
      }
      result.path = mergePaths(vo.local_path, global_path_, target.index);
    }
  }
  return result;
}

// 계획 결과를 ROS로 내보낸다. 여기서부터 DataHub -> PathFollower -> PPID로
// 이어지며, 이 함수는 어떤 알고리즘이 경로를 만들었는지 알지 못한다.
void PlanningModule::publishPlan(
  const PlanResult & result,
  const PlanRequest & request)
{
  nav_msgs::msg::Path path_message;
  path_message.header.stamp = clock_->now();
  path_message.header.frame_id = request.frame_id;
  path_message.poses.reserve(result.path.size());
  for (const auto & waypoint : result.path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_message.header;
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = waypoint.z;
    pose.pose.orientation.w = 1.0;
    path_message.poses.push_back(std::move(pose));
  }
  path_pub_->publish(path_message);
  publishPoint(current_point_pub_, request.start, request.frame_id);
  publishPoint(goal_point_pub_, request.goal, request.frame_id);
  publishPoint(torpedo_point_pub_, request.torpedo, request.frame_id);
  visualizer_->publishScene(
    request.frame_id, request.start, request.goal, result.obstacles,
    result.path);
}

// runPlanner가 돌려준 실패 사유를 로그로 옮긴다. 문자열과 throttle 주기는
// 분리 전(runPlanner 안에서 직접 로그하던 때)과 동일하게 유지한다.
void PlanningModule::reportPlanFailure(
  const PlanFailure failure,
  const PlanRequest & request)
{
  switch (failure) {
    case PlanFailure::kGoalInsideBarrier:
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min goal (%.1f, %.1f, %.1f) is inside the torpedo barrier; "
        "waiting for the torpedo to move away",
        request.goal.x, request.goal.y, request.goal.z);
      break;
    case PlanFailure::kStartInsideBarrier:
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min start is inside the torpedo barrier; cannot plan until "
        "the torpedo moves away");
      break;
    case PlanFailure::kNoAStarPath:
      RCLCPP_WARN(logger_, "team_min A* found no path");
      break;
    case PlanFailure::kNoSafeLocalPath:
      RCLCPP_WARN(logger_, "team_min Dynamic VO found no safe local path");
      break;
    case PlanFailure::kNone:
      break;
  }
}

void PlanningModule::execute(const PlanningWork & work)
{
  const PlanRequest & request = work.request;
  const auto calculation_start = std::chrono::steady_clock::now();
  try {
    const PlanResult result = runPlanner(work);
    if (!result.valid) {
      // 판단은 runPlanner가 했고, 로그는 여기서 낸다(순수 계층 유지).
      // 문자열과 throttle 동작은 분리 전과 동일하다.
      reportPlanFailure(result.failure, request);
      // 회피 경로를 못 찾은 경우만 정지시킨다. A*가 한 번 실패한 것이나
      // 어뢰가 목표를 덮은 경우는 기존 경로를 두고 다음 틱에 재시도한다.
      if (result.stop_requested) {
        publishStopPath(request.frame_id);
      }
      return;
    }
    if (hit_latched_.load()) {
      return;  // 피격 정지 중에는 낡은 계획 결과를 발행하지 않는다.
    }

    publishPlan(result, request);

    const double calculation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - calculation_start).count();
    RCLCPP_INFO(
      logger_,
      "team_min %s time=%.3f ms, waypoints=%zu, boxes=%zu, Dynamic VO=%s",
      plannerName(request.planner), calculation_ms, result.path.size(),
      result.obstacles.size(), result.vo_active ? "active" : "clear");
  } catch (const std::exception & error) {
    const double calculation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - calculation_start).count();
    RCLCPP_ERROR(
      logger_, "team_min hybrid failed after %.3f ms: %s",
      calculation_ms, error.what());
  }
}

void PlanningModule::publishPoint(
  const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
  const Point3D & point,
  const std::string & frame_id)
{
  geometry_msgs::msg::PointStamped message;
  message.header.stamp = clock_->now();
  message.header.frame_id = frame_id;
  message.point.x = point.x;
  message.point.y = point.y;
  message.point.z = point.z;
  publisher->publish(message);
}

}  // namespace bluerov_integration::team_min
