#include "bluerov_integration/team_min/planning_module.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bluerov_integration::team_min
{
namespace
{

// 예측 파라미터는 team_min 소유이므로 통합 노드(loadPlanningConfig)를
// 수정하지 않고 여기서 직접 선언한다.
PredictionConfig loadPredictionParameters(
  rclcpp::Node & node,
  PredictionConfig defaults)
{
  defaults.enabled = node.declare_parameter<bool>(
    "planning.prediction.enabled", defaults.enabled);
  defaults.horizon_sec = node.declare_parameter<double>(
    "planning.prediction.horizon_sec", defaults.horizon_sec);
  defaults.spacing = node.declare_parameter<double>(
    "planning.prediction.spacing", defaults.spacing);
  const std::int64_t max_boxes = node.declare_parameter<std::int64_t>(
    "planning.prediction.max_boxes",
    static_cast<std::int64_t>(defaults.max_boxes));
  if (max_boxes < 0 || max_boxes > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(
            "planning.prediction.max_boxes is out of range");
  }
  defaults.max_boxes = static_cast<int>(max_boxes);
  defaults.min_speed = node.declare_parameter<double>(
    "planning.prediction.min_speed", defaults.min_speed);
  defaults.velocity_alpha = node.declare_parameter<double>(
    "planning.prediction.velocity_alpha", defaults.velocity_alpha);
  defaults.start_clearance = node.declare_parameter<double>(
    "planning.prediction.start_clearance", defaults.start_clearance);
  return defaults;
}

ReplanPolicyConfig loadReplanParameters(
  rclcpp::Node & node,
  ReplanPolicyConfig defaults)
{
  defaults.collision_only = node.declare_parameter<bool>(
    "planning.replan.collision_only", defaults.collision_only);
  defaults.path_deviation_distance = node.declare_parameter<double>(
    "planning.replan.path_deviation_distance",
    defaults.path_deviation_distance);
  defaults.collision_margin = node.declare_parameter<double>(
    "planning.replan.collision_margin", defaults.collision_margin);
  defaults.min_interval_sec = node.declare_parameter<double>(
    "planning.replan.min_interval_sec", defaults.min_interval_sec);
  return defaults;
}

AvoidConfig loadAvoidParameters(
  rclcpp::Node & node,
  AvoidConfig defaults)
{
  defaults.torpedo_timeout_sec = node.declare_parameter<double>(
    "planning.avoid.torpedo_timeout_sec", defaults.torpedo_timeout_sec);
  defaults.engage_radius = node.declare_parameter<double>(
    "planning.avoid.engage_radius", defaults.engage_radius);
  defaults.hit_radius = node.declare_parameter<double>(
    "planning.avoid.hit_radius", defaults.hit_radius);
  return defaults;
}

// 노드가 채운 평면 PlanningConfig에서 코어가 아는 값만 옮긴다.
// ROS 토픽명은 코어로 넘어가지 않는다.
PlanningCoreConfig toCoreConfig(const PlanningConfig & config)
{
  PlanningCoreConfig core;
  core.use_dynamic_map = config.use_dynamic_map;
  core.use_target_topic_for_goal = config.use_target_topic_for_goal;
  core.goal_offset_x = config.goal_offset_x;
  core.goal_offset_y = config.goal_offset_y;
  core.goal_offset_z = config.goal_offset_z;
  core.map_padding_x = config.map_padding_x;
  core.map_padding_y = config.map_padding_y;
  core.map_padding_z = config.map_padding_z;
  core.torpedo_replan_distance = config.torpedo_replan_distance;
  core.robot_replan_distance = config.robot_replan_distance;
  core.goal_replan_distance = config.goal_replan_distance;
  core.fixed_map = config.fixed_map;
  core.astar = config.astar;
  core.torpedo_barrier = config.torpedo_barrier;
  core.prediction = config.prediction;
  core.replan = config.replan;
  core.avoid = config.avoid;
  return core;
}

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
  config_.prediction = loadPredictionParameters(node, config_.prediction);
  config_.replan = loadReplanParameters(node, config_.replan);
  config_.avoid = loadAvoidParameters(node, config_.avoid);
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
    decision.frame_id, input.bluerov.position, decision.avoid_mode,
    decision.hit_latched, decision.hit_distance,
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
      pending_request_ = *decision.plan_request;
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
    std::optional<PlanRequest> request;
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

void PlanningModule::execute(const PlanRequest & request)
{
  const auto calculation_start = std::chrono::steady_clock::now();
  try {
    const auto obstacles = buildTorpedoObstacles(request, core_config_);
    // 어뢰 현재 위치 박스가 start/goal을 덮으면 A*는 성공할 수 없다.
    // 일반 예외 대신 원인을 명시하고, 어뢰가 지나간 뒤의 재시도(경로가
    // 없으므로 needsReplan이 계속 true)에 맡긴다. NORMAL 모드에서는
    // 장애물이 없으므로 검사하지 않는다.
    if (!obstacles.empty()) {
      const auto & live_barrier = obstacles.front();
      if (boxContains(
          live_barrier, request.goal, core_config_.astar.safety_margin))
      {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "team_min goal (%.1f, %.1f, %.1f) is inside the torpedo barrier; "
          "waiting for the torpedo to move away",
          request.goal.x, request.goal.y, request.goal.z);
        return;
      }
      if (boxContains(
          live_barrier, request.start, core_config_.astar.safety_margin))
      {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "team_min start is inside the torpedo barrier; cannot plan until "
          "the torpedo moves away");
        return;
      }
    }
    const auto path = runEnhancedAStar3D(
      request.start,
      request.goal,
      planningMap(request, obstacles, core_config_),
      obstacles,
      core_config_.astar);
    if (path.empty()) {
      RCLCPP_WARN(logger_, "team_min A* found no path");
      return;
    }
    if (hit_latched_.load()) {
      return;  // 피격 정지 중에는 낡은 계획 결과를 발행하지 않는다.
    }

    nav_msgs::msg::Path path_message;
    path_message.header.stamp = clock_->now();
    path_message.header.frame_id = request.frame_id;
    path_message.poses.reserve(path.size());
    for (const auto & waypoint : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_message.header;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.position.z = waypoint.z;
      pose.pose.orientation.w = 1.0;
      path_message.poses.push_back(std::move(pose));
    }
    path_pub_->publish(path_message);
    {
      // needsReplan의 충돌·이탈 검사가 최신 경로를 보도록 보관한다.
      std::lock_guard<std::mutex> lock(request_mutex_);
      last_path_ = path;
    }
    publishPoint(current_point_pub_, request.start, request.frame_id);
    publishPoint(goal_point_pub_, request.goal, request.frame_id);
    publishPoint(torpedo_point_pub_, request.torpedo, request.frame_id);
    visualizer_->publishScene(
      request.frame_id, request.start, request.goal, obstacles, path);

    const double calculation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - calculation_start).count();
    RCLCPP_INFO(
      logger_, "team_min A* time=%.3f ms, waypoints=%zu, boxes=%zu",
      calculation_ms, path.size(), obstacles.size());
  } catch (const std::exception & error) {
    const double calculation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - calculation_start).count();
    RCLCPP_ERROR(
      logger_, "team_min A* failed after %.3f ms: %s",
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
