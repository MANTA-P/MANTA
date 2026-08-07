#include "bluerov_integration/team_min/planning_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

}  // namespace

PlanningModule::PlanningModule(rclcpp::Node & node, PlanningConfig config)
: config_(std::move(config)),
  logger_(node.get_logger()),
  clock_(node.get_clock())
{
  config_.prediction = loadPredictionParameters(node, config_.prediction);
  config_.replan = loadReplanParameters(node, config_.replan);
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

bool PlanningModule::framesCompatible(
  const std::string & first,
  const std::string & second)
{
  return first.empty() || second.empty() || first == second;
}

bool PlanningModule::boxContains(
  const BoxObstacle & box,
  const Point3D & point,
  const double margin)
{
  return std::abs(point.x - box.center.x) <= box.size_x * 0.5 + margin &&
         std::abs(point.y - box.center.y) <= box.size_y * 0.5 + margin &&
         std::abs(point.z - box.center.z) <= box.size_z * 0.5 + margin;
}

// 어뢰 twist는 body 프레임일 수 있어 map 프레임 위치 차분으로 속도를 추정한다.
void PlanningModule::updateTorpedoVelocity(
  const common::ReceivedSample<nav_msgs::msg::Odometry> & sample)
{
  if (!sample.metadata.valid) {
    return;
  }
  if (have_torpedo_history_ &&
    sample.metadata.sequence == last_torpedo_sequence_)
  {
    return;  // planning 타이머가 같은 snapshot을 다시 본 경우
  }

  const Point3D position = positionOf(sample.message);
  const auto stamp = sample.metadata.received_steady_time;
  const rclcpp::Time message_stamp(sample.message.header.stamp, RCL_ROS_TIME);
  const std::string & frame = sample.message.header.frame_id;

  if (have_torpedo_history_ && frame == last_torpedo_frame_) {
    // use_sim_time에서 RTF가 1이 아니면 wall-clock dt로는 속도가 왜곡되므로
    // 메시지 스탬프(sim time) 차분을 우선 쓰고, 스탬프가 없거나 역행하면
    // 수신 시각 차분으로 폴백한다.
    double dt =
      std::chrono::duration<double>(stamp - last_torpedo_stamp_).count();
    if (message_stamp.nanoseconds() > 0 &&
      last_torpedo_msg_stamp_.nanoseconds() > 0)
    {
      const double message_dt =
        (message_stamp - last_torpedo_msg_stamp_).seconds();
      if (message_dt > 0.0) {
        dt = message_dt;
      }
    }
    if (dt > 1.0e-3 && dt < 2.0) {
      const Point3D raw{
        (position.x - last_torpedo_position_.x) / dt,
        (position.y - last_torpedo_position_.y) / dt,
        (position.z - last_torpedo_position_.z) / dt};
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

  last_torpedo_position_ = position;
  last_torpedo_stamp_ = stamp;
  last_torpedo_msg_stamp_ = message_stamp;
  last_torpedo_frame_ = frame;
  last_torpedo_sequence_ = sample.metadata.sequence;
  have_torpedo_history_ = true;
}

bool PlanningModule::needsReplan(
  const PlanningRequest & request,
  const std::vector<BoxObstacle> & obstacles) const
{
  if (!last_requested_) {
    return true;
  }
  if (request.frame_id != last_requested_->frame_id) {
    return true;
  }

  if (!config_.replan.collision_only) {
    // 기존 거리 기반 정책이다.
    return distance3D(request.start, last_requested_->start) >=
             config_.robot_replan_distance ||
           distance3D(request.torpedo, last_requested_->torpedo) >=
             config_.torpedo_replan_distance ||
           distance3D(request.goal, last_requested_->goal) >=
             config_.goal_replan_distance;
  }

  // 히스테리시스 정책: 충돌·이탈·목표 변경 때만 재계획한다.
  // 계획 실패가 반복돼도 5Hz로 폭주하지 않도록 요청 간격을 제한한다.
  const double since_last_request = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_replan_request_time_).count();
  if (since_last_request < config_.replan.min_interval_sec) {
    return false;
  }

  if (last_path_.empty()) {
    return true;  // 유효한 경로가 아직 없다(직전 계획 실패 포함).
  }
  // 경로 끝점이 현재 목표와 다르면 목표가 바뀌었거나 직전 계획이
  // 실패한 채 남은 옛 경로라는 뜻이므로 다시 계획한다.
  if (distance3D(request.goal, last_path_.back()) >=
    config_.goal_replan_distance)
  {
    return true;
  }

  // 로봇에서 가장 가까운 waypoint를 찾아 경로 이탈 여부를 본다.
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < last_path_.size(); ++index) {
    const double candidate = distance3D(request.start, last_path_[index]);
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
  for (std::size_t index = closest_index; index < last_path_.size(); ++index) {
    for (const auto & obstacle : obstacles) {
      if (boxContains(obstacle, last_path_[index], margin)) {
        return true;
      }
    }
  }
  return false;
}

void PlanningModule::update(const common::StateSnapshot & snapshot)
{
  // 미션 목표가 아직 없어도 어뢰 속도 추정은 계속 워밍업한다.
  updateTorpedoVelocity(snapshot.torpedo_odometry);

  if (snapshot.bluerov_odometry.metadata.valid) {
    visualizer_->publishBlueRov(snapshot.bluerov_odometry.message);
  }
  if (snapshot.torpedo_odometry.metadata.valid) {
    BoxObstacle live_barrier = config_.torpedo_barrier;
    live_barrier.center = positionOf(snapshot.torpedo_odometry.message);
    visualizer_->publishTorpedo(
      snapshot.torpedo_odometry.message, live_barrier, config_.enabled);
  }
  visualizer_->publishTelemetry(snapshot);

  if (!config_.enabled ||
    !snapshot.bluerov_odometry.metadata.valid ||
    !snapshot.torpedo_odometry.metadata.valid)
  {
    return;
  }

  const auto & bluerov = snapshot.bluerov_odometry.message;
  const auto & torpedo = snapshot.torpedo_odometry.message;
  const std::string frame_id =
    bluerov.header.frame_id.empty() ? "map" : bluerov.header.frame_id;
  if (!framesCompatible(frame_id, torpedo.header.frame_id)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "team_min frame mismatch: BlueROV='%s', torpedo='%s'",
      frame_id.c_str(), torpedo.header.frame_id.c_str());
    return;
  }

  const Point3D start = positionOf(bluerov);
  const Point3D torpedo_position = positionOf(torpedo);
  Point3D goal;
  if (config_.use_target_topic_for_goal) {
    if (!snapshot.mission_goal.metadata.valid) {
      return;
    }
    const auto & target = snapshot.mission_goal.message;
    if (!framesCompatible(frame_id, target.header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min target frame mismatch: planning='%s', target='%s'",
        frame_id.c_str(), target.header.frame_id.c_str());
      return;
    }
    goal = {target.point.x, target.point.y, target.point.z};
  } else {
    if (!fixed_goal_) {
      fixed_goal_ = Point3D{
        start.x + config_.goal_offset_x,
        start.y + config_.goal_offset_y,
        start.z + config_.goal_offset_z};
    }
    goal = *fixed_goal_;
  }

  const Point3D velocity_estimate =
    have_torpedo_velocity_ ? torpedo_velocity_ : Point3D{};
  const PlanningRequest request{
    start, goal, torpedo_position, velocity_estimate, frame_id};
  // 충돌 검사용 통로는 execute()와 같은 함수로 만들어 판정을 일치시킨다.
  const auto obstacles = buildTorpedoObstacles(request);
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (!needsReplan(request, obstacles)) {
      return;
    }
    pending_request_ = request;
    last_requested_ = request;
    last_replan_request_time_ = std::chrono::steady_clock::now();
  }
  request_cv_.notify_one();
}

// 어뢰의 진행방향 앞에 겹치는 예측 박스를 깔아 통로(corridor)를 만든다.
// A*가 미래 궤적 전체를 한 번에 우회하므로 어뢰가 예측대로 움직이는 동안
// 경로가 흔들리지 않는다. obstacles[0]은 항상 현재 위치 박스다.
std::vector<BoxObstacle> PlanningModule::buildTorpedoObstacles(
  const PlanningRequest & request) const
{
  std::vector<BoxObstacle> obstacles;
  BoxObstacle current = config_.torpedo_barrier;
  current.center = request.torpedo;
  obstacles.push_back(current);

  const PredictionConfig & prediction = config_.prediction;
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
    config_.astar.safety_margin + prediction.start_clearance;
  for (int i = 1; i <= count; ++i) {
    BoxObstacle box = config_.torpedo_barrier;
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

GridMapConfig PlanningModule::planningMap(
  const PlanningRequest & request,
  const std::vector<BoxObstacle> & obstacles) const
{
  if (!config_.use_dynamic_map) {
    return config_.fixed_map;
  }

  GridMapConfig map = config_.fixed_map;
  double min_x = std::min(request.start.x, request.goal.x);
  double max_x = std::max(request.start.x, request.goal.x);
  double min_y = std::min(request.start.y, request.goal.y);
  double max_y = std::max(request.start.y, request.goal.y);
  double min_z = std::min(request.start.z, request.goal.z);
  double max_z = std::max(request.start.z, request.goal.z);
  // 예측 통로 박스가 맵 경계에서 잘려 못 막는 일이 없도록 범위에 포함한다.
  for (const auto & obstacle : obstacles) {
    const double half_x = obstacle.size_x * 0.5 + config_.astar.safety_margin;
    const double half_y = obstacle.size_y * 0.5 + config_.astar.safety_margin;
    const double half_z = obstacle.size_z * 0.5 + config_.astar.safety_margin;
    min_x = std::min(min_x, obstacle.center.x - half_x);
    max_x = std::max(max_x, obstacle.center.x + half_x);
    min_y = std::min(min_y, obstacle.center.y - half_y);
    max_y = std::max(max_y, obstacle.center.y + half_y);
    min_z = std::min(min_z, obstacle.center.z - half_z);
    max_z = std::max(max_z, obstacle.center.z + half_z);
  }
  map.min_x = min_x - config_.map_padding_x;
  map.max_x = max_x + config_.map_padding_x;
  map.min_y = min_y - config_.map_padding_y;
  map.max_y = max_y + config_.map_padding_y;
  map.min_z = min_z - config_.map_padding_z;
  map.max_z = max_z + config_.map_padding_z;
  return map;
}

void PlanningModule::workerLoop()
{
  while (running_.load()) {
    std::optional<PlanningRequest> request;
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

void PlanningModule::execute(const PlanningRequest & request)
{
  const auto calculation_start = std::chrono::steady_clock::now();
  try {
    const auto obstacles = buildTorpedoObstacles(request);
    // 어뢰 현재 위치 박스가 start/goal을 덮으면 A*는 성공할 수 없다.
    // 일반 예외 대신 원인을 명시하고, 어뢰가 지나간 뒤의 재시도(경로가
    // 없으므로 needsReplan이 계속 true)에 맡긴다.
    const auto & live_barrier = obstacles.front();
    if (boxContains(live_barrier, request.goal, config_.astar.safety_margin)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min goal (%.1f, %.1f, %.1f) is inside the torpedo barrier; "
        "waiting for the torpedo to move away",
        request.goal.x, request.goal.y, request.goal.z);
      return;
    }
    if (boxContains(live_barrier, request.start, config_.astar.safety_margin)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "team_min start is inside the torpedo barrier; cannot plan until "
        "the torpedo moves away");
      return;
    }
    const auto path = runEnhancedAStar3D(
      request.start,
      request.goal,
      planningMap(request, obstacles),
      obstacles,
      config_.astar);
    if (path.empty()) {
      RCLCPP_WARN(logger_, "team_min A* found no path");
      return;
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
