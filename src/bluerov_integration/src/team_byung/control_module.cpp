#include "bluerov_integration/team_byung/control_module.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace bluerov_integration::team_byung
{

ControlModule::ControlModule(
  rclcpp::Node & node,
  ControlModuleConfig config)
: config_(std::move(config)),
  logger_(node.get_logger()),
  clock_(node.get_clock()),
  controller_(config_.controller)
{
  if (config_.velocity_source != "odometry" &&
    config_.velocity_source != "dvl")
  {
    throw std::invalid_argument(
            "control.velocity_source must be 'odometry' or 'dvl'");
  }
  if (config_.nominal_dt <= 0.0 ||
    config_.odometry_timeout_sec <= 0.0 ||
    config_.dvl_timeout_sec <= 0.0)
  {
    throw std::invalid_argument("Control timing parameters must be positive");
  }

  for (std::size_t index = 0; index < thruster_pubs_.size(); ++index) {
    thruster_pubs_[index] =
      node.create_publisher<std_msgs::msg::Float64>(
      config_.thruster_topics[index], rclcpp::QoS(10).reliable());
  }
  ppid_logger_ = std::make_unique<PpidLogger>(node, config_.logger);
  ppid_logger_->start();
  publishZero();

  RCLCPP_INFO(
    logger_, "team_byung PPID module %s, velocity source=%s",
    config_.enabled ? "enabled" : "disabled",
    config_.velocity_source.c_str());
}

ControlModule::~ControlModule()
{
  stop();
}

bool ControlModule::finiteTarget(
  const geometry_msgs::msg::PointStamped & target) const
{
  return std::isfinite(target.point.x) &&
         std::isfinite(target.point.y) &&
         std::isfinite(target.point.z);
}

bool ControlModule::targetFrameMatches(
  const geometry_msgs::msg::PointStamped & target,
  const nav_msgs::msg::Odometry & odometry) const
{
  return !config_.require_matching_target_frame ||
         target.header.frame_id.empty() ||
         odometry.header.frame_id.empty() ||
         target.header.frame_id == odometry.header.frame_id;
}

void ControlModule::update(
  const common::StateSnapshot & snapshot,
  const common::ControlTarget & tracking_target)
{
  if (!config_.enabled || stopped_) {
    return;
  }

  const auto steady_now = std::chrono::steady_clock::now();
  if (!common::isFresh(
      snapshot.bluerov_odometry.metadata,
      steady_now,
      config_.odometry_timeout_sec))
  {
    deactivate("BlueROV odometry missing or stale");
    return;
  }
  if (!tracking_target.valid ||
    !common::isFresh(
      tracking_target.source_metadata,
      steady_now,
      config_.target_timeout_sec))
  {
    deactivate("tracking target missing or stale");
    return;
  }

  const auto & odometry = snapshot.bluerov_odometry.message;
  const auto & target = tracking_target.message;
  if (!finiteTarget(target)) {
    deactivate("target position contains non-finite value");
    return;
  }
  if (!targetFrameMatches(target, odometry)) {
    deactivate("target and odometry frames differ");
    return;
  }

  // Path follower integration: begin
  const bool mission_updated =
    !have_target_ ||
    mission_sequence_ != tracking_target.mission_sequence;
  const bool target_updated =
    !have_target_ ||
    tracking_sequence_ != tracking_target.tracking_sequence;
  if (mission_updated) {
    controller_.reset();
  }
  if (target_updated)
  {
    // 민 팀이 만든 추종 목표를 병 팀 PPID에 넣는 실제 연결 한 줄.
    controller_.setTargetPosition({tracking_target.message.point.x, tracking_target.message.point.y, tracking_target.message.point.z});
    mission_sequence_ = tracking_target.mission_sequence;
    tracking_sequence_ = tracking_target.tracking_sequence;
    have_target_ = true;
    RCLCPP_INFO(
      logger_,
      "team_byung tracking target=(%.3f, %.3f, %.3f), waypoint=%zu, final=%d",
      target.point.x, target.point.y, target.point.z,
      tracking_target.waypoint_index,
      tracking_target.final_goal);
  }
  // Path follower integration: end

  Vector3 current_velocity{
    odometry.twist.twist.linear.x,
    odometry.twist.twist.linear.y,
    odometry.twist.twist.linear.z};
  if (config_.velocity_source == "dvl") {
    if (!common::isFresh(
        snapshot.dvl.metadata, steady_now, config_.dvl_timeout_sec))
    {
      deactivate("DVL velocity missing or stale");
      return;
    }
    current_velocity = {
      snapshot.dvl.message.velocity.twist.linear.x,
      snapshot.dvl.message.velocity.twist.linear.y,
      snapshot.dvl.message.velocity.twist.linear.z};
  }

  double dt = config_.nominal_dt;
  if (have_previous_tick_) {
    dt = std::chrono::duration<double>(steady_now - previous_tick_).count();
  }
  previous_tick_ = steady_now;
  have_previous_tick_ = true;

  const OdometryState state{
    {odometry.pose.pose.position.x,
     odometry.pose.pose.position.y,
     odometry.pose.pose.position.z},
    current_velocity,
    {odometry.pose.pose.orientation.x,
     odometry.pose.pose.orientation.y,
     odometry.pose.pose.orientation.z,
     odometry.pose.pose.orientation.w},
    odometry.twist.twist.angular.z};

  const auto commands = controller_.update(state, dt);
  publish(commands);
  active_ = true;
  ppid_logger_->enqueue(controller_.telemetry(), target_updated);

  const auto & data = controller_.telemetry();
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "PPID dt=%.4f target=(%.2f %.2f %.2f) current=(%.2f %.2f %.2f) "
    "motor=[%.1f %.1f %.1f %.1f %.1f %.1f]",
    data.control_dt_sec,
    data.target_position_world.x,
    data.target_position_world.y,
    data.target_position_world.z,
    data.current_position_world.x,
    data.current_position_world.y,
    data.current_position_world.z,
    commands[0], commands[1], commands[2],
    commands[3], commands[4], commands[5]);
}

void ControlModule::publish(const std::array<double, 6> & commands)
{
  for (std::size_t index = 0; index < thruster_pubs_.size(); ++index) {
    std_msgs::msg::Float64 message;
    message.data = commands[index];
    thruster_pubs_[index]->publish(message);
  }
}

void ControlModule::publishZero()
{
  publish({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void ControlModule::deactivate(const char * reason)
{
  if (active_) {
    publishZero();
    controller_.reset();
    active_ = false;
    have_previous_tick_ = false;
  }
  RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "PPID idle: %s", reason);
}

void ControlModule::stop()
{
  if (stopped_) {
    return;
  }
  stopped_ = true;
  publishZero();
  if (ppid_logger_) {
    ppid_logger_->finish();
  }
}

std::uint64_t ControlModule::loggerOverflowCount() const
{
  return ppid_logger_ ? ppid_logger_->overflowCount() : 0U;
}

}  // namespace bluerov_integration::team_byung
