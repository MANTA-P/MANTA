#include "torpedo_control_v2/state_receiver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace torpedo_control_v2
{

StateReceiver::StateReceiver(rclcpp::Node & node, Config config)
: config_(std::move(config))
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = config_.callback_group;

  odometry_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    config_.odometry_topic,
    rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      if (message) {
        on_odometry(*message);
      }
    },
    options);

  target_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    config_.target_odometry_topic,
    rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      if (message) {
        on_target_odometry(*message);
      }
    },
    options);

  joints_sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_states_topic,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      if (message) {
        on_joint_states(*message);
      }
    },
    options);
}

double StateReceiver::message_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

double StateReceiver::wall_time_now_sec()
{
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

void StateReceiver::on_odometry(const nav_msgs::msg::Odometry & message)
{
  OdometryState sample;
  sample.valid = true;
  sample.stamp_sec = message_stamp(message.header.stamp);
  sample.position_x = message.pose.pose.position.x;
  sample.position_y = message.pose.pose.position.y;
  sample.position_z = message.pose.pose.position.z;
  sample.orientation_x = message.pose.pose.orientation.x;
  sample.orientation_y = message.pose.pose.orientation.y;
  sample.orientation_z = message.pose.pose.orientation.z;
  sample.orientation_w = message.pose.pose.orientation.w;
  sample.linear_x = message.twist.twist.linear.x;
  sample.linear_y = message.twist.twist.linear.y;
  sample.linear_z = message.twist.twist.linear.z;
  sample.angular_x = message.twist.twist.angular.x;
  sample.angular_y = message.twist.twist.angular.y;
  sample.angular_z = message.twist.twist.angular.z;

  RawSensorFrame raw;
  raw.kind = SensorKind::ODOMETRY;
  raw.received_wall_time_sec = wall_time_now_sec();
  raw.odometry = sample;

  std::lock_guard<std::mutex> lock(mutex_);
  latest_.odometry = sample;
  push_raw_sample_locked(std::move(raw));
}

void StateReceiver::on_target_odometry(const nav_msgs::msg::Odometry & message)
{
  TargetState sample;
  sample.valid = true;
  sample.stamp_sec = message_stamp(message.header.stamp);
  sample.x = message.pose.pose.position.x;
  sample.y = message.pose.pose.position.y;
  sample.z = message.pose.pose.position.z;

  RawSensorFrame raw;
  raw.kind = SensorKind::TARGET_ODOMETRY;
  raw.received_wall_time_sec = wall_time_now_sec();
  raw.target = sample;

  std::lock_guard<std::mutex> lock(mutex_);
  latest_.target = sample;
  push_raw_sample_locked(std::move(raw));
}

void StateReceiver::on_joint_states(const sensor_msgs::msg::JointState & message)
{
  JointStateState sample;
  sample.stamp_sec = message_stamp(message.header.stamp);

  for (std::size_t index = 0; index < message.name.size(); ++index) {
    const double position = index < message.position.size() ?
      message.position[index] : kNaN;
    const double velocity = index < message.velocity.size() ?
      message.velocity[index] : kNaN;
    const auto & name = message.name[index];

    if (name == config_.propeller_front_joint_name) {
      sample.propeller_front_position = position;
      sample.propeller_front_velocity = velocity;
      sample.valid = sample.valid || std::isfinite(position) || std::isfinite(velocity);
    } else if (name == config_.propeller_rear_joint_name) {
      sample.propeller_rear_position = position;
      sample.propeller_rear_velocity = velocity;
      sample.valid = sample.valid || std::isfinite(position) || std::isfinite(velocity);
    } else if (name == config_.fin_top_joint_name) {
      sample.fin_top = position;
      sample.valid = sample.valid || std::isfinite(position);
    } else if (name == config_.fin_bottom_joint_name) {
      sample.fin_bottom = position;
      sample.valid = sample.valid || std::isfinite(position);
    } else if (name == config_.fin_left_joint_name) {
      sample.fin_left = position;
      sample.valid = sample.valid || std::isfinite(position);
    } else if (name == config_.fin_right_joint_name) {
      sample.fin_right = position;
      sample.valid = sample.valid || std::isfinite(position);
    }
  }

  RawSensorFrame raw;
  raw.kind = SensorKind::JOINT_STATES;
  raw.received_wall_time_sec = wall_time_now_sec();
  raw.joints = sample;

  std::lock_guard<std::mutex> lock(mutex_);
  latest_.joints = sample;
  push_raw_sample_locked(std::move(raw));
}

void StateReceiver::push_raw_sample_locked(RawSensorFrame sample)
{
  if (config_.raw_queue_capacity == 0U) {
    ++raw_overflow_count_;
    return;
  }
  if (raw_samples_.size() >= config_.raw_queue_capacity) {
    raw_samples_.pop_front();
    ++raw_overflow_count_;
  }
  raw_samples_.push_back(std::move(sample));
}

StateSnapshot StateReceiver::snapshot() const
{
  StateSnapshot copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    copy = latest_;
  }
  copy.copied_wall_time_sec = wall_time_now_sec();
  return copy;
}

std::deque<RawSensorFrame> StateReceiver::drain_raw_samples()
{
  std::deque<RawSensorFrame> samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples.swap(raw_samples_);
  }
  return samples;
}

std::size_t StateReceiver::raw_overflow_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return raw_overflow_count_;
}

}  // namespace torpedo_control_v2
