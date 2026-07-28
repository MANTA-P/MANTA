#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "torpedo_control_v2/control_types.hpp"

namespace torpedo_control_v2
{

class StateReceiver
{
public:
  struct Config
  {
    std::string odometry_topic;
    std::string target_odometry_topic;
    std::string joint_states_topic;
    std::string propeller_front_joint_name;
    std::string propeller_rear_joint_name;
    std::string fin_top_joint_name;
    std::string fin_bottom_joint_name;
    std::string fin_left_joint_name;
    std::string fin_right_joint_name;
    std::size_t raw_queue_capacity{8192U};
    rclcpp::CallbackGroup::SharedPtr callback_group;
  };

  StateReceiver(rclcpp::Node & node, Config config);

  // Copies only the latest state while holding the mutex. All calculations
  // happen after this function returns.
  StateSnapshot snapshot() const;

  // Swaps the raw-sample queue under the mutex, so the caller can process it
  // without keeping the receiver locked.
  std::deque<RawSensorFrame> drain_raw_samples();

  std::size_t raw_overflow_count() const;

private:
  static double message_stamp(const builtin_interfaces::msg::Time & stamp);
  static double wall_time_now_sec();

  void on_odometry(const nav_msgs::msg::Odometry & message);
  void on_target_odometry(const nav_msgs::msg::Odometry & message);
  void on_joint_states(const sensor_msgs::msg::JointState & message);

  void push_raw_sample_locked(RawSensorFrame sample);

  Config config_;
  mutable std::mutex mutex_;
  StateSnapshot latest_;
  std::deque<RawSensorFrame> raw_samples_;
  std::size_t raw_overflow_count_{0U};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
};

}  // namespace torpedo_control_v2
