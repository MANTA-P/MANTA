#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <variant>

namespace torpedo_control_v2
{

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct OdometryState
{
  bool valid{false};
  double stamp_sec{kNaN};
  double position_x{kNaN};
  double position_y{kNaN};
  double position_z{kNaN};
  double orientation_x{kNaN};
  double orientation_y{kNaN};
  double orientation_z{kNaN};
  double orientation_w{kNaN};
  double linear_x{kNaN};
  double linear_y{kNaN};
  double linear_z{kNaN};
  double angular_x{kNaN};
  double angular_y{kNaN};
  double angular_z{kNaN};
};

struct TargetState
{
  bool valid{false};
  double stamp_sec{kNaN};
  double x{kNaN};
  double y{kNaN};
  double z{kNaN};
};

struct JointStateState
{
  bool valid{false};
  double stamp_sec{kNaN};
  double propeller_front_position{kNaN};
  double propeller_front_velocity{kNaN};
  double propeller_rear_position{kNaN};
  double propeller_rear_velocity{kNaN};
  double fin_top{kNaN};
  double fin_bottom{kNaN};
  double fin_left{kNaN};
  double fin_right{kNaN};
};

struct StateSnapshot
{
  double copied_wall_time_sec{kNaN};
  OdometryState odometry;
  TargetState target;
  JointStateState joints;
};

enum class SensorKind
{
  ODOMETRY,
  TARGET_ODOMETRY,
  JOINT_STATES
};

// A raw callback sample. It is copied into the receiver queue once and then
// moved to the asynchronous logger. No ROS message reference crosses threads.
struct RawSensorFrame
{
  SensorKind kind{SensorKind::ODOMETRY};
  double received_wall_time_sec{kNaN};
  OdometryState odometry;
  TargetState target;
  JointStateState joints;
};

struct KeyEvent
{
  char key{'\0'};
  double monotonic_time_sec{kNaN};
};

struct ControllerConfig
{
  std::string thrust_topic;
  std::string fin_top_topic;
  std::string fin_bottom_topic;
  std::string fin_left_topic;
  std::string fin_right_topic;
  std::string odometry_topic;
  std::string target_odometry_topic;
  std::string joint_states_topic;
  std::string propeller_front_joint_name;
  std::string propeller_rear_joint_name;
  std::string fin_top_joint_name;
  std::string fin_bottom_joint_name;
  std::string fin_left_joint_name;
  std::string fin_right_joint_name;

  char nose_push_key{'w'};
  char nose_pull_key{'s'};
  char yaw_left_key{'a'};
  char yaw_right_key{'d'};
  char thrust_increase_key{'r'};
  char thrust_decrease_key{'f'};
  char thrust_stop_key{' '};
  char auto_track_key{'t'};

  double thrust_initial{0.0};
  double thrust_step{1500.0};
  double thrust_min{0.0};
  double thrust_max{1500.0};
  double fin_command_rad{0.05};
  double fin_limit_rad{0.50};
  double fin_min_rad{-0.50};
  double fin_max_rad{0.50};
  double publish_rate_hz{20.0};
  double p_gain_pitch{0.8};
  double p_gain_yaw{0.8};

  bool roll_control_enabled{true};
  double roll_kp{0.6};
  double roll_kd{0.15};
  double roll_limit_rad{0.10};
  double roll_input_sign{-1.0};
  double roll_top_sign{1.0};
  double roll_bottom_sign{-1.0};

  double thrust_output_sign{1.0};
  double pitch_input_sign{1.0};
  double yaw_input_sign{1.0};
  double pitch_left_sign{1.0};
  double pitch_right_sign{1.0};
  double yaw_top_sign{1.0};
  double yaw_bottom_sign{1.0};

  double fin_top_neutral_rad{0.0};
  double fin_bottom_neutral_rad{0.0};
  double fin_left_neutral_rad{0.0};
  double fin_right_neutral_rad{0.0};

  bool log_enabled{true};
  std::string log_directory{"~/.ros/torpedo_control_v2_logs"};
  int log_flush_every_n_rows{1};
  std::size_t sensor_queue_capacity{8192U};
  std::size_t key_queue_capacity{1024U};
  std::size_t logger_queue_capacity{100000U};
};

struct ActuatorCommand
{
  double thrust{0.0};
  double fin_top{0.0};
  double fin_bottom{0.0};
  double fin_left{0.0};
  double fin_right{0.0};
  double pitch_target_rad{0.0};
  double yaw_target_rad{0.0};
  double roll_correction_applied_rad{0.0};
  double roll_mix_scale{1.0};
};

struct ControlResult
{
  ActuatorCommand command;
  double thrust_level{0.0};
  double pitch_error_rad{kNaN};
  double yaw_error_rad{kNaN};
  double roll_angle_rad{kNaN};
  double roll_rate_rad_per_sec{kNaN};
  double roll_correction_rad{0.0};
  bool auto_track{false};
  std::string events;
};

struct TelemetryFrame
{
  double wall_time_unix_sec{kNaN};
  double elapsed_sec{kNaN};
  double ros_time_sec{kNaN};
  std::string key_events{"NONE"};
  ControlResult control;
  StateSnapshot state;
  std::size_t keyboard_overflow_count{0U};
  std::size_t sensor_overflow_count{0U};
  std::size_t logger_overflow_count{0U};
};

using LogRecord = std::variant<TelemetryFrame, RawSensorFrame>;

}  // namespace torpedo_control_v2
