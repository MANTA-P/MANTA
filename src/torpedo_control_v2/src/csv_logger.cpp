#include "torpedo_control_v2/csv_logger.hpp"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace torpedo_control_v2
{

CsvLogger::CsvLogger(Config config)
: config_(std::move(config))
{
}

CsvLogger::~CsvLogger()
{
  stop();
}

void CsvLogger::start()
{
  if (!config_.enabled || started_) {
    return;
  }
  if (config_.flush_every_n_rows <= 0) {
    throw std::runtime_error("log_flush_every_n_rows must be greater than zero");
  }

  open_files();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = true;
    stop_requested_ = false;
  }
  worker_ = std::thread(&CsvLogger::worker_loop, this);
}

void CsvLogger::stop()
{
  if (!config_.enabled) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) {
      return;
    }
    stop_requested_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  control_file_.flush();
  sensor_file_.flush();
  control_file_.close();
  sensor_file_.close();
  started_ = false;
}

bool CsvLogger::enqueue(LogRecord record)
{
  if (!config_.enabled) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stop_requested_) {
      return false;
    }
    if (config_.queue_capacity == 0U) {
      ++overflow_count_;
      return false;
    }
    if (queue_.size() >= config_.queue_capacity) {
      queue_.pop_front();
      ++overflow_count_;
    }
    queue_.push_back(std::move(record));
  }
  condition_.notify_one();
  return true;
}

std::size_t CsvLogger::overflow_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return overflow_count_;
}

const std::string & CsvLogger::control_log_path() const
{
  return control_log_path_;
}

const std::string & CsvLogger::sensor_log_path() const
{
  return sensor_log_path_;
}

std::string CsvLogger::expand_user_path(const std::string & path)
{
  if (path == "~" || path.rfind("~/", 0) == 0U) {
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      throw std::runtime_error("HOME is not set; cannot expand log_directory");
    }
    return std::string(home) + path.substr(1);
  }
  return path;
}

double CsvLogger::wall_time_now_sec()
{
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

std::string CsvLogger::sensor_kind_name(const SensorKind kind)
{
  switch (kind) {
    case SensorKind::ODOMETRY:
      return "odometry";
    case SensorKind::TARGET_ODOMETRY:
      return "target_odometry";
    case SensorKind::JOINT_STATES:
      return "joint_states";
  }
  return "unknown";
}

void CsvLogger::open_files()
{
  const std::filesystem::path directory(expand_user_path(config_.directory));
  std::filesystem::create_directories(directory);

  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream suffix;
  suffix << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  control_log_path_ = (directory / ("torpedo_control_cycle_" + suffix.str() + ".csv")).string();
  sensor_log_path_ = (directory / ("torpedo_sensor_samples_" + suffix.str() + ".csv")).string();

  control_file_.open(control_log_path_, std::ios::out | std::ios::trunc);
  sensor_file_.open(sensor_log_path_, std::ios::out | std::ios::trunc);
  if (!control_file_.is_open() || !sensor_file_.is_open()) {
    throw std::runtime_error("Cannot open CSV log files in " + directory.string());
  }

  control_file_ << std::fixed << std::setprecision(9);
  sensor_file_ << std::fixed << std::setprecision(9);
  write_control_header();
  write_sensor_header();
}

void CsvLogger::write_control_header()
{
  control_file_ <<
    "wall_time_unix_sec,elapsed_sec,ros_time_sec,key_events,auto_track,"
    "thrust_level,thrust_command,pitch_target_rad,yaw_target_rad,"
    "fin_top_command_rad,fin_bottom_command_rad,fin_left_command_rad,"
    "fin_right_command_rad,pitch_error_rad,yaw_error_rad,roll_angle_rad,"
    "roll_rate_rad_per_sec,roll_correction_requested_rad,"
    "roll_correction_applied_rad,roll_mix_scale,"
    "odom_valid,odom_stamp_sec,odom_position_x,odom_position_y,odom_position_z,"
    "odom_orientation_x,odom_orientation_y,odom_orientation_z,odom_orientation_w,"
    "odom_twist_linear_x,odom_twist_linear_y,odom_twist_linear_z,"
    "odom_twist_angular_x,odom_twist_angular_y,odom_twist_angular_z,"
    "joint_states_valid,joint_stamp_sec,joint_fin_top_position_rad,"
    "joint_fin_bottom_position_rad,joint_fin_left_position_rad,"
    "joint_fin_right_position_rad,joint_propeller_front_position_rad,"
    "joint_propeller_front_velocity_rad_per_sec,joint_propeller_rear_position_rad,"
    "joint_propeller_rear_velocity_rad_per_sec,target_valid,target_stamp_sec,"
    "target_x,target_y,target_z,keyboard_overflow_count,sensor_overflow_count,"
    "logger_overflow_count\n";
}

void CsvLogger::write_sensor_header()
{
  sensor_file_ <<
    "received_wall_time_unix_sec,source,stamp_sec,valid,"
    "position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,"
    "orientation_w,twist_linear_x,twist_linear_y,twist_linear_z,"
    "twist_angular_x,twist_angular_y,twist_angular_z,target_x,target_y,target_z,"
    "joint_fin_top_position_rad,joint_fin_bottom_position_rad,joint_fin_left_position_rad,"
    "joint_fin_right_position_rad,joint_propeller_front_position_rad,"
    "joint_propeller_front_velocity_rad_per_sec,joint_propeller_rear_position_rad,"
    "joint_propeller_rear_velocity_rad_per_sec\n";
}

void CsvLogger::worker_loop()
{
  while (true) {
    std::deque<LogRecord> batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() { return stop_requested_ || !queue_.empty(); });
      if (queue_.empty() && stop_requested_) {
        break;
      }
      batch.swap(queue_);
    }

    for (auto & record : batch) {
      std::visit([this](const auto & value) { write_record(value); }, record);
    }
  }
}

void CsvLogger::write_record(const TelemetryFrame & frame)
{
  if (!control_file_.is_open()) {
    return;
  }
  const auto & command = frame.control.command;
  const auto & odometry = frame.state.odometry;
  const auto & joints = frame.state.joints;
  const auto & target = frame.state.target;

  control_file_ << frame.wall_time_unix_sec << ',' << frame.elapsed_sec << ',' <<
    frame.ros_time_sec << ',' << frame.key_events << ',' << (frame.control.auto_track ? 1 : 0) << ',' <<
    frame.control.thrust_level << ',' << command.thrust << ',' << command.pitch_target_rad << ',' <<
    command.yaw_target_rad << ',' << command.fin_top << ',' << command.fin_bottom << ',' <<
    command.fin_left << ',' << command.fin_right << ',' << frame.control.pitch_error_rad << ',' <<
    frame.control.yaw_error_rad << ',' << frame.control.roll_angle_rad << ',' <<
    frame.control.roll_rate_rad_per_sec << ',' << frame.control.roll_correction_rad << ',' <<
    command.roll_correction_applied_rad << ',' << command.roll_mix_scale << ',' <<
    (odometry.valid ? 1 : 0) << ',' << odometry.stamp_sec << ',' <<
    odometry.position_x << ',' << odometry.position_y << ',' << odometry.position_z << ',' <<
    odometry.orientation_x << ',' << odometry.orientation_y << ',' << odometry.orientation_z << ',' <<
    odometry.orientation_w << ',' << odometry.linear_x << ',' << odometry.linear_y << ',' <<
    odometry.linear_z << ',' << odometry.angular_x << ',' << odometry.angular_y << ',' <<
    odometry.angular_z << ',' << (joints.valid ? 1 : 0) << ',' << joints.stamp_sec << ',' <<
    joints.fin_top << ',' << joints.fin_bottom << ',' << joints.fin_left << ',' << joints.fin_right << ',' <<
    joints.propeller_front_position << ',' << joints.propeller_front_velocity << ',' <<
    joints.propeller_rear_position << ',' << joints.propeller_rear_velocity << ',' <<
    (target.valid ? 1 : 0) << ',' << target.stamp_sec << ',' << target.x << ',' << target.y << ',' <<
    target.z << ',' << frame.keyboard_overflow_count << ',' << frame.sensor_overflow_count << ',' <<
    frame.logger_overflow_count << '\n';

  ++control_row_count_;
  if (control_row_count_ % static_cast<std::size_t>(config_.flush_every_n_rows) == 0U) {
    control_file_.flush();
  }
}

void CsvLogger::write_record(const RawSensorFrame & frame)
{
  if (!sensor_file_.is_open()) {
    return;
  }
  const auto & odometry = frame.odometry;
  const auto & target = frame.target;
  const auto & joints = frame.joints;
  const bool is_odom = frame.kind == SensorKind::ODOMETRY;
  const bool is_target = frame.kind == SensorKind::TARGET_ODOMETRY;
  const bool is_joint = frame.kind == SensorKind::JOINT_STATES;

  double stamp = kNaN;
  bool valid = false;
  if (is_odom) {
    stamp = odometry.stamp_sec;
    valid = odometry.valid;
  } else if (is_target) {
    stamp = target.stamp_sec;
    valid = target.valid;
  } else if (is_joint) {
    stamp = joints.stamp_sec;
    valid = joints.valid;
  }

  sensor_file_ << frame.received_wall_time_sec << ',' << sensor_kind_name(frame.kind) << ',' <<
    stamp << ',' << (valid ? 1 : 0) << ',' <<
    (is_odom ? odometry.position_x : kNaN) << ',' <<
    (is_odom ? odometry.position_y : kNaN) << ',' <<
    (is_odom ? odometry.position_z : kNaN) << ',' <<
    (is_odom ? odometry.orientation_x : kNaN) << ',' <<
    (is_odom ? odometry.orientation_y : kNaN) << ',' <<
    (is_odom ? odometry.orientation_z : kNaN) << ',' <<
    (is_odom ? odometry.orientation_w : kNaN) << ',' <<
    (is_odom ? odometry.linear_x : kNaN) << ',' <<
    (is_odom ? odometry.linear_y : kNaN) << ',' <<
    (is_odom ? odometry.linear_z : kNaN) << ',' <<
    (is_odom ? odometry.angular_x : kNaN) << ',' <<
    (is_odom ? odometry.angular_y : kNaN) << ',' <<
    (is_odom ? odometry.angular_z : kNaN) << ',' <<
    (is_target ? target.x : kNaN) << ',' <<
    (is_target ? target.y : kNaN) << ',' <<
    (is_target ? target.z : kNaN) << ',' <<
    (is_joint ? joints.fin_top : kNaN) << ',' <<
    (is_joint ? joints.fin_bottom : kNaN) << ',' <<
    (is_joint ? joints.fin_left : kNaN) << ',' <<
    (is_joint ? joints.fin_right : kNaN) << ',' <<
    (is_joint ? joints.propeller_front_position : kNaN) << ',' <<
    (is_joint ? joints.propeller_front_velocity : kNaN) << ',' <<
    (is_joint ? joints.propeller_rear_position : kNaN) << ',' <<
    (is_joint ? joints.propeller_rear_velocity : kNaN) << '\n';

  ++sensor_row_count_;
  if (sensor_row_count_ % static_cast<std::size_t>(config_.flush_every_n_rows) == 0U) {
    sensor_file_.flush();
  }
}

}  // namespace torpedo_control_v2
