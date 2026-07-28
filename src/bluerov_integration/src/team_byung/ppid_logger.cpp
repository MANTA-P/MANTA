#include "bluerov_integration/team_byung/ppid_logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "bluerov_integration/team_byung/ppid_log_plot.hpp"

namespace bluerov_integration::team_byung
{

PpidLogger::PpidLogger(rclcpp::Node & node, PpidLoggerConfig config)
: logger_(node.get_logger()),
  clock_(node.get_clock()),
  config_(std::move(config))
{
}

PpidLogger::~PpidLogger()
{
  finish();
}

void PpidLogger::start()
{
  if (!config_.enabled || started_.exchange(true)) {
    return;
  }
  if (config_.queue_limit == 0U) {
    throw std::invalid_argument("PPID logger queue_limit must be positive");
  }

  std::filesystem::create_directories(config_.directory);
  const std::time_t current_time = std::time(nullptr);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &current_time);
#else
  localtime_r(&current_time, &local_time);
#endif
  std::ostringstream name;
  name << "ppid_session_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  csv_path_ = std::filesystem::absolute(
    config_.directory / (name.str() + ".csv"));
  html_path_ = std::filesystem::absolute(
    config_.directory / (name.str() + "_plot.html"));

  log_file_.open(csv_path_, std::ios::out | std::ios::trunc);
  if (!log_file_) {
    started_.store(false);
    throw std::runtime_error("Cannot open PPID log: " + csv_path_.string());
  }
  log_file_ <<
    "wall_time_unix_sec,elapsed_sec,ros_time_sec,event,logger_queue_overflow,"
    "control_dt_sec,"
    "target_position_x,target_position_y,target_position_z,"
    "current_position_x,current_position_y,current_position_z,"
    "position_error_x,position_error_y,position_error_z,"
    "target_velocity_x,target_velocity_y,target_velocity_z,"
    "current_velocity_x,current_velocity_y,current_velocity_z,"
    "velocity_error_x,velocity_error_y,velocity_error_z,"
    "body_command_x,body_command_y,body_command_z,"
    "yaw,yaw_rate,yaw_command,"
    "motor1_command,motor2_command,motor3_command,"
    "motor4_command,motor5_command,motor6_command\n";
  log_file_ << std::fixed << std::setprecision(9);

  start_time_ = std::chrono::steady_clock::now();
  running_.store(true);
  worker_ = std::thread(&PpidLogger::workerLoop, this);
  RCLCPP_INFO(logger_, "PPID async CSV log: %s", csv_path_.c_str());
}

void PpidLogger::enqueue(
  const ControlTelemetry & telemetry,
  const bool target_updated)
{
  if (!running_.load()) {
    return;
  }
  const auto now_steady = std::chrono::steady_clock::now();
  LogEntry entry;
  entry.telemetry = telemetry;
  entry.target_updated = target_updated;
  entry.wall_time_sec = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  entry.elapsed_sec =
    std::chrono::duration<double>(now_steady - start_time_).count();
  entry.ros_time_sec = clock_->now().seconds();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= config_.queue_limit) {
      queue_.pop_front();
      overflow_count_.fetch_add(1);
    }
    entry.queue_overflow = overflow_count_.load();
    queue_.push_back(std::move(entry));
  }
  queue_cv_.notify_one();
}

void PpidLogger::finish()
{
  if (!started_.load()) {
    return;
  }
  running_.store(false);
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (log_file_.is_open()) {
    log_file_.flush();
    log_file_.close();
  }

  if (config_.generate_html && !csv_path_.empty()) {
    try {
      generatePpidHtml(csv_path_, html_path_);
      RCLCPP_INFO(logger_, "PPID HTML report: %s", html_path_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "PPID HTML generation failed: %s", error.what());
    }
  }
  started_.store(false);
}

std::uint64_t PpidLogger::overflowCount() const
{
  return overflow_count_.load();
}

void PpidLogger::workerLoop()
{
  std::size_t rows_since_flush = 0U;
  while (true) {
    std::optional<LogEntry> entry;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !running_.load() || !queue_.empty();
      });
      if (!running_.load() && queue_.empty()) {
        break;
      }
      if (!queue_.empty()) {
        entry = std::move(queue_.front());
        queue_.pop_front();
      }
    }
    if (entry) {
      writeRow(*entry);
      if (++rows_since_flush >= 200U) {
        log_file_.flush();
        rows_since_flush = 0U;
      }
    }
  }
}

void PpidLogger::writeRow(const LogEntry & entry)
{
  const auto & data = entry.telemetry;
  const Vector3 position_error{
    data.target_position_world.x - data.current_position_world.x,
    data.target_position_world.y - data.current_position_world.y,
    data.target_position_world.z - data.current_position_world.z};
  const Vector3 velocity_error{
    data.target_velocity_body.x - data.current_velocity_body.x,
    data.target_velocity_body.y - data.current_velocity_body.y,
    data.target_velocity_body.z - data.current_velocity_body.z};

  log_file_
    << entry.wall_time_sec << ',' << entry.elapsed_sec << ',' <<
    entry.ros_time_sec << ',' <<
    (entry.target_updated ? "TARGET_UPDATED" : "NONE") << ',' <<
    entry.queue_overflow << ',' << data.control_dt_sec << ',' <<
    data.target_position_world.x << ',' << data.target_position_world.y << ',' <<
    data.target_position_world.z << ',' << data.current_position_world.x << ',' <<
    data.current_position_world.y << ',' << data.current_position_world.z << ',' <<
    position_error.x << ',' << position_error.y << ',' << position_error.z << ',' <<
    data.target_velocity_body.x << ',' << data.target_velocity_body.y << ',' <<
    data.target_velocity_body.z << ',' << data.current_velocity_body.x << ',' <<
    data.current_velocity_body.y << ',' << data.current_velocity_body.z << ',' <<
    velocity_error.x << ',' << velocity_error.y << ',' << velocity_error.z << ',' <<
    data.body_command.x << ',' << data.body_command.y << ',' <<
    data.body_command.z << ',' << data.yaw << ',' << data.yaw_rate << ',' <<
    data.yaw_command;
  for (const double command : data.motor_commands) {
    log_file_ << ',' << command;
  }
  log_file_ << '\n';
}

}  // namespace bluerov_integration::team_byung
