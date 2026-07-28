#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/team_byung/ppid_controller.hpp"

namespace bluerov_integration::team_byung
{

struct PpidLoggerConfig
{
  bool enabled{true};
  bool generate_html{true};
  std::filesystem::path directory{"ppid_logs"};
  std::size_t queue_limit{10'000U};
};

class PpidLogger
{
public:
  PpidLogger(rclcpp::Node & node, PpidLoggerConfig config);
  ~PpidLogger();

  void start();
  void enqueue(const ControlTelemetry & telemetry, bool target_updated);
  void finish();
  std::uint64_t overflowCount() const;

private:
  struct LogEntry
  {
    ControlTelemetry telemetry;
    bool target_updated{false};
    double wall_time_sec{0.0};
    double elapsed_sec{0.0};
    double ros_time_sec{0.0};
    std::uint64_t queue_overflow{0};
  };

  void workerLoop();
  void writeRow(const LogEntry & entry);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  PpidLoggerConfig config_;
  std::ofstream log_file_;
  std::filesystem::path csv_path_;
  std::filesystem::path html_path_;
  std::chrono::steady_clock::time_point start_time_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<LogEntry> queue_;
  std::atomic<bool> running_{false};
  std::atomic<bool> started_{false};
  std::atomic<std::uint64_t> overflow_count_{0};
  std::thread worker_;
};

}  // namespace bluerov_integration::team_byung
