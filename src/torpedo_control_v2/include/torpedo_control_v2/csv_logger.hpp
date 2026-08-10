#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "torpedo_control_v2/control_types.hpp"

namespace torpedo_control_v2
{

class CsvLogger
{
public:
  struct Config
  {
    bool enabled{true};
    std::string directory{"~/.ros/torpedo_control_v2_logs"};
    int flush_every_n_rows{1};
    std::size_t queue_capacity{100000U};
  };

  explicit CsvLogger(Config config);
  ~CsvLogger();

  CsvLogger(const CsvLogger &) = delete;
  CsvLogger & operator=(const CsvLogger &) = delete;

  void start();
  void stop();

  // The lock protects only the queue insertion. Disk I/O is performed by the
  // logger thread after it has released the lock.
  bool enqueue(LogRecord record);

  std::size_t overflow_count() const;
  const std::string & control_log_path() const;
  const std::string & sensor_log_path() const;

private:
  static std::string expand_user_path(const std::string & path);
  static std::string sensor_kind_name(SensorKind kind);
  static double wall_time_now_sec();

  void open_files();
  void worker_loop();
  void write_record(const TelemetryFrame & frame);
  void write_record(const RawSensorFrame & frame);
  void write_control_header();
  void write_sensor_header();

  Config config_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<LogRecord> queue_;
  std::size_t overflow_count_{0U};
  bool started_{false};
  bool stop_requested_{false};
  std::thread worker_;

  std::ofstream control_file_;
  std::ofstream sensor_file_;
  std::string control_log_path_;
  std::string sensor_log_path_;
  std::size_t control_row_count_{0U};
  std::size_t sensor_row_count_{0U};
};

}  // namespace torpedo_control_v2
