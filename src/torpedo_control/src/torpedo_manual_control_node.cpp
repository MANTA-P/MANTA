#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

class TorpedoManualControl : public rclcpp::Node
{
public:
  TorpedoManualControl()
  : Node("torpedo_manual_control"),
    session_start_steady_(std::chrono::steady_clock::now())
  {
    load_parameters();
    validate_parameters();
    create_publishers();
    create_state_subscribers();
    configure_logging();
    configure_terminal();

    thrust_level_ = std::clamp(thrust_initial_, thrust_min_, thrust_max_);

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&TorpedoManualControl::update, this));

    append_event("START");
    const auto commands = make_command_snapshot();
    publish_commands(commands);
    write_log_row(commands);

    RCLCPP_INFO(
      get_logger(),
      "Keyboard ready: W/S pitch, A/D yaw, R/F thrust, Space stop, Ctrl+C exit");
    if (log_enabled_) {
      RCLCPP_INFO(get_logger(), "CSV log: %s", log_path_.c_str());
    }
  }

  ~TorpedoManualControl() override
  {
    publish_neutral_best_effort();
    restore_terminal();
    if (log_file_.is_open()) {
      log_file_.flush();
      log_file_.close();
    }
  }

private:
  struct CommandSnapshot
  {
    double thrust{0.0};
    double pitch_target_rad{0.0};
    double yaw_target_rad{0.0};
    double fin_top{0.0};
    double fin_bottom{0.0};
    double fin_left{0.0};
    double fin_right{0.0};
  };

  struct OdometrySnapshot
  {
    bool valid{false};
    double stamp{std::numeric_limits<double>::quiet_NaN()};
    double position_x{std::numeric_limits<double>::quiet_NaN()};
    double position_y{std::numeric_limits<double>::quiet_NaN()};
    double position_z{std::numeric_limits<double>::quiet_NaN()};
    double orientation_x{std::numeric_limits<double>::quiet_NaN()};
    double orientation_y{std::numeric_limits<double>::quiet_NaN()};
    double orientation_z{std::numeric_limits<double>::quiet_NaN()};
    double orientation_w{std::numeric_limits<double>::quiet_NaN()};
    double linear_x{std::numeric_limits<double>::quiet_NaN()};
    double linear_y{std::numeric_limits<double>::quiet_NaN()};
    double linear_z{std::numeric_limits<double>::quiet_NaN()};
    double angular_x{std::numeric_limits<double>::quiet_NaN()};
    double angular_y{std::numeric_limits<double>::quiet_NaN()};
    double angular_z{std::numeric_limits<double>::quiet_NaN()};
  };

  struct JointStateSnapshot
  {
    bool valid{false};
    double stamp{std::numeric_limits<double>::quiet_NaN()};
    double propeller_front_position{std::numeric_limits<double>::quiet_NaN()};
    double propeller_front_velocity{std::numeric_limits<double>::quiet_NaN()};
    double propeller_rear_position{std::numeric_limits<double>::quiet_NaN()};
    double propeller_rear_velocity{std::numeric_limits<double>::quiet_NaN()};
    double fin_top{std::numeric_limits<double>::quiet_NaN()};
    double fin_bottom{std::numeric_limits<double>::quiet_NaN()};
    double fin_left{std::numeric_limits<double>::quiet_NaN()};
    double fin_right{std::numeric_limits<double>::quiet_NaN()};
  };

  static char single_key(const std::string & value, const std::string & parameter_name)
  {
    if (value.size() != 1U) {
      throw std::runtime_error(parameter_name + " must contain exactly one character");
    }

    return static_cast<char>(
      std::tolower(static_cast<unsigned char>(value.front())));
  }

  static double message_stamp(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
  }

  static std::string expand_user_path(const std::string & path)
  {
    if (path == "~" || path.rfind("~/", 0) == 0) {
      const char * home = std::getenv("HOME");
      if (home == nullptr) {
        throw std::runtime_error("HOME is not set; cannot expand log_directory");
      }
      if (path == "~") {
        return std::string(home);
      }
      return std::string(home) + path.substr(1);
    }
    return path;
  }

  void load_parameters()
  {
    thrust_topic_ = declare_parameter<std::string>(
      "thrust_topic", "/torpedo/actuators/thruster/command");
    fin_top_topic_ = declare_parameter<std::string>(
      "fin_top_topic", "/torpedo/actuators/fins/top/command");
    fin_bottom_topic_ = declare_parameter<std::string>(
      "fin_bottom_topic", "/torpedo/actuators/fins/bottom/command");
    fin_left_topic_ = declare_parameter<std::string>(
      "fin_left_topic", "/torpedo/actuators/fins/left/command");
    fin_right_topic_ = declare_parameter<std::string>(
      "fin_right_topic", "/torpedo/actuators/fins/right/command");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/torpedo/state/odometry");
    joint_states_topic_ = declare_parameter<std::string>(
      "joint_states_topic", "/torpedo/state/joint_states");

    propeller_front_joint_name_ = declare_parameter<std::string>(
      "propeller_front_joint_name", "propeller_joint");
    propeller_rear_joint_name_ = declare_parameter<std::string>(
      "propeller_rear_joint_name", "propeller_rear_joint");
    fin_top_joint_name_ = declare_parameter<std::string>(
      "fin_top_joint_name", "fin_top_joint");
    fin_bottom_joint_name_ = declare_parameter<std::string>(
      "fin_bottom_joint_name", "fin_bottom_joint");
    fin_left_joint_name_ = declare_parameter<std::string>(
      "fin_left_joint_name", "fin_left_joint");
    fin_right_joint_name_ = declare_parameter<std::string>(
      "fin_right_joint_name", "fin_right_joint");

    nose_push_key_ = single_key(
      declare_parameter<std::string>("nose_push_key", "w"), "nose_push_key");
    nose_pull_key_ = single_key(
      declare_parameter<std::string>("nose_pull_key", "s"), "nose_pull_key");
    yaw_left_key_ = single_key(
      declare_parameter<std::string>("yaw_left_key", "a"), "yaw_left_key");
    yaw_right_key_ = single_key(
      declare_parameter<std::string>("yaw_right_key", "d"), "yaw_right_key");
    thrust_increase_key_ = single_key(
      declare_parameter<std::string>("thrust_increase_key", "r"),
      "thrust_increase_key");
    thrust_decrease_key_ = single_key(
      declare_parameter<std::string>("thrust_decrease_key", "f"),
      "thrust_decrease_key");
    thrust_stop_key_ = single_key(
      declare_parameter<std::string>("thrust_stop_key", " "), "thrust_stop_key");

    thrust_initial_ = declare_parameter<double>("thrust_initial", 0.0);
    thrust_step_ = declare_parameter<double>("thrust_step", 1500.0);
    thrust_min_ = declare_parameter<double>("thrust_min", 0.0);
    thrust_max_ = declare_parameter<double>("thrust_max", 1500.0);
    fin_command_rad_ = declare_parameter<double>("fin_command_rad", 0.05);
    fin_limit_rad_ = declare_parameter<double>("fin_limit_rad", 0.50);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    thrust_output_sign_ = declare_parameter<double>("thrust_output_sign", 1.0);
    pitch_input_sign_ = declare_parameter<double>("pitch_input_sign", 1.0);
    yaw_input_sign_ = declare_parameter<double>("yaw_input_sign", 1.0);
    pitch_left_sign_ = declare_parameter<double>("pitch_left_sign", 1.0);
    pitch_right_sign_ = declare_parameter<double>("pitch_right_sign", 1.0);
    yaw_top_sign_ = declare_parameter<double>("yaw_top_sign", 1.0);
    yaw_bottom_sign_ = declare_parameter<double>("yaw_bottom_sign", 1.0);

    fin_top_neutral_rad_ = declare_parameter<double>("fin_top_neutral_rad", 0.0);
    fin_bottom_neutral_rad_ = declare_parameter<double>("fin_bottom_neutral_rad", 0.0);
    fin_left_neutral_rad_ = declare_parameter<double>("fin_left_neutral_rad", 0.0);
    fin_right_neutral_rad_ = declare_parameter<double>("fin_right_neutral_rad", 0.0);

    log_enabled_ = declare_parameter<bool>("log_enabled", true);
    log_directory_ = declare_parameter<std::string>(
      "log_directory", "~/.ros/torpedo_control_logs");
    log_flush_every_n_rows_ = declare_parameter<int>("log_flush_every_n_rows", 1);
  }

  void validate_parameters() const
  {
    const double values[] = {
      thrust_initial_, thrust_step_, thrust_min_, thrust_max_, fin_command_rad_,
      fin_limit_rad_, publish_rate_hz_, thrust_output_sign_,
      pitch_input_sign_, yaw_input_sign_, pitch_left_sign_, pitch_right_sign_,
      yaw_top_sign_, yaw_bottom_sign_, fin_top_neutral_rad_, fin_bottom_neutral_rad_,
      fin_left_neutral_rad_, fin_right_neutral_rad_};

    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("All numeric controller parameters must be finite");
      }
    }

    if (thrust_min_ > thrust_max_) {
      throw std::runtime_error("thrust_min must be less than or equal to thrust_max");
    }
    if (thrust_step_ <= 0.0) {
      throw std::runtime_error("thrust_step must be greater than zero");
    }
    if (fin_command_rad_ < 0.0 || fin_limit_rad_ <= 0.0 ||
      fin_command_rad_ > fin_limit_rad_)
    {
      throw std::runtime_error(
              "fin_command_rad must be non-negative and no greater than fin_limit_rad");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than zero");
    }
    if (log_flush_every_n_rows_ <= 0) {
      throw std::runtime_error("log_flush_every_n_rows must be greater than zero");
    }
  }

  void create_publishers()
  {
    thrust_pub_ = create_publisher<std_msgs::msg::Float64>(thrust_topic_, 10);
    fin_top_pub_ = create_publisher<std_msgs::msg::Float64>(fin_top_topic_, 10);
    fin_bottom_pub_ = create_publisher<std_msgs::msg::Float64>(fin_bottom_topic_, 10);
    fin_left_pub_ = create_publisher<std_msgs::msg::Float64>(fin_left_topic_, 10);
    fin_right_pub_ = create_publisher<std_msgs::msg::Float64>(fin_right_topic_, 10);
  }

  void create_state_subscribers()
  {
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        odometry_.valid = true;
        odometry_.stamp = message_stamp(message->header.stamp);
        odometry_.position_x = message->pose.pose.position.x;
        odometry_.position_y = message->pose.pose.position.y;
        odometry_.position_z = message->pose.pose.position.z;
        odometry_.orientation_x = message->pose.pose.orientation.x;
        odometry_.orientation_y = message->pose.pose.orientation.y;
        odometry_.orientation_z = message->pose.pose.orientation.z;
        odometry_.orientation_w = message->pose.pose.orientation.w;
        odometry_.linear_x = message->twist.twist.linear.x;
        odometry_.linear_y = message->twist.twist.linear.y;
        odometry_.linear_z = message->twist.twist.linear.z;
        odometry_.angular_x = message->twist.twist.angular.x;
        odometry_.angular_y = message->twist.twist.angular.y;
        odometry_.angular_z = message->twist.twist.angular.z;
      });

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        joint_states_.valid = false;
        joint_states_.stamp = message_stamp(message->header.stamp);
        joint_states_.propeller_front_position = std::numeric_limits<double>::quiet_NaN();
        joint_states_.propeller_front_velocity = std::numeric_limits<double>::quiet_NaN();
        joint_states_.propeller_rear_position = std::numeric_limits<double>::quiet_NaN();
        joint_states_.propeller_rear_velocity = std::numeric_limits<double>::quiet_NaN();
        joint_states_.fin_top = std::numeric_limits<double>::quiet_NaN();
        joint_states_.fin_bottom = std::numeric_limits<double>::quiet_NaN();
        joint_states_.fin_left = std::numeric_limits<double>::quiet_NaN();
        joint_states_.fin_right = std::numeric_limits<double>::quiet_NaN();

        for (std::size_t index = 0; index < message->name.size(); ++index) {
          const auto & name = message->name[index];
          const double position = index < message->position.size() ?
            message->position[index] : std::numeric_limits<double>::quiet_NaN();
          const double velocity = index < message->velocity.size() ?
            message->velocity[index] : std::numeric_limits<double>::quiet_NaN();

          if (name == propeller_front_joint_name_) {
            joint_states_.propeller_front_position = position;
            joint_states_.propeller_front_velocity = velocity;
            joint_states_.valid = joint_states_.valid ||
              std::isfinite(position) || std::isfinite(velocity);
          } else if (name == propeller_rear_joint_name_) {
            joint_states_.propeller_rear_position = position;
            joint_states_.propeller_rear_velocity = velocity;
            joint_states_.valid = joint_states_.valid ||
              std::isfinite(position) || std::isfinite(velocity);
          } else if (name == fin_top_joint_name_) {
            joint_states_.fin_top = position;
            joint_states_.valid = joint_states_.valid || std::isfinite(position);
          } else if (name == fin_bottom_joint_name_) {
            joint_states_.fin_bottom = position;
            joint_states_.valid = joint_states_.valid || std::isfinite(position);
          } else if (name == fin_left_joint_name_) {
            joint_states_.fin_left = position;
            joint_states_.valid = joint_states_.valid || std::isfinite(position);
          } else if (name == fin_right_joint_name_) {
            joint_states_.fin_right = position;
            joint_states_.valid = joint_states_.valid || std::isfinite(position);
          }
        }
      });
  }

  void configure_logging()
  {
    if (!log_enabled_) {
      return;
    }

    const std::filesystem::path directory(expand_user_path(log_directory_));
    std::filesystem::create_directories(directory);

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream filename;
    filename << "torpedo_session_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
    log_path_ = (directory / filename.str()).string();

    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      throw std::runtime_error("Cannot open CSV log: " + log_path_);
    }

    log_file_ <<
      "wall_time_unix_sec,elapsed_sec,ros_time_sec,key_events,"
      "thrust_level,thrust_command,pitch_target_rad,yaw_target_rad,"
      "fin_top_command_rad,fin_bottom_command_rad,fin_left_command_rad,"
      "fin_right_command_rad,odom_valid,odom_stamp_sec,"
      "odom_position_x,odom_position_y,odom_position_z,"
      "odom_orientation_x,odom_orientation_y,odom_orientation_z,odom_orientation_w,"
      "odom_twist_linear_x,odom_twist_linear_y,odom_twist_linear_z,"
      "odom_twist_angular_x,odom_twist_angular_y,odom_twist_angular_z,"
      "joint_states_valid,joint_stamp_sec,joint_fin_top_position_rad,"
      "joint_fin_bottom_position_rad,joint_fin_left_position_rad,"
      "joint_fin_right_position_rad,joint_propeller_front_position_rad,"
      "joint_propeller_front_velocity_rad_per_sec,"
      "joint_propeller_rear_position_rad,"
      "joint_propeller_rear_velocity_rad_per_sec\n";
    log_file_ << std::fixed << std::setprecision(9);
    log_file_.flush();
  }

  void configure_terminal()
  {
    tty_fd_ = ::open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (tty_fd_ < 0) {
      throw std::runtime_error(
              std::string("Cannot open /dev/tty for keyboard input: ") + std::strerror(errno));
    }

    if (::tcgetattr(tty_fd_, &original_terminal_) != 0) {
      const std::string message =
        std::string("Cannot read terminal settings: ") + std::strerror(errno);
      ::close(tty_fd_);
      tty_fd_ = -1;
      throw std::runtime_error(message);
    }

    termios raw_terminal = original_terminal_;
    raw_terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw_terminal.c_cc[VMIN] = 0;
    raw_terminal.c_cc[VTIME] = 0;

    if (::tcsetattr(tty_fd_, TCSANOW, &raw_terminal) != 0) {
      const std::string message =
        std::string("Cannot configure terminal: ") + std::strerror(errno);
      ::close(tty_fd_);
      tty_fd_ = -1;
      throw std::runtime_error(message);
    }

    terminal_configured_ = true;
  }

  void restore_terminal()
  {
    if (terminal_configured_ && tty_fd_ >= 0) {
      (void)::tcsetattr(tty_fd_, TCSANOW, &original_terminal_);
    }
    if (tty_fd_ >= 0) {
      (void)::close(tty_fd_);
      tty_fd_ = -1;
    }
    terminal_configured_ = false;
  }

  void update()
  {
    read_keyboard();
    const auto commands = make_command_snapshot();
    publish_commands(commands);
    write_log_row(commands);
  }

  void append_event(const std::string & event)
  {
    if (pending_events_.size() > 128U) {
      return;
    }
    if (!pending_events_.empty()) {
      pending_events_ += "+";
    }
    pending_events_ += event;
  }

  void read_keyboard()
  {
    char buffer[32];

    while (true) {
      const ssize_t bytes_read = ::read(tty_fd_, buffer, sizeof(buffer));
      if (bytes_read > 0) {
        for (ssize_t i = 0; i < bytes_read; ++i) {
          handle_key(static_cast<char>(
              std::tolower(static_cast<unsigned char>(buffer[i]))));
        }
        continue;
      }

      if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Keyboard read failed: %s", std::strerror(errno));
      }
      break;
    }
  }

  void handle_key(const char key)
  {
    if (key == nose_push_key_) {
      pitch_target_rad_ = limited_fin_command(pitch_target_rad_ + fin_command_rad_);
      append_event("W");
      print_pitch_target();
    } else if (key == nose_pull_key_) {
      pitch_target_rad_ = limited_fin_command(pitch_target_rad_ - fin_command_rad_);
      append_event("S");
      print_pitch_target();
    } else if (key == yaw_left_key_) {
      yaw_target_rad_ = limited_fin_command(yaw_target_rad_ + fin_command_rad_);
      append_event("A");
      print_yaw_target();
    } else if (key == yaw_right_key_) {
      yaw_target_rad_ = limited_fin_command(yaw_target_rad_ - fin_command_rad_);
      append_event("D");
      print_yaw_target();
    } else if (key == thrust_increase_key_) {
      thrust_level_ = std::clamp(thrust_level_ + thrust_step_, thrust_min_, thrust_max_);
      append_event("R");
      RCLCPP_INFO(get_logger(), "Thrust level: %.3f", thrust_level_);
    } else if (key == thrust_decrease_key_) {
      thrust_level_ = std::clamp(thrust_level_ - thrust_step_, thrust_min_, thrust_max_);
      append_event("F");
      RCLCPP_INFO(get_logger(), "Thrust level: %.3f", thrust_level_);
    } else if (key == thrust_stop_key_) {
      thrust_level_ = 0.0;
      append_event("SPACE");
      RCLCPP_INFO(get_logger(), "Thrust stopped");
    }
  }

  double limited_fin_command(const double value) const
  {
    return std::clamp(value, -fin_limit_rad_, fin_limit_rad_);
  }

  void print_pitch_target() const
  {
    const auto commands = make_command_snapshot();
    RCLCPP_INFO(
      get_logger(), "Pitch target: %+.3f rad | left: %+.3f, right: %+.3f",
      pitch_target_rad_, commands.fin_left, commands.fin_right);
  }

  void print_yaw_target() const
  {
    const auto commands = make_command_snapshot();
    RCLCPP_INFO(
      get_logger(), "Yaw target: %+.3f rad | top: %+.3f, bottom: %+.3f",
      yaw_target_rad_, commands.fin_top, commands.fin_bottom);
  }

  CommandSnapshot make_command_snapshot() const
  {
    const double pitch_command = pitch_target_rad_ * pitch_input_sign_;
    const double yaw_command = yaw_target_rad_ * yaw_input_sign_;

    CommandSnapshot commands;
    commands.thrust = thrust_output_sign_ * thrust_level_;
    commands.pitch_target_rad = pitch_target_rad_;
    commands.yaw_target_rad = yaw_target_rad_;
    commands.fin_top = limited_fin_command(
      fin_top_neutral_rad_ + yaw_top_sign_ * yaw_command);
    commands.fin_bottom = limited_fin_command(
      fin_bottom_neutral_rad_ + yaw_bottom_sign_ * yaw_command);
    commands.fin_left = limited_fin_command(
      fin_left_neutral_rad_ + pitch_left_sign_ * pitch_command);
    commands.fin_right = limited_fin_command(
      fin_right_neutral_rad_ + pitch_right_sign_ * pitch_command);
    return commands;
  }

  void publish_commands(const CommandSnapshot & commands)
  {
    std_msgs::msg::Float64 message;

    message.data = commands.thrust;
    thrust_pub_->publish(message);
    message.data = commands.fin_top;
    fin_top_pub_->publish(message);
    message.data = commands.fin_bottom;
    fin_bottom_pub_->publish(message);
    message.data = commands.fin_left;
    fin_left_pub_->publish(message);
    message.data = commands.fin_right;
    fin_right_pub_->publish(message);
  }

  void write_log_row(const CommandSnapshot & commands)
  {
    if (!log_enabled_ || !log_file_.is_open()) {
      pending_events_.clear();
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    const auto system_now = std::chrono::system_clock::now();
    const double elapsed = std::chrono::duration<double>(
      steady_now - session_start_steady_).count();
    const double wall_time = std::chrono::duration<double>(
      system_now.time_since_epoch()).count();
    const std::string events = pending_events_.empty() ? "NONE" : pending_events_;

    log_file_ << wall_time << ',' << elapsed << ',' << now().seconds() << ',' << events << ',' <<
      thrust_level_ << ',' << commands.thrust << ',' << commands.pitch_target_rad << ',' <<
      commands.yaw_target_rad << ',' << commands.fin_top << ',' << commands.fin_bottom << ',' <<
      commands.fin_left << ',' << commands.fin_right << ',' << (odometry_.valid ? 1 : 0) << ',' <<
      odometry_.stamp << ',' << odometry_.position_x << ',' << odometry_.position_y << ',' <<
      odometry_.position_z << ',' << odometry_.orientation_x << ',' <<
      odometry_.orientation_y << ',' << odometry_.orientation_z << ',' <<
      odometry_.orientation_w << ',' << odometry_.linear_x << ',' << odometry_.linear_y << ',' <<
      odometry_.linear_z << ',' << odometry_.angular_x << ',' << odometry_.angular_y << ',' <<
      odometry_.angular_z << ',' << (joint_states_.valid ? 1 : 0) << ',' <<
      joint_states_.stamp << ',' << joint_states_.fin_top << ',' << joint_states_.fin_bottom <<
      ',' << joint_states_.fin_left << ',' << joint_states_.fin_right << ',' <<
      joint_states_.propeller_front_position << ',' << joint_states_.propeller_front_velocity <<
      ',' << joint_states_.propeller_rear_position << ',' <<
      joint_states_.propeller_rear_velocity << '\n';

    ++log_row_count_;
    if (log_row_count_ % static_cast<std::size_t>(log_flush_every_n_rows_) == 0U) {
      log_file_.flush();
    }
    pending_events_.clear();
  }

  void publish_neutral_best_effort()
  {
    thrust_level_ = 0.0;
    pitch_target_rad_ = 0.0;
    yaw_target_rad_ = 0.0;
    append_event("SHUTDOWN");
    const auto commands = make_command_snapshot();

    if (rclcpp::ok() && thrust_pub_) {
      try {
        publish_commands(commands);
      } catch (const std::exception &) {
        // The ROS context may already be shutting down.
      }
    }
    write_log_row(commands);
  }

  std::string thrust_topic_;
  std::string fin_top_topic_;
  std::string fin_bottom_topic_;
  std::string fin_left_topic_;
  std::string fin_right_topic_;
  std::string odometry_topic_;
  std::string joint_states_topic_;
  std::string propeller_front_joint_name_;
  std::string propeller_rear_joint_name_;
  std::string fin_top_joint_name_;
  std::string fin_bottom_joint_name_;
  std::string fin_left_joint_name_;
  std::string fin_right_joint_name_;

  char nose_push_key_{};
  char nose_pull_key_{};
  char yaw_left_key_{};
  char yaw_right_key_{};
  char thrust_increase_key_{};
  char thrust_decrease_key_{};
  char thrust_stop_key_{};

  double thrust_initial_{};
  double thrust_step_{};
  double thrust_min_{};
  double thrust_max_{};
  double fin_command_rad_{};
  double fin_limit_rad_{};
  double publish_rate_hz_{};
  double thrust_output_sign_{};
  double pitch_input_sign_{};
  double yaw_input_sign_{};
  double pitch_left_sign_{};
  double pitch_right_sign_{};
  double yaw_top_sign_{};
  double yaw_bottom_sign_{};
  double fin_top_neutral_rad_{};
  double fin_bottom_neutral_rad_{};
  double fin_left_neutral_rad_{};
  double fin_right_neutral_rad_{};

  bool log_enabled_{true};
  std::string log_directory_;
  int log_flush_every_n_rows_{1};
  std::string log_path_;
  std::ofstream log_file_;
  std::size_t log_row_count_{0};
  std::string pending_events_;

  double thrust_level_{0.0};
  double pitch_target_rad_{0.0};
  double yaw_target_rad_{0.0};
  OdometrySnapshot odometry_;
  JointStateSnapshot joint_states_;

  std::chrono::steady_clock::time_point session_start_steady_;

  int tty_fd_{-1};
  bool terminal_configured_{false};
  termios original_terminal_{};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_top_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_bottom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_right_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<TorpedoManualControl>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("torpedo_manual_control"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
