#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "torpedo_control_v2/control_logic.hpp"
#include "torpedo_control_v2/csv_logger.hpp"
#include "torpedo_control_v2/keyboard_input.hpp"
#include "torpedo_control_v2/state_receiver.hpp"

namespace torpedo_control_v2
{

class TorpedoControlNode final : public rclcpp::Node
{
public:
  TorpedoControlNode()
  : Node("torpedo_manual_control_v2"),
    session_start_steady_(std::chrono::steady_clock::now())
  {
    config_ = load_config();
    validate_config(config_);

    sensor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    control_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    create_publishers();

    StateReceiver::Config receiver_config;
    receiver_config.odometry_topic = config_.odometry_topic;
    receiver_config.target_odometry_topic = config_.target_odometry_topic;
    receiver_config.joint_states_topic = config_.joint_states_topic;
    receiver_config.propeller_front_joint_name = config_.propeller_front_joint_name;
    receiver_config.propeller_rear_joint_name = config_.propeller_rear_joint_name;
    receiver_config.fin_top_joint_name = config_.fin_top_joint_name;
    receiver_config.fin_bottom_joint_name = config_.fin_bottom_joint_name;
    receiver_config.fin_left_joint_name = config_.fin_left_joint_name;
    receiver_config.fin_right_joint_name = config_.fin_right_joint_name;
    receiver_config.raw_queue_capacity = config_.sensor_queue_capacity;
    receiver_config.callback_group = sensor_callback_group_;
    receiver_ = std::make_unique<StateReceiver>(*this, std::move(receiver_config));

    logic_ = std::make_unique<ControlLogic>(config_);

    CsvLogger::Config logger_config;
    logger_config.enabled = config_.log_enabled;
    logger_config.directory = config_.log_directory;
    logger_config.flush_every_n_rows = config_.log_flush_every_n_rows;
    logger_config.queue_capacity = config_.logger_queue_capacity;
    logger_ = std::make_unique<CsvLogger>(std::move(logger_config));
    logger_->start();

    keyboard_ = std::make_unique<KeyboardInput>(config_.key_queue_capacity);
    keyboard_->start();

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / config_.publish_rate_hz));
    timer_ = create_wall_timer(
      timer_period,
      std::bind(&TorpedoControlNode::on_control_timer, this),
      control_callback_group_);

    const StateSnapshot initial_state = receiver_->snapshot();
    const ControlResult initial_result = logic_->initial_result();
    publish_commands(initial_result.command);
    enqueue_telemetry(initial_result, initial_state);

    RCLCPP_INFO(
      get_logger(),
      "Keyboard ready: W/S pitch, A/D yaw, R/F thrust, Space stop, T auto-track, Ctrl+C exit");
    if (config_.log_enabled) {
      RCLCPP_INFO(get_logger(), "Control CSV: %s", logger_->control_log_path().c_str());
      RCLCPP_INFO(get_logger(), "Sensor CSV: %s", logger_->sensor_log_path().c_str());
    }
  }

  ~TorpedoControlNode() override
  {
    timer_.reset();
    if (keyboard_) {
      keyboard_->stop();
    }

    if (logic_ && receiver_ && logger_) {
      const StateSnapshot state = receiver_->snapshot();
      const ControlResult stop_result = logic_->emergency_stop();
      if (rclcpp::ok()) {
        try {
          publish_commands(stop_result.command);
        } catch (...) {
          // ROS may already be shutting down. Logging the stop frame remains safe.
        }
      }
      for (auto & sample : receiver_->drain_raw_samples()) {
        (void)logger_->enqueue(std::move(sample));
      }
      enqueue_telemetry(stop_result, state);
      logger_->stop();
    }
  }

private:
  static char single_key(const std::string & value, const std::string & name)
  {
    if (value.size() != 1U) {
      throw std::runtime_error(name + " must contain exactly one character");
    }
    return value.front() >= 'A' && value.front() <= 'Z' ?
      static_cast<char>(value.front() - 'A' + 'a') : value.front();
  }

  ControllerConfig load_config()
  {
    ControllerConfig config;
    config.thrust_topic = declare_parameter<std::string>(
      "thrust_topic", "/torpedo/actuators/thruster/command");
    config.fin_top_topic = declare_parameter<std::string>(
      "fin_top_topic", "/torpedo/actuators/fins/top/command");
    config.fin_bottom_topic = declare_parameter<std::string>(
      "fin_bottom_topic", "/torpedo/actuators/fins/bottom/command");
    config.fin_left_topic = declare_parameter<std::string>(
      "fin_left_topic", "/torpedo/actuators/fins/left/command");
    config.fin_right_topic = declare_parameter<std::string>(
      "fin_right_topic", "/torpedo/actuators/fins/right/command");
    config.odometry_topic = declare_parameter<std::string>(
      "odometry_topic", "/torpedo/state/odometry");
    config.target_odometry_topic = declare_parameter<std::string>(
      "target_odometry_topic", "/model/bluerov2/odometry");
    config.joint_states_topic = declare_parameter<std::string>(
      "joint_states_topic", "/torpedo/state/joint_states");

    config.propeller_front_joint_name = declare_parameter<std::string>(
      "propeller_front_joint_name", "propeller_joint");
    config.propeller_rear_joint_name = declare_parameter<std::string>(
      "propeller_rear_joint_name", "propeller_rear_joint");
    config.fin_top_joint_name = declare_parameter<std::string>(
      "fin_top_joint_name", "fin_top_joint");
    config.fin_bottom_joint_name = declare_parameter<std::string>(
      "fin_bottom_joint_name", "fin_bottom_joint");
    config.fin_left_joint_name = declare_parameter<std::string>(
      "fin_left_joint_name", "fin_left_joint");
    config.fin_right_joint_name = declare_parameter<std::string>(
      "fin_right_joint_name", "fin_right_joint");

    config.nose_push_key = single_key(
      declare_parameter<std::string>("nose_push_key", "w"), "nose_push_key");
    config.nose_pull_key = single_key(
      declare_parameter<std::string>("nose_pull_key", "s"), "nose_pull_key");
    config.yaw_left_key = single_key(
      declare_parameter<std::string>("yaw_left_key", "a"), "yaw_left_key");
    config.yaw_right_key = single_key(
      declare_parameter<std::string>("yaw_right_key", "d"), "yaw_right_key");
    config.thrust_increase_key = single_key(
      declare_parameter<std::string>("thrust_increase_key", "r"), "thrust_increase_key");
    config.thrust_decrease_key = single_key(
      declare_parameter<std::string>("thrust_decrease_key", "f"), "thrust_decrease_key");
    config.thrust_stop_key = single_key(
      declare_parameter<std::string>("thrust_stop_key", " "), "thrust_stop_key");
    config.auto_track_key = single_key(
      declare_parameter<std::string>("auto_track_key", "t"), "auto_track_key");

    config.thrust_initial = declare_parameter<double>("thrust_initial", 0.0);
    config.thrust_step = declare_parameter<double>("thrust_step", 1500.0);
    config.thrust_min = declare_parameter<double>("thrust_min", 0.0);
    config.thrust_max = declare_parameter<double>("thrust_max", 1500.0);
    config.fin_command_rad = declare_parameter<double>("fin_command_rad", 0.05);
    config.fin_limit_rad = declare_parameter<double>("fin_limit_rad", 0.50);
    config.fin_min_rad = declare_parameter<double>("fin_min_rad", -0.50);
    config.fin_max_rad = declare_parameter<double>("fin_max_rad", 0.50);
    config.publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    config.p_gain_pitch = declare_parameter<double>("p_gain_pitch", 0.8);
    config.p_gain_yaw = declare_parameter<double>("p_gain_yaw", 0.8);
    config.roll_control_enabled = declare_parameter<bool>("roll_control_enabled", true);
    config.roll_kp = declare_parameter<double>("roll_kp", 0.6);
    config.roll_kd = declare_parameter<double>("roll_kd", 0.15);
    config.roll_limit_rad = declare_parameter<double>("roll_limit_rad", 0.10);
    config.roll_input_sign = declare_parameter<double>("roll_input_sign", -1.0);
    config.roll_top_sign = declare_parameter<double>("roll_top_sign", 1.0);
    config.roll_bottom_sign = declare_parameter<double>("roll_bottom_sign", -1.0);

    config.thrust_output_sign = declare_parameter<double>("thrust_output_sign", 1.0);
    config.pitch_input_sign = declare_parameter<double>("pitch_input_sign", 1.0);
    config.yaw_input_sign = declare_parameter<double>("yaw_input_sign", 1.0);
    config.pitch_left_sign = declare_parameter<double>("pitch_left_sign", 1.0);
    config.pitch_right_sign = declare_parameter<double>("pitch_right_sign", 1.0);
    config.yaw_top_sign = declare_parameter<double>("yaw_top_sign", 1.0);
    config.yaw_bottom_sign = declare_parameter<double>("yaw_bottom_sign", 1.0);

    config.fin_top_neutral_rad = declare_parameter<double>("fin_top_neutral_rad", 0.0);
    config.fin_bottom_neutral_rad = declare_parameter<double>("fin_bottom_neutral_rad", 0.0);
    config.fin_left_neutral_rad = declare_parameter<double>("fin_left_neutral_rad", 0.0);
    config.fin_right_neutral_rad = declare_parameter<double>("fin_right_neutral_rad", 0.0);

    config.log_enabled = declare_parameter<bool>("log_enabled", true);
    config.log_directory = declare_parameter<std::string>(
      "log_directory", "~/.ros/torpedo_control_v2_logs");
    config.log_flush_every_n_rows = declare_parameter<int>("log_flush_every_n_rows", 1);
    const int sensor_capacity = declare_parameter<int>("sensor_queue_capacity", 8192);
    const int key_capacity = declare_parameter<int>("key_queue_capacity", 1024);
    const int logger_capacity = declare_parameter<int>("logger_queue_capacity", 100000);
    if (sensor_capacity < 0 || key_capacity < 0 || logger_capacity < 0) {
      throw std::runtime_error("Queue capacities must not be negative");
    }
    config.sensor_queue_capacity = static_cast<std::size_t>(sensor_capacity);
    config.key_queue_capacity = static_cast<std::size_t>(key_capacity);
    config.logger_queue_capacity = static_cast<std::size_t>(logger_capacity);
    return config;
  }

  static void validate_config(const ControllerConfig & config)
  {
    const double values[] = {
      config.thrust_initial, config.thrust_step, config.thrust_min, config.thrust_max,
      config.fin_command_rad, config.fin_limit_rad, config.fin_min_rad, config.fin_max_rad,
      config.publish_rate_hz,
      config.p_gain_pitch, config.p_gain_yaw, config.roll_kp, config.roll_kd,
      config.roll_limit_rad, config.roll_input_sign, config.roll_top_sign,
      config.roll_bottom_sign, config.thrust_output_sign,
      config.pitch_input_sign, config.yaw_input_sign, config.pitch_left_sign,
      config.pitch_right_sign, config.yaw_top_sign, config.yaw_bottom_sign,
      config.fin_top_neutral_rad, config.fin_bottom_neutral_rad,
      config.fin_left_neutral_rad, config.fin_right_neutral_rad};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("All numeric controller parameters must be finite");
      }
    }
    if (config.thrust_min > config.thrust_max) {
      throw std::runtime_error("thrust_min must be <= thrust_max");
    }
    if (config.thrust_step <= 0.0) {
      throw std::runtime_error("thrust_step must be greater than zero");
    }
    if (config.fin_command_rad < 0.0 || config.fin_limit_rad <= 0.0 ||
      config.fin_command_rad > config.fin_limit_rad)
    {
      throw std::runtime_error(
              "fin_command_rad must be non-negative and no greater than fin_limit_rad");
    }
    if (config.fin_min_rad >= config.fin_max_rad) {
      throw std::runtime_error("fin_min_rad must be less than fin_max_rad");
    }
    if (config.publish_rate_hz <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than zero");
    }
    if (config.roll_kp < 0.0 || config.roll_kd < 0.0 || config.roll_limit_rad <= 0.0) {
      throw std::runtime_error("Roll gains must be non-negative and roll_limit_rad must be positive");
    }
    if (config.log_flush_every_n_rows <= 0) {
      throw std::runtime_error("log_flush_every_n_rows must be greater than zero");
    }
  }

  void create_publishers()
  {
    thrust_pub_ = create_publisher<std_msgs::msg::Float64>(config_.thrust_topic, 10);
    fin_top_pub_ = create_publisher<std_msgs::msg::Float64>(config_.fin_top_topic, 10);
    fin_bottom_pub_ = create_publisher<std_msgs::msg::Float64>(config_.fin_bottom_topic, 10);
    fin_left_pub_ = create_publisher<std_msgs::msg::Float64>(config_.fin_left_topic, 10);
    fin_right_pub_ = create_publisher<std_msgs::msg::Float64>(config_.fin_right_topic, 10);
  }

  void on_control_timer()
  {
    // The receiver lock is held only inside snapshot(). The expensive work
    // below is performed on local copies and never blocks sensor callbacks.
    const StateSnapshot state = receiver_->snapshot();
    const std::vector<KeyEvent> events = keyboard_->drain();
    const ControlResult result = logic_->step(state, events);

    publish_commands(result.command);

    // Keyboard events are printed as soon as the control cycle consumes them.
    // The periodic status line below is throttled so console I/O cannot flood
    // the 20 Hz control loop.
    if (result.events != "NONE") {
      RCLCPP_INFO(
        get_logger(),
        "[KEY] %s | thrust=%.1f | pitch_target=%+.3f rad | yaw_target=%+.3f rad",
        result.events.c_str(), result.thrust_level,
        result.command.pitch_target_rad, result.command.yaw_target_rad);
    }
    print_status_if_due(result, state);

    // Move raw samples out of the receiver before enqueueing them. The logger
    // queue has its own mutex and disk I/O happens on its worker thread.
    for (auto & sample : receiver_->drain_raw_samples()) {
      (void)logger_->enqueue(std::move(sample));
    }
    enqueue_telemetry(result, state);
  }

  void print_status_if_due(const ControlResult & result, const StateSnapshot & state)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady - last_status_print_steady_ < std::chrono::milliseconds(500)) {
      return;
    }
    last_status_print_steady_ = now_steady;

    const auto to_degrees = [](const double radians) {
        return std::isfinite(radians) ? radians * 180.0 / 3.14159265358979323846 : radians;
      };
    const char * state_text = state.odometry.valid ? "ODOM_OK" : "ODOM_WAIT";
    const char * target_text = state.target.valid ? "TARGET_OK" : "TARGET_WAIT";
    RCLCPP_INFO(
      get_logger(),
      "[STATUS] %s/%s | mode=%s | pitch_err=%+.3f rad (%+.1f deg) "
      "yaw_err=%+.3f rad (%+.1f deg) | fin[T:%+.3f B:%+.3f L:%+.3f R:%+.3f] | "
      "roll=%+.3f rad (%+.1f deg) rate=%+.3f req=%+.3f applied=%+.3f scale=%.2f | "
      "queue_overflow[key:%zu sensor:%zu log:%zu]",
      state_text, target_text, result.auto_track ? "AUTO" : "MANUAL",
      result.pitch_error_rad, to_degrees(result.pitch_error_rad),
      result.yaw_error_rad, to_degrees(result.yaw_error_rad),
      result.command.fin_top, result.command.fin_bottom,
      result.command.fin_left, result.command.fin_right,
      result.roll_angle_rad, to_degrees(result.roll_angle_rad),
      result.roll_rate_rad_per_sec, result.roll_correction_rad,
      result.command.roll_correction_applied_rad, result.command.roll_mix_scale,
      keyboard_->overflow_count(), receiver_->raw_overflow_count(), logger_->overflow_count());
  }

  void publish_commands(const ActuatorCommand & command)
  {
    std_msgs::msg::Float64 message;
    message.data = command.thrust;
    thrust_pub_->publish(message);
    message.data = command.fin_top;
    fin_top_pub_->publish(message);
    message.data = command.fin_bottom;
    fin_bottom_pub_->publish(message);
    message.data = command.fin_left;
    fin_left_pub_->publish(message);
    message.data = command.fin_right;
    fin_right_pub_->publish(message);
  }

  void enqueue_telemetry(const ControlResult & result, const StateSnapshot & state)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto system_now = std::chrono::system_clock::now();
    TelemetryFrame frame;
    frame.wall_time_unix_sec = std::chrono::duration<double>(
      system_now.time_since_epoch()).count();
    frame.elapsed_sec = std::chrono::duration<double>(
      steady_now - session_start_steady_).count();
    frame.ros_time_sec = now().seconds();
    frame.key_events = result.events.empty() ? "NONE" : result.events;
    frame.control = result;
    frame.state = state;
    frame.keyboard_overflow_count = keyboard_ ? keyboard_->overflow_count() : 0U;
    frame.sensor_overflow_count = receiver_ ? receiver_->raw_overflow_count() : 0U;
    frame.logger_overflow_count = logger_ ? logger_->overflow_count() : 0U;
    (void)logger_->enqueue(std::move(frame));
  }

  ControllerConfig config_;
  std::chrono::steady_clock::time_point session_start_steady_;
  std::chrono::steady_clock::time_point last_status_print_steady_{session_start_steady_};

  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_top_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_bottom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_right_pub_;

  std::unique_ptr<StateReceiver> receiver_;
  std::unique_ptr<KeyboardInput> keyboard_;
  std::unique_ptr<ControlLogic> logic_;
  std::unique_ptr<CsvLogger> logger_;
};

}  // namespace torpedo_control_v2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<torpedo_control_v2::TorpedoControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("torpedo_control_node_v2"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
