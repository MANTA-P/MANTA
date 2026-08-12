#pragma once

#include <memory>
#include <mutex>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "torpedo_control_v2/types.hpp"
#include "torpedo_control_v2/util/tick_tock.hpp"

class RosInterface final : public rclcpp::Node
{
public:
    static std::shared_ptr<RosInterface> create(int argc, char * argv[]);
    ~RosInterface();

    void start();
    void stop();
    bool ok() const;

    SensorData latest_sensor_data() const;
    void publish_command(const ActuatorCommand & command);

private:
    RosInterface();
    void spin();

    void on_torpedo_odometry(const nav_msgs::msg::Odometry & message);
    void on_target_odometry(const nav_msgs::msg::Odometry & message);
    mutable std::mutex mutex_;
    SensorData latest_data_;
    torpedo_control_v2::util::TickTock torpedo_odometry_timer_;
    torpedo_control_v2::util::TickTock target_odometry_timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr torpedo_odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odometry_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_top_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_bottom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fin_right_pub_;

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread ros_thread_;
};
