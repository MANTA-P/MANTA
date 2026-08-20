#include <memory>

#include "torpedo_control_v2/ros_interface.hpp"

namespace
{
    OdometryData convert_odometry(const nav_msgs::msg::Odometry & message)
    {
        OdometryData data;
        data.valid = true;

        data.position_x = message.pose.pose.position.x;
        data.position_y = message.pose.pose.position.y;
        data.position_z = message.pose.pose.position.z;

        data.orientation_x = message.pose.pose.orientation.x;
        data.orientation_y = message.pose.pose.orientation.y;
        data.orientation_z = message.pose.pose.orientation.z;
        data.orientation_w = message.pose.pose.orientation.w;

        data.linear_x = message.twist.twist.linear.x;
        data.linear_y = message.twist.twist.linear.y;
        data.linear_z = message.twist.twist.linear.z;

        data.angular_x = message.twist.twist.angular.x;
        data.angular_y = message.twist.twist.angular.y;
        data.angular_z = message.twist.twist.angular.z;

        return data;
    }
}  // namespace

std::shared_ptr<RosInterface> RosInterface::create(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    return std::shared_ptr<RosInterface>(new RosInterface());
}

RosInterface::RosInterface() : Node("torpedo_control_node_v2")
{
    torpedo_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    torpedo_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/torpedo/state/odometry",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
            if (message) {
                on_torpedo_odometry(*message);
            }
        });

    target_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/model/bluerov2/odometry",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
            if (message) {
                on_target_odometry(*message);
            }
        });

    thrust_pub_ = create_publisher<std_msgs::msg::Float64>("/torpedo/actuators/thruster/command", 10);
    fin_top_pub_ = create_publisher<std_msgs::msg::Float64>("/torpedo/actuators/fins/top/command", 10);
    fin_bottom_pub_ = create_publisher<std_msgs::msg::Float64>("/torpedo/actuators/fins/bottom/command", 10);
    fin_left_pub_ = create_publisher<std_msgs::msg::Float64>("/torpedo/actuators/fins/left/command", 10);
    fin_right_pub_ = create_publisher<std_msgs::msg::Float64>("/torpedo/actuators/fins/right/command", 10);

    executor_.add_node(get_node_base_interface());
}

RosInterface::~RosInterface()
{
    stop();

    executor_.remove_node(get_node_base_interface());
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void RosInterface::start()
{
    if (!ros_thread_.joinable()) {
        ros_thread_ = std::thread(&RosInterface::spin, this);
    }
}

void RosInterface::stop()
{
    executor_.cancel();

    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
}

void RosInterface::spin()
{
    executor_.spin();
}

bool RosInterface::ok() const
{
    return rclcpp::ok();
}

SensorData RosInterface::latest_sensor_data() const
{
    SensorData snapshot;
    long long torpedo_odometry_age_us;
    long long target_odometry_age_us;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = latest_data_;
        torpedo_odometry_age_us = torpedo_odometry_timer_.tock();
        target_odometry_age_us = target_odometry_timer_.tock();
    }

    // Timeout values are in microseconds.
    snapshot.torpedo_odometry.valid &= torpedo_odometry_age_us <= 30'000;
    snapshot.target_odometry.valid &= target_odometry_age_us <= 30'000;

    return snapshot;
}

void RosInterface::publish_command(const ActuatorCommand & command)
{
    std_msgs::msg::Float64 message;

    message.data = command.thrust; thrust_pub_->publish(message);
    message.data = command.fin_top; fin_top_pub_->publish(message);
    message.data = command.fin_bottom; fin_bottom_pub_->publish(message);
    message.data = command.fin_left; fin_left_pub_->publish(message);
    message.data = command.fin_right; fin_right_pub_->publish(message);
}

void RosInterface::on_torpedo_odometry(const nav_msgs::msg::Odometry & message)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_data_.torpedo_odometry = convert_odometry(message);
        torpedo_odometry_timer_.tick();
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message.header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "torpedo_base_link";
    transform.transform.translation.x = message.pose.pose.position.x;
    transform.transform.translation.y = message.pose.pose.position.y;
    transform.transform.translation.z = message.pose.pose.position.z;
    transform.transform.rotation = message.pose.pose.orientation;
    torpedo_tf_broadcaster_->sendTransform(transform);
}

void RosInterface::on_target_odometry(const nav_msgs::msg::Odometry & message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_data_.target_odometry = convert_odometry(message);
    target_odometry_timer_.tick();
}
