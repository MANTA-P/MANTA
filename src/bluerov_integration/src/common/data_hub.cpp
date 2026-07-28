#include "bluerov_integration/common/data_hub.hpp"

#include <functional>
#include <utility>

namespace bluerov_integration::common
{

DataHub::DataHub(rclcpp::Node & node, DataHubConfig config)
: node_(node),
  config_(std::move(config)),
  callback_group_(node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant))
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  const auto sensor_qos = rclcpp::SensorDataQoS();

  if (!config_.pressure_topic.empty()) {
    pressure_sub_ = node_.create_subscription<sensor_msgs::msg::FluidPressure>(
      config_.pressure_topic, sensor_qos,
      std::bind(&DataHub::pressureCallback, this, std::placeholders::_1), options);
  }
  if (!config_.depth_topic.empty()) {
    depth_sub_ = node_.create_subscription<geometry_msgs::msg::PointStamped>(
      config_.depth_topic, sensor_qos,
      std::bind(&DataHub::depthCallback, this, std::placeholders::_1), options);
  }
  if (!config_.imu_topic.empty()) {
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      config_.imu_topic, sensor_qos,
      std::bind(&DataHub::imuCallback, this, std::placeholders::_1), options);
  }
  if (!config_.bluerov_odometry_topic.empty()) {
    bluerov_odometry_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      config_.bluerov_odometry_topic, sensor_qos,
      std::bind(&DataHub::bluerovOdometryCallback, this, std::placeholders::_1), options);
  }
  if (!config_.dvl_topic.empty()) {
    dvl_sub_ = node_.create_subscription<dave_interfaces::msg::DVL>(
      config_.dvl_topic, sensor_qos,
      std::bind(&DataHub::dvlCallback, this, std::placeholders::_1), options);
  }
  if (!config_.torpedo_odometry_topic.empty()) {
    torpedo_odometry_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      config_.torpedo_odometry_topic, sensor_qos,
      std::bind(&DataHub::torpedoOdometryCallback, this, std::placeholders::_1), options);
  }
  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  if (!config_.mission_goal_topic.empty()) {
    mission_goal_sub_ = node_.create_subscription<geometry_msgs::msg::PointStamped>(
      config_.mission_goal_topic,
      latched_qos,
      std::bind(&DataHub::missionGoalCallback, this, std::placeholders::_1), options);
  }
  if (!config_.reference_path_topic.empty()) {
    reference_path_sub_ = node_.create_subscription<nav_msgs::msg::Path>(
      config_.reference_path_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&DataHub::referencePathCallback, this, std::placeholders::_1), options);
  }

  RCLCPP_INFO(node_.get_logger(), "BlueROV DataHub subscriptions initialized");
}

StateSnapshot DataHub::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void DataHub::pressureCallback(
  const sensor_msgs::msg::FluidPressure::ConstSharedPtr message)
{
  store(state_.pressure, *message);
}

void DataHub::depthCallback(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  store(state_.depth, *message);
}

void DataHub::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  store(state_.imu, *message);
}

void DataHub::bluerovOdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  store(state_.bluerov_odometry, *message);
}

void DataHub::dvlCallback(const dave_interfaces::msg::DVL::ConstSharedPtr message)
{
  store(state_.dvl, *message);
}

void DataHub::torpedoOdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  store(state_.torpedo_odometry, *message);
}

void DataHub::missionGoalCallback(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  store(state_.mission_goal, *message);
}

void DataHub::referencePathCallback(
  const nav_msgs::msg::Path::ConstSharedPtr message)
{
  store(state_.reference_path, message);
}

}  // namespace bluerov_integration::common
