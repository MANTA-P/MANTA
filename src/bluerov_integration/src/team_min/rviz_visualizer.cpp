#include "bluerov_integration/team_min/rviz_visualizer.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace bluerov_integration::team_min
{
namespace
{

geometry_msgs::msg::Quaternion torpedoArrowOrientation(
  const geometry_msgs::msg::Quaternion & body_orientation)
{
  // RViz ARROW의 전방은 local +X지만 glider_slocum의 전방은 body +Y다.
  // body orientation 뒤에 local Z +90 deg 회전을 합성해 화살표를 +Y에 맞춘다.
  constexpr double half_angle = 0.7853981633974483;
  geometry_msgs::msg::Quaternion offset;
  offset.z = std::sin(half_angle);
  offset.w = std::cos(half_angle);
  geometry_msgs::msg::Quaternion result;
  result.x =
    body_orientation.w * offset.x + body_orientation.x * offset.w +
    body_orientation.y * offset.z - body_orientation.z * offset.y;
  result.y =
    body_orientation.w * offset.y - body_orientation.x * offset.z +
    body_orientation.y * offset.w + body_orientation.z * offset.x;
  result.z =
    body_orientation.w * offset.z + body_orientation.x * offset.y -
    body_orientation.y * offset.x + body_orientation.z * offset.w;
  result.w =
    body_orientation.w * offset.w - body_orientation.x * offset.x -
    body_orientation.y * offset.y - body_orientation.z * offset.z;
  return result;
}

}  // namespace

RvizVisualizer::RvizVisualizer(
  rclcpp::Node & node,
  const std::string & marker_topic)
: clock_(node.get_clock())
{
  // transient_local이라 depth가 곧 늦게 붙는 RViz가 받는 마커 수다.
  // 서로 다른 ns/id 마커가 20개를 넘으면 일부가 안 보이므로 여유를 둔다.
  marker_pub_ = node.create_publisher<visualization_msgs::msg::Marker>(
    marker_topic,
    rclcpp::QoS(rclcpp::KeepLast(64)).reliable().transient_local());
}

visualization_msgs::msg::Marker RvizVisualizer::baseMarker(
  const std::string & frame_id,
  const std::string & name_space,
  const int id,
  const int type) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = clock_->now();
  marker.ns = name_space;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

void RvizVisualizer::publishLabel(
  const std::string & frame_id,
  const Point3D & position,
  const std::string & text,
  const int id,
  const float red,
  const float green,
  const float blue)
{
  auto marker = baseMarker(
    frame_id, "vehicle_labels", id,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  marker.pose.position.x = position.x;
  marker.pose.position.y = position.y;
  marker.pose.position.z = position.z + 0.8;
  marker.scale.z = 0.45;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  marker.text = text;
  marker_pub_->publish(marker);
}

void RvizVisualizer::publishBlueRov(
  const nav_msgs::msg::Odometry & odometry)
{
  const std::string frame_id =
    odometry.header.frame_id.empty() ? "map" : odometry.header.frame_id;
  auto marker = baseMarker(
    frame_id, "bluerov2", 0, visualization_msgs::msg::Marker::ARROW);
  marker.pose = odometry.pose.pose;
  marker.scale.x = 1.2;
  marker.scale.y = 0.45;
  marker.scale.z = 0.45;
  marker.color.g = 1.0F;
  marker.color.a = 0.95F;
  marker_pub_->publish(marker);

  publishLabel(
    frame_id,
    {odometry.pose.pose.position.x,
     odometry.pose.pose.position.y,
     odometry.pose.pose.position.z},
    "BlueROV2", 0, 0.1F, 1.0F, 0.1F);
}

void RvizVisualizer::publishTorpedo(
  const nav_msgs::msg::Odometry & odometry,
  const BoxObstacle & barrier_shape,
  const bool show_barrier)
{
  const std::string frame_id =
    odometry.header.frame_id.empty() ? "map" : odometry.header.frame_id;
  auto marker = baseMarker(
    frame_id, "torpedo", 0, visualization_msgs::msg::Marker::ARROW);
  marker.pose = odometry.pose.pose;
  marker.pose.orientation = torpedoArrowOrientation(
    odometry.pose.pose.orientation);
  marker.scale.x = 1.6;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.1F;
  marker.color.g = 0.45F;
  marker.color.b = 1.0F;
  marker.color.a = 0.95F;
  marker_pub_->publish(marker);

  if (show_barrier) {
    auto barrier = baseMarker(
      frame_id, "torpedo_live_barrier", 0,
      visualization_msgs::msg::Marker::CUBE);
    barrier.pose.position = odometry.pose.pose.position;
    barrier.scale.x = barrier_shape.size_x;
    barrier.scale.y = barrier_shape.size_y;
    barrier.scale.z = barrier_shape.size_z;
    barrier.color.b = 1.0F;
    barrier.color.a = 0.18F;
    marker_pub_->publish(barrier);
  }

  publishLabel(
    frame_id,
    {odometry.pose.pose.position.x,
     odometry.pose.pose.position.y,
     odometry.pose.pose.position.z},
    "Torpedo", 1, 0.2F, 0.55F, 1.0F);
}

void RvizVisualizer::publishTelemetry(
  const common::StateSnapshot & snapshot)
{
  if (!snapshot.bluerov_odometry.metadata.valid) {
    return;
  }

  const auto & odometry = snapshot.bluerov_odometry.message;
  const auto & position = odometry.pose.pose.position;
  const auto & velocity = odometry.twist.twist.linear;
  const std::string frame_id =
    odometry.header.frame_id.empty() ? "map" : odometry.header.frame_id;

  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << "BlueROV xyz [" << position.x << ", " << position.y << ", "
       << position.z << "] m\n"
       << "Odom v [" << velocity.x << ", " << velocity.y << ", "
       << velocity.z << "] m/s";

  if (snapshot.dvl.metadata.valid) {
    const auto & dvl = snapshot.dvl.message.velocity.twist.linear;
    text << "\nDVL v [" << dvl.x << ", " << dvl.y << ", " << dvl.z
         << "] m/s";
  }
  if (snapshot.depth.metadata.valid) {
    text << "\nDepth z " << snapshot.depth.message.point.z << " m";
  }
  if (snapshot.pressure.metadata.valid) {
    text << "\nPressure " << snapshot.pressure.message.fluid_pressure << " Pa";
  }
  if (snapshot.imu.metadata.valid) {
    const auto & angular_velocity = snapshot.imu.message.angular_velocity;
    text << "\nIMU w [" << angular_velocity.x << ", "
         << angular_velocity.y << ", " << angular_velocity.z << "] rad/s";
  }
  if (snapshot.torpedo_odometry.metadata.valid) {
    const auto & torpedo =
      snapshot.torpedo_odometry.message.pose.pose.position;
    text << "\nTorpedo xyz [" << torpedo.x << ", " << torpedo.y << ", "
         << torpedo.z << "] m";
  }
  if (snapshot.mission_goal.metadata.valid) {
    const auto & target = snapshot.mission_goal.message.point;
    text << "\nMission xyz [" << target.x << ", " << target.y << ", "
         << target.z << "] m";
  }

  auto marker = baseMarker(
    frame_id, "telemetry", 0,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  marker.pose.position = position;
  marker.pose.position.z += 2.0;
  marker.scale.z = 0.28;
  marker.color.r = 0.95F;
  marker.color.g = 0.95F;
  marker.color.b = 0.95F;
  marker.color.a = 1.0F;
  marker.text = text.str();
  marker_pub_->publish(marker);
}

void RvizVisualizer::publishStatus(
  const std::string & frame_id,
  const Point3D & position,
  const std::string & planner_name,
  const bool avoid_mode,
  const bool hit_latched,
  const double hit_distance,
  const bool show_avoided,
  const double avoided_distance)
{
  auto mode = baseMarker(
    frame_id, "planning_status", 0,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  mode.pose.position.x = position.x;
  mode.pose.position.y = position.y;
  mode.pose.position.z = position.z + 3.4;
  mode.scale.z = 0.5;
  if (avoid_mode) {
    mode.color.r = 1.0F;
    mode.color.g = 0.6F;
    mode.color.b = 0.1F;
  } else {
    mode.color.r = 0.6F;
    mode.color.g = 1.0F;
    mode.color.b = 0.6F;
  }
  mode.color.a = 1.0F;
  // 어떤 알고리즘으로 도는 중인지 한눈에 보이게 함께 적는다.
  mode.text = planner_name + " | " + (avoid_mode ? "AVOID" : "NORMAL");
  marker_pub_->publish(mode);

  auto event = baseMarker(
    frame_id, "engagement_event", 0,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  event.pose.position.x = position.x;
  event.pose.position.y = position.y;
  event.pose.position.z = position.z + 4.4;
  if (hit_latched) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << "HIT! (" << hit_distance << " m)";
    event.text = text.str();
    event.scale.z = 1.0;
    event.color.r = 1.0F;
    event.color.a = 1.0F;
  } else if (show_avoided) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(1)
         << "AVOIDED (min " << avoided_distance << " m)";
    event.text = text.str();
    event.scale.z = 0.6;
    event.color.g = 1.0F;
    event.color.a = 1.0F;
  } else {
    // 이벤트가 없으면 transient_local로 남은 텍스트를 지운다.
    event.action = visualization_msgs::msg::Marker::DELETE;
  }
  marker_pub_->publish(event);
}

// 예측 통로는 계획 결과가 아니라 매 틱의 어뢰 상태를 그린다. A*는 충돌
// 때만 재계획하므로(측정상 한 판에 1회) 계획 결과에 실어 보내면 어뢰가
// 계속 움직이는 동안 박스가 옛 위치에 그대로 남는다.
void RvizVisualizer::publishTorpedoCorridor(
  const std::string & frame_id,
  const std::vector<BoxObstacle> & torpedo_obstacles)
{
  auto torpedo_center = baseMarker(
    frame_id, "torpedo_center", 0, visualization_msgs::msg::Marker::SPHERE);
  if (torpedo_obstacles.empty()) {
    torpedo_center.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    const auto & live = torpedo_obstacles.front();
    torpedo_center.pose.position.x = live.center.x;
    torpedo_center.pose.position.y = live.center.y;
    torpedo_center.pose.position.z = live.center.z;
    torpedo_center.scale.x = torpedo_center.scale.y =
      torpedo_center.scale.z = 0.5;
    torpedo_center.color.b = 1.0F;
    torpedo_center.color.a = 1.0F;
  }
  marker_pub_->publish(torpedo_center);

  // 예측 통로 박스는 CUBE_LIST 하나로 묶어 발행한다. 박스마다 마커를
  // 따로 쏘면 어뢰 탐지 중 5Hz x 30여 개가 되어 RViz가 눈에 띄게 느려진다.
  // 모든 박스가 같은 크기라 CUBE_LIST의 단일 scale로 표현되고, 색은 점별
  // colors로 준다. points[0]은 어뢰 현재 위치 박스(파랑), 나머지는 예측
  // 통로(연한 시안)다.
  auto barriers = baseMarker(
    frame_id, "torpedo_barrier", 0,
    visualization_msgs::msg::Marker::CUBE_LIST);
  if (torpedo_obstacles.empty()) {
    // NORMAL 모드나 DVO 단독: transient_local이라 지우지 않으면 남는다.
    barriers.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    barriers.scale.x = torpedo_obstacles.front().size_x;
    barriers.scale.y = torpedo_obstacles.front().size_y;
    barriers.scale.z = torpedo_obstacles.front().size_z;
    barriers.points.reserve(torpedo_obstacles.size());
    barriers.colors.reserve(torpedo_obstacles.size());
    for (std::size_t index = 0; index < torpedo_obstacles.size(); ++index) {
      geometry_msgs::msg::Point center;
      center.x = torpedo_obstacles[index].center.x;
      center.y = torpedo_obstacles[index].center.y;
      center.z = torpedo_obstacles[index].center.z;
      barriers.points.push_back(center);

      std_msgs::msg::ColorRGBA color;
      if (index == 0) {
        color.b = 1.0F;
        color.a = 0.2F;
      } else {
        color.g = 0.7F;
        color.b = 1.0F;
        color.a = 0.12F;
      }
      barriers.colors.push_back(color);
    }
  }
  marker_pub_->publish(barriers);
}

void RvizVisualizer::publishScene(
  const std::string & frame_id,
  const Point3D & current,
  const Point3D & goal,
  const std::vector<Point3D> & path)
{
  auto robot = baseMarker(
    frame_id, "uuv_position", 0, visualization_msgs::msg::Marker::SPHERE);
  robot.pose.position.x = current.x;
  robot.pose.position.y = current.y;
  robot.pose.position.z = current.z;
  robot.scale.x = robot.scale.y = robot.scale.z = 0.5;
  robot.color.g = 1.0F;
  robot.color.a = 0.9F;
  marker_pub_->publish(robot);

  auto goal_marker = baseMarker(
    frame_id, "astar_goal", 0, visualization_msgs::msg::Marker::SPHERE);
  goal_marker.pose.position.x = goal.x;
  goal_marker.pose.position.y = goal.y;
  goal_marker.pose.position.z = goal.z;
  goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.8;
  goal_marker.color.r = 1.0F;
  goal_marker.color.a = 1.0F;
  marker_pub_->publish(goal_marker);

  auto arrow = baseMarker(
    frame_id, "goal_arrow", 0, visualization_msgs::msg::Marker::ARROW);
  geometry_msgs::msg::Point arrow_start;
  arrow_start.x = current.x;
  arrow_start.y = current.y;
  arrow_start.z = current.z;
  geometry_msgs::msg::Point arrow_end;
  arrow_end.x = goal.x;
  arrow_end.y = goal.y;
  arrow_end.z = goal.z;
  arrow.points = {arrow_start, arrow_end};
  arrow.scale.x = 0.08;
  arrow.scale.y = 0.18;
  arrow.scale.z = 0.30;
  arrow.color.r = 1.0F;
  arrow.color.g = 1.0F;
  arrow.color.a = 1.0F;
  marker_pub_->publish(arrow);

  auto path_marker = baseMarker(
    frame_id, "astar_path", 0, visualization_msgs::msg::Marker::LINE_STRIP);
  path_marker.scale.x = 0.12;
  path_marker.color.r = 1.0F;
  path_marker.color.g = 0.55F;
  path_marker.color.a = 1.0F;
  path_marker.points.reserve(path.size());
  for (const auto & waypoint : path) {
    geometry_msgs::msg::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    point.z = waypoint.z;
    path_marker.points.push_back(point);
  }
  marker_pub_->publish(path_marker);
}

}  // namespace bluerov_integration::team_min
