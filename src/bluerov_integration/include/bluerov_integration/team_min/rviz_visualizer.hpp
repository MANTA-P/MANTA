#pragma once

#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "bluerov_integration/common/data_types.hpp"
#include "bluerov_integration/team_min/astar_planner.hpp"

namespace bluerov_integration::team_min
{

// team_min 시각화 기능을 구독 노드가 아닌 출력 전용 도우미로 분리했다.
// 모든 입력은 PlanningModule이 DataHub snapshot에서 전달한다.
class RvizVisualizer
{
public:
  RvizVisualizer(rclcpp::Node & node, const std::string & marker_topic);

  void publishBlueRov(const nav_msgs::msg::Odometry & odometry);
  void publishTorpedo(
    const nav_msgs::msg::Odometry & odometry,
    const BoxObstacle & barrier_shape,
    bool show_barrier);
  void publishTelemetry(const common::StateSnapshot & snapshot);
  void publishScene(
    const std::string & frame_id,
    const Point3D & current,
    const Point3D & goal,
    const BoxObstacle & torpedo,
    const std::vector<Point3D> & path);

private:
  visualization_msgs::msg::Marker baseMarker(
    const std::string & frame_id,
    const std::string & name_space,
    int id,
    int type) const;
  void publishLabel(
    const std::string & frame_id,
    const Point3D & position,
    const std::string & text,
    int id,
    float red,
    float green,
    float blue);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace bluerov_integration::team_min
