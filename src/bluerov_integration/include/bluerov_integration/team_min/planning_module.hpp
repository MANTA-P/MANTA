#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/common/data_types.hpp"
#include "bluerov_integration/team_min/astar_planner.hpp"
#include "bluerov_integration/team_min/rviz_visualizer.hpp"

namespace bluerov_integration::team_min
{

struct PlanningConfig
{
  bool enabled{true};
  bool use_dynamic_map{true};
  bool use_target_topic_for_goal{true};
  double goal_offset_x{100.0};
  double goal_offset_y{0.0};
  double goal_offset_z{0.0};
  double map_padding_x{10.0};
  double map_padding_y{15.0};
  double map_padding_z{15.0};
  double torpedo_replan_distance{0.5};
  double robot_replan_distance{1.0};
  double goal_replan_distance{0.1};
  GridMapConfig fixed_map{};
  AStarOptions astar{};
  BoxObstacle torpedo_barrier{};
  std::string path_topic{"/uuv/reference_path"};
  std::string current_point_topic{"/uuv/current_position_point"};
  std::string goal_point_topic{"/uuv/goal_point"};
  std::string torpedo_point_topic{"/uuv/torpedo_center_point"};
  std::string marker_topic{"/rviz/uuv_astar_markers"};
};

// A*는 전용 worker에서 실행한다. update()는 최신 snapshot을 검토하고
// 재계획 요청만 교체하므로 ROS 센서 콜백이나 제어 타이머를 막지 않는다.
class PlanningModule
{
public:
  PlanningModule(rclcpp::Node & node, PlanningConfig config);
  ~PlanningModule();

  void update(const common::StateSnapshot & snapshot);
  void stop();

private:
  struct PlanningRequest
  {
    Point3D start;
    Point3D goal;
    Point3D torpedo;
    std::string frame_id;
  };

  static Point3D positionOf(const nav_msgs::msg::Odometry & odometry);
  static bool framesCompatible(
    const std::string & first,
    const std::string & second);
  bool needsReplan(const PlanningRequest & request) const;
  GridMapConfig planningMap(const PlanningRequest & request) const;
  void workerLoop();
  void execute(const PlanningRequest & request);
  void publishPoint(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
    const Point3D & point,
    const std::string & frame_id);

  PlanningConfig config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr current_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr torpedo_point_pub_;
  std::unique_ptr<RvizVisualizer> visualizer_;

  mutable std::mutex request_mutex_;
  std::condition_variable request_cv_;
  std::optional<PlanningRequest> pending_request_;
  std::optional<PlanningRequest> last_requested_;
  std::optional<Point3D> fixed_goal_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

}  // namespace bluerov_integration::team_min
