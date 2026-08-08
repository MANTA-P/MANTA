#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/common/data_types.hpp"
#include "bluerov_integration/team_min/planning_core.hpp"
#include "bluerov_integration/team_min/planning_types.hpp"
#include "bluerov_integration/team_min/rviz_visualizer.hpp"

namespace bluerov_integration::team_min
{

// 통합 노드가 채우는 설정이다. 필드 배치는 기존과 동일하게 유지한다
// (bluerov_integration_node.cpp가 평면 필드명으로 직접 접근한다).
// 알고리즘 값은 어댑터가 PlanningCoreConfig로 옮겨 코어에 넘기고,
// ROS 토픽명은 어댑터만 안다.
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
  PredictionConfig prediction{};
  ReplanPolicyConfig replan{};
  AvoidConfig avoid{};
  std::string path_topic{"/uuv/reference_path"};
  std::string current_point_topic{"/uuv/current_position_point"};
  std::string goal_point_topic{"/uuv/goal_point"};
  std::string torpedo_point_topic{"/uuv/torpedo_center_point"};
  std::string marker_topic{"/rviz/uuv_astar_markers"};
};

// ROS 어댑터다. "StateSnapshot -> PlanningInput 변환 -> PlanningCore 판단
// -> Decision에 따른 발행/로그/마커"만 담당하고, 판단 로직은 전부
// ROS를 모르는 PlanningCore(planning_core.hpp)에 있다.
// A*는 전용 worker에서 실행한다. update()는 매 틱 판단만 하고 재계획
// 요청만 교체하므로 ROS 센서 콜백이나 제어 타이머를 막지 않는다.
class PlanningModule
{
public:
  PlanningModule(rclcpp::Node & node, PlanningConfig config);
  ~PlanningModule();

  void update(const common::StateSnapshot & snapshot);
  void stop();

private:
  static Point3D positionOf(const nav_msgs::msg::Odometry & odometry);
  static VehicleState toVehicleState(
    const common::ReceivedSample<nav_msgs::msg::Odometry> & sample);
  PlanningInput makePlanningInput(
    const common::StateSnapshot & snapshot) const;
  void handleDecision(const Decision & decision, const PlanningInput & input);
  void publishStopPath(const std::string & frame_id);
  void workerLoop();
  void execute(const PlanRequest & request);
  void publishPoint(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
    const Point3D & point,
    const std::string & frame_id);

  PlanningConfig config_;
  // worker(execute)가 통로/맵 생성에 쓰는 코어 설정 사본이다(불변).
  PlanningCoreConfig core_config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr current_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr torpedo_point_pub_;
  std::unique_ptr<RvizVisualizer> visualizer_;
  // 판단 코어. planning 타이머(MutuallyExclusive)에서만 update()를
  // 부르므로 잠금 없이 쓴다. planning.enabled=false면 만들지 않는다.
  std::unique_ptr<PlanningCore> core_;

  mutable std::mutex request_mutex_;
  std::condition_variable request_cv_;
  std::optional<PlanRequest> pending_request_;
  // 마지막으로 발행한 경로다. worker가 쓰고 update()가 읽으므로
  // request_mutex_ 아래에서만 접근한다(코어에는 복사로 넘긴다).
  std::vector<Point3D> last_path_;
  // 코어의 hit 래치 미러다. worker가 낡은 계획 결과의 발행을 억제할 때
  // 읽으므로 atomic으로 둔다.
  std::atomic<bool> hit_latched_{false};

  std::atomic<bool> running_{false};
  std::thread worker_;
};

}  // namespace bluerov_integration::team_min
