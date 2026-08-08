#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/common/data_types.hpp"
#include "bluerov_integration/team_min/astar_planner.hpp"
#include "bluerov_integration/team_min/rviz_visualizer.hpp"

namespace bluerov_integration::team_min
{

// 어뢰 진행방향 앞에 예측 박스를 깔아 통로(corridor)를 만드는 설정이다.
// 어뢰가 예측대로 움직이는 동안 점유 격자가 유지되어 경로가 고정된다.
struct PredictionConfig
{
  bool enabled{true};
  double horizon_sec{4.0};      // 예측 시간(어뢰 ~12 m/s가 4초간 가는 거리 커버)
  double spacing{1.5};          // 박스 간격(박스 크기의 절반이면 겹침)
  int max_boxes{32};            // 예측 박스 상한(통로 길이 = spacing*max_boxes = 48 m)
  double min_speed{0.2};        // m/s, 이하면 예측을 끈다(노이즈 게이트)
  double velocity_alpha{0.3};   // 속도 EMA 계수
  double start_clearance{0.5};  // 로봇/목표를 덮는 박스 스킵 여유
};

// 재계획 정책이다. collision_only가 true면 어뢰/로봇 이동 거리 대신
// "기존 경로가 새 통로와 충돌하거나 로봇이 경로를 이탈했을 때"만
// 재계획한다(히스테리시스). 어뢰가 예측대로 움직이는 동안 A*가 아예
// 다시 돌지 않으므로 경로가 완전히 고정된다.
struct ReplanPolicyConfig
{
  bool collision_only{true};
  double path_deviation_distance{1.5};  // 로봇-경로 이탈 허용치(m)
  double collision_margin{0.3};         // 경로-박스 충돌 판정 여유(m)
  double min_interval_sec{0.5};         // 재계획 요청 최소 간격(초)
};

// 어뢰 탐지/교전 판정 설정이다. 탐지는 odometry 수신 신선도로 판단해
// NORMAL(장애물 없이 계획)/AVOID(통로 회피) 모드를 가르고, 교전 결과
// (HIT/AVOIDED)는 ROV-어뢰 최근접 거리로 판정한다.
struct AvoidConfig
{
  double torpedo_timeout_sec{2.0};  // 이 시간 내 수신이 없으면 NORMAL 복귀
  double engage_radius{30.0};       // 교전 시작 반경(m)
  double hit_radius{1.0};           // 피격 판정 최근접 거리(m)
};

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
    // 추정된 어뢰 속도다. 예측 통로 생성에만 쓰고 needsReplan 비교에서는
    // 제외한다(속도는 항상 조금씩 변하므로 비교하면 매번 재계획된다).
    Point3D torpedo_velocity;
    // false면 어뢰 미탐지(NORMAL 모드)로, torpedo/torpedo_velocity 값은
    // 무시되고 장애물 없이 계획한다.
    bool torpedo_valid{false};
    std::string frame_id;
  };

  static Point3D positionOf(const nav_msgs::msg::Odometry & odometry);
  static bool framesCompatible(
    const std::string & first,
    const std::string & second);
  static bool boxContains(
    const BoxObstacle & box,
    const Point3D & point,
    double margin);
  static double closestApproachDistance(
    const Point3D & rov_previous,
    const Point3D & rov_current,
    const Point3D & torpedo_previous,
    const Point3D & torpedo_current);
  bool needsReplan(
    const PlanningRequest & request,
    const std::vector<BoxObstacle> & obstacles) const;
  void updateTorpedoVelocity(
    const common::ReceivedSample<nav_msgs::msg::Odometry> & sample);
  void updateEngagement(
    const common::StateSnapshot & snapshot,
    bool torpedo_detected,
    const Point3D & rov_position,
    const std::string & frame_id);
  void publishStopPath(const std::string & frame_id);
  std::vector<BoxObstacle> buildTorpedoObstacles(
    const PlanningRequest & request) const;
  GridMapConfig planningMap(
    const PlanningRequest & request,
    const std::vector<BoxObstacle> & obstacles) const;
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
  // 마지막으로 발행한 경로다. worker가 쓰고 update()가 읽으므로
  // request_mutex_ 아래에서만 접근한다.
  std::vector<Point3D> last_path_;
  std::chrono::steady_clock::time_point last_replan_request_time_{};

  // 어뢰 속도 추정(위치 차분 + EMA) 상태다. update()를 부르는 planning
  // 타이머(MutuallyExclusive 그룹)에서만 접근하므로 별도 잠금이 필요 없다.
  bool have_torpedo_history_{false};
  bool have_torpedo_velocity_{false};
  std::uint64_t last_torpedo_sequence_{0};
  Point3D last_torpedo_position_{};
  std::chrono::steady_clock::time_point last_torpedo_stamp_{};
  // use_sim_time에서 RTF(실시간 배율)와 무관하게 dt를 재기 위한 메시지 스탬프.
  rclcpp::Time last_torpedo_msg_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_torpedo_frame_{};
  Point3D torpedo_velocity_{};

  // NORMAL/AVOID 모드와 교전(HIT/AVOIDED) 판정 상태다. update() 타이머
  // 스레드 전용이라 잠금이 없고, hit_latched_만 worker가 낡은 계획 결과의
  // 발행을 억제할 때 읽으므로 atomic으로 둔다.
  bool avoid_mode_{false};
  bool engaged_{false};
  double engagement_min_distance_{0.0};
  std::atomic<bool> hit_latched_{false};
  double hit_distance_{0.0};
  std::uint64_t hit_mission_sequence_{0};
  bool have_avoided_event_{false};
  double avoided_min_distance_{0.0};
  std::chrono::steady_clock::time_point avoided_event_time_{};
  bool have_engagement_history_{false};
  Point3D previous_rov_position_{};
  Point3D previous_torpedo_position_{};

  std::atomic<bool> running_{false};
  std::thread worker_;
};

}  // namespace bluerov_integration::team_min
