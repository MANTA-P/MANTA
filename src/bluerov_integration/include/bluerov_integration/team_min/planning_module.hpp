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
#include "bluerov_integration/team_min/planning_config.hpp"
#include "bluerov_integration/team_min/planning_engine.hpp"
#include "bluerov_integration/team_min/rviz_visualizer.hpp"

namespace bluerov_integration::team_min
{


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
  void execute(const PlanningWork & work);
  // 계획 결과를 ROS로 내보낸다(여기부터 PathFollower -> PPID 파이프라인).
  void publishPlan(const PlanResult & result, const PlanRequest & request);
  // runPlanner가 돌려준 실패 사유를 ROS 로그로 옮긴다.
  void reportPlanFailure(PlanFailure failure, const PlanRequest & request);
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
  // 판단 코어. planning 타이머(MutuallyExclusive)에서만 update()를
  // 부르므로 잠금 없이 쓴다. planning.enabled=false면 만들지 않는다.
  std::unique_ptr<PlanningCore> core_;
  // 알고리즘 선택·조합과 전역 A* 경로 캐시는 순수 C++ 엔진이 소유한다.
  std::unique_ptr<PlanningEngine> engine_;
  // 런타임 플래너 변경 시 worker의 plan()과 reset()을 직렬화한다.
  std::mutex engine_mutex_;

  // 실험 중 재실행 없이 알고리즘을 갈아끼운다:
  //   ros2 param set /bluerov_integration_node planning.planner dvo
  // 파라미터 콜백(executor 스레드), 계획 타이머, worker가 함께 보므로
  // atomic으로 둔다. 콜백 핸들은 살아있어야 콜백이 유지된다.
  std::atomic<PlannerType> planner_{PlannerType::kHybrid};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;

  mutable std::mutex request_mutex_;
  std::condition_variable request_cv_;
  std::optional<PlanningWork> pending_request_;
  // 마지막으로 발행한 경로다. worker가 쓰고 update()가 읽으므로
  // request_mutex_ 아래에서만 접근한다(코어에는 복사로 넘긴다).
  std::vector<Point3D> last_path_;
  // 반전(jink)이 실제로 걸렸는지 로그로 남기기 위한 상태. 전환될 때만
  // 찍어 로그가 넘치지 않게 한다.
  bool reverse_active_{false};
  // 코어의 hit 래치 미러다. worker가 낡은 계획 결과의 발행을 억제할 때
  // 읽으므로 atomic으로 둔다.
  // 마지막으로 로그한 유도법(바뀔 때만 알린다)
  TorpedoGuidanceLaw last_guidance_{TorpedoGuidanceLaw::kUnknown};
  std::atomic<bool> hit_latched_{false};

  std::atomic<bool> running_{false};
  std::thread worker_;
};

}  // namespace bluerov_integration::team_min
