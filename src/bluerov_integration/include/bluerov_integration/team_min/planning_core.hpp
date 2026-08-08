#pragma once

// team_min 판단 코어다. ROS를 전혀 모른다(rclcpp include 금지) — 어뢰
// 속도 추정, NORMAL/AVOID 모드, 교전(HIT/AVOIDED) 판정, collision-only
// 재계획 정책을 담당하고, 행동(발행·로그)은 어댑터(PlanningModule)가 한다.

#include <optional>
#include <string>
#include <vector>

#include "bluerov_integration/team_min/planning_types.hpp"

namespace bluerov_integration::team_min
{

// ---- 순수 자유 함수: worker 스레드에서도 호출된다(무상태, thread-safe) ----

bool framesCompatible(const std::string & first, const std::string & second);

bool boxContains(
  const BoxObstacle & box,
  const Point3D & point,
  double margin);

// 샘플이 유효하고 timeout_sec 내에 수신됐는지(어뢰 탐지 판정과 동일 식).
bool sampleFresh(
  const VehicleState & sample,
  double now_sec,
  double timeout_sec);

// 어뢰의 진행방향 앞에 겹치는 예측 박스를 깔아 통로(corridor)를 만든다.
// obstacles[0]은 항상 현재 위치 박스, torpedo_valid=false면 빈 벡터.
std::vector<BoxObstacle> buildTorpedoObstacles(
  const PlanRequest & request,
  const PlanningCoreConfig & config);

GridMapConfig planningMap(
  const PlanRequest & request,
  const std::vector<BoxObstacle> & obstacles,
  const PlanningCoreConfig & config);

// ---- 매 틱 판단 코어 ----
// 단일 스레드(계획 타이머) 전용이라 mutex가 없다. last_path는 내부 공유
// 상태가 아니라 명시적 입력이다(worker가 만든 최신 경로를 어댑터가
// 자기 mutex로 관리해 복사로 넘긴다).
class PlanningCore
{
public:
  // 설정이 잘못되면 std::invalid_argument를 던진다(기존 검증 메시지 유지).
  explicit PlanningCore(PlanningCoreConfig config);

  Decision update(
    const PlanningInput & input,
    const std::vector<Point3D> & last_path);

private:
  static double closestApproachDistance(
    const Point3D & rov_previous,
    const Point3D & rov_current,
    const Point3D & torpedo_previous,
    const Point3D & torpedo_current);

  void updateTorpedoVelocity(const VehicleState & torpedo);
  void updateEngagement(
    const PlanningInput & input,
    bool torpedo_detected,
    const Point3D & rov_position,
    Decision & decision);
  bool needsReplan(
    const PlanRequest & request,
    const std::vector<BoxObstacle> & obstacles,
    const std::vector<Point3D> & last_path,
    double now_sec) const;

  PlanningCoreConfig config_;

  // 어뢰 속도 추정(위치 차분 + EMA) 상태
  bool have_torpedo_history_{false};
  bool have_torpedo_velocity_{false};
  std::uint64_t last_torpedo_sequence_{0};
  Point3D last_torpedo_position_{};
  double last_torpedo_received_sec_{0.0};
  double last_torpedo_stamp_sec_{0.0};
  std::string last_torpedo_frame_{};
  Point3D torpedo_velocity_{};

  // 모드/교전(HIT/AVOIDED) 상태
  bool avoid_mode_{false};
  bool engaged_{false};
  double engagement_min_distance_{0.0};
  bool hit_latched_{false};
  double hit_distance_{0.0};
  std::uint64_t hit_mission_sequence_{0};
  bool have_avoided_event_{false};
  double avoided_min_distance_{0.0};
  double avoided_event_sec_{0.0};
  bool have_engagement_history_{false};
  Point3D previous_rov_position_{};
  Point3D previous_torpedo_position_{};

  // 재계획 정책 상태
  std::optional<PlanRequest> last_requested_;
  std::optional<Point3D> fixed_goal_;
  double last_replan_request_sec_{-1.0e18};
};

}  // namespace bluerov_integration::team_min
