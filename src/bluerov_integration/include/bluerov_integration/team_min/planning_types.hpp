#pragma once

// team_min 판단 코어(PlanningCore)의 경계 언어다. 코어가 ROS 없이
// 컴파일·테스트되도록 이 파일과 planning_core.*에는 rclcpp/nav_msgs/
// geometry_msgs include를 두지 않는다. ROS 메시지 변환은 전부
// planning_module(어댑터)에서 한다.

#include <cstdint>
#include <optional>
#include <string>

#include "bluerov_integration/team_min/astar_planner.hpp"

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

// 코어가 아는 설정 전부다. ROS 토픽명은 어댑터(PlanningConfig)에만 있고
// 코어는 토픽 문자열을 모른다.
struct PlanningCoreConfig
{
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
};

// nav_msgs::Odometry 대체다. twist는 일부러 담지 않는다 — 어뢰 속도
// 추정은 body 프레임 문제 때문에 map 프레임 위치 차분으로만 한다.
struct VehicleState
{
  bool valid{false};
  Point3D position{};
  std::string frame_id;
  std::uint64_t sequence{0};
  double stamp_sec{0.0};     // 메시지 header.stamp(sim time), 없으면 0
  double received_sec{0.0};  // 수신 시각(steady clock 기준 초)
};

struct GoalSample
{
  bool valid{false};
  Point3D point{};
  std::string frame_id;
  std::uint64_t sequence{0};
};

// 매 틱 코어에 넘기는 입력이다. 코어는 clock을 직접 부르지 않고
// now_sec(steady clock 기준 초)을 입력으로 받는다(테스트 가능성).
struct PlanningInput
{
  double now_sec{0.0};
  VehicleState bluerov;
  VehicleState torpedo;
  GoalSample goal;
};

// worker 스레드가 실행할 계획 요청이다.
struct PlanRequest
{
  Point3D start;
  // Dynamic VO: map-frame ROV velocity estimated from odometry positions.
  Point3D robot_velocity;
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

// 매 틱 판단 결과다. 판단은 코어가 하고, 행동(발행·로그·마커)은
// 어댑터가 이 값을 보고 수행한다.
struct Decision
{
  // 계획을 건너뛴 이유다. 어댑터가 throttled warn을 낸다.
  enum class Skip
  {
    kNone,
    kHitLatched,
    kTorpedoFrameMismatch,
    kGoalFrameMismatch,
  };

  bool avoid_mode{false};
  bool mode_changed{false};          // 이번 틱에 NORMAL<->AVOID 전환 발생
  // Dynamic VO: A* is refreshed only when this flag is true; VO may run every tick.
  bool global_replan_required{false};
  std::optional<PlanRequest> plan_request;

  // 이번 틱에 발생한 이벤트(각각 1회성)
  bool engagement_started{false};
  double engagement_distance{0.0};
  bool hit{false};                   // 피격 판정 -> stop_requested와 함께
  bool avoided{false};               // 어뢰가 지나가서 회피 성공
  bool torpedo_lost_avoided{false};  // 교전 중 어뢰 소실로 회피 마감
  bool reset{false};                 // 새 목표로 hit 래치 해제
  bool stop_requested{false};        // 빈 경로 발행(정지) 필요

  // 표시용 현재 상태(래치·타이머 반영값)
  bool hit_latched{false};
  double hit_distance{0.0};
  bool show_avoided{false};
  double avoided_min_distance{0.0};

  Skip skip{Skip::kNone};
  std::string frame_id;  // 이번 틱의 계획 프레임(로봇 frame, 기본 "map")
};

}  // namespace bluerov_integration::team_min
