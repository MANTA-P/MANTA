#pragma once

// team_min 판단 코어(PlanningCore)의 경계 언어다. 코어가 ROS 없이
// 컴파일·테스트되도록 이 파일과 planning_core.*에는 rclcpp/nav_msgs/
// geometry_msgs include를 두지 않는다. ROS 메시지 변환은 전부
// planning_module(어댑터)에서 한다.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "bluerov_integration/team_min/astar_planner.hpp"

namespace bluerov_integration::team_min
{

// 어떤 알고리즘으로 계획할지다. A*와 DVO를 각각 단독으로 돌려 비교하기
// 위해 분리했고, kHybrid는 A*(전역) + DVO(국소) 결합이다.
// 런타임에 바뀌므로(planning.planner 파라미터) config가 아니라 매 틱
// 입력(PlanningInput)과 요청(PlanRequest)에 실어 나른다.
enum class PlannerType
{
  kAStar,
  kDynamicVO,
  kHybrid,
  // 시공간 A*: 어뢰를 정적 통로가 아니라 시각별 점유로 다룬다.
  // 통과·대기·가로지르기가 선택지에 들어오고, 속도가 경로에서 바로 나온다.
  kSpaceTime
};

inline const char * plannerName(const PlannerType planner)
{
  switch (planner) {
    case PlannerType::kAStar:
      return "astar";
    case PlannerType::kDynamicVO:
      return "dvo";
    case PlannerType::kHybrid:
      return "hybrid";
    case PlannerType::kSpaceTime:
      return "spacetime";
  }
  return "unknown";
}

enum class TorpedoGuidanceLaw
{
  kUnknown,
  kProportionalNavigation,
  kPurePursuit
};

inline const char * guidanceLawName(const TorpedoGuidanceLaw law)
{
  switch (law) {
    case TorpedoGuidanceLaw::kUnknown:
      return "unknown";
    case TorpedoGuidanceLaw::kProportionalNavigation:
      return "proportional_navigation";
    case TorpedoGuidanceLaw::kPurePursuit:
      return "pure_pursuit";
  }
  return "unknown";
}

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

  // 어뢰를 등속 직진이 아니라 "유도되는 물체"로 예측한다.
  // PN 과 순수추적의 one-step 예측 오차를 온라인 비교해 어느 유도법을
  // 쓰는지 식별하고, 그 법칙으로 곡선 궤적을 만들어 통로로 쓴다.
  // 등속 가정은 PNG 어뢰에서 깨진다 — 비켜도 매 틱 재조준하기 때문이다.
  bool guidance_enabled{true};
  double guidance_horizon_sec{6.0};
  double guidance_step_sec{0.2};
  double pn_navigation_constant{3.0};
  double pursuit_turn_rate{1.5};                   // rad/s
  double guidance_max_lateral_acceleration{30.0};  // m/s^2
  double classification_alpha{0.25};
  std::size_t classification_min_samples{5U};
  // 최상 모델 오차가 다른 모델의 이 비율보다 작아야 분류를 확정한다.
  double classification_ratio{0.8};
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

// 교전 국면. TTC(충돌까지 남은 시간)로 갈린다.
//
//   kCruise   어뢰가 멀다. A* 단독으로 임무 경로만 따라간다.
//   kApproach 접근 중. A* 경로를 DVO가 평상시 가중치로 다듬는다.
//   kBreak    임박. A*를 무시하고 DVO가 이격 최대화만 한다(급기동).
//
// 비례항법 유도는 일찍 시작한 완만한 회피를 리드각으로 상쇄한다. 그래서
// 회피는 늦게, 크게 해야 한다. 실측에서 A*는 150 m 전부터 2.0 m만 비켜
// 갔고 최근접이 0.55~0.99 m였다(피격반경 1.0 m).
enum class EngagementPhase
{
  kCruise,
  kApproach,
  kBreak
};

inline const char * phaseName(const EngagementPhase phase)
{
  switch (phase) {
    case EngagementPhase::kCruise:
      return "cruise";
    case EngagementPhase::kApproach:
      return "approach";
    case EngagementPhase::kBreak:
      return "break";
  }
  return "unknown";
}

// TTC 기반 국면 전환 설정.
//
// 세 값 모두 추정이 아니라 (a) 이 저장소의 코드에서 읽은 값이거나
// (b) 문헌에서 인용한 규칙이다. 근거를 아래에 남긴다.
//
// ── break_sec = 1.85 초 ─────────────────────────────────────────────────
// 규칙 출처: Zarchan, "Tactical and Strategic Missile Guidance".
//   비례항법(PN)에 대한 회피는 t_go ≈ 3~5 τ_g 에서 이격거리가 최대다.
//   일찍 시작하면 PN이 리드각으로 상쇄하고, 늦으면 변위가 모자란다.
//   유효항법비 N'=3 일 때 통상 4 τ_g 를 쓴다.
//
// τ_g 를 이루는 값은 전부 이 저장소에서 읽었다:
//   dave_robot_models/glider_slocum/model.sdf
//     Izz = 13.193856 kg m^2   (base_link inertial)
//     nR  = 32                 (Hydrodynamics yaw 감쇠)
//     → 기체 yaw 시상수 Izz/nR = 0.412 초
//   torpedo_control_v2/src/default_control_config.cpp
//     update_rate_hz          = 20.0  → 제어주기 0.050 초
//     png_navigation_constant = 3.0   → N' = 3
//
//   τ_g = 0.412 + 0.050 = 0.46 초,  4 τ_g = 1.85 초
//
// 이 값은 보수적이다. 시뮬 어뢰는 정확한 odometry를 그대로 받아 탐색기
// 필터가 없지만, 실제 음향호밍 어뢰는 필터 지연이 붙어 τ_g 가 더 크다.
// τ_g 가 크면 회피 개시 시점도 뒤로 밀리므로, 여기서 정한 1.85 초는
// 실제보다 이른 쪽이다(= 회피에 불리한 쪽).
//
// ── approach_sec = 6.0 초 ───────────────────────────────────────────────
// DVO 자신의 예측 지평과 맞춘다.
//   dynamic_vo_planner.hpp: prediction_horizon = 6.0
// DVO의 충돌원뿔은 6초 앞까지만 본다. 그보다 일찍 켜도 어뢰를 위협으로
// 인식하지 못하므로 계산만 늘고 하는 일이 없다.
//
// ── dwell_sec = 1.5 초 ──────────────────────────────────────────────────
// 출처: "Real-Time Obstacle Avoidance Algorithms for Unmanned Aerial and
//   Ground Vehicles" (arXiv:2506.20311). 전역<->반응 모드 전환의 C2 조건이
//   최소 체류시간 K ≈ 1.5 초를 요구한다. 없으면 경계에서 모드가 왕복한다.
// 이 저장소에서도 같은 진동을 확인했다 — 정적 A*가 재계획마다 회피 축을
// 수직<->수평으로 바꿔(60~28 m 수직, 20~14 m 수평, 10 m 다시 수직) 회피가
// 누적되지 않고 상쇄됐다.
struct TtcSwitchConfig
{
  bool enabled{true};
  double approach_sec{6.0};   // 이 아래면 DVO 개입 (kApproach)
  // 급기동 시점.
  //
  // Zarchan 의 3~5 tau_g (실측 tau_g = 0.46 s -> 1.4~2.3 s) 를 따라
  // 1.85 s 와 2.3 s 를 써 봤으나 둘 다 PNG 상대로 안 통했다
  // (모드3: 1.85 s 에서 0/6, 2.3 s 에서 1/8, 최근접 0.90 -> 0.77 m).
  //
  // 궤적 실측이 가리키는 것은 더 늦은 시점이다. ROV 는 속도가 막혀
  // 거리를 선형으로 버는데(3 m/s) 어뢰는 t^2 로 수정해 오므로, 일찍
  // 틀수록 어뢰가 따라잡을 시간을 준다:
  //   1.85 s 에 기동 -> 순변위 -0.82 m
  //   1.0  s 에 기동 -> 순변위 +1.55 m
  // 1.0 s 로 내려 12판 돌렸으나 1/12 로 1.85 s(0/6) 와 다르지 않았다.
  // 시점 조정으로는 PNG 를 못 피한다는 것이 네 번의 시도로 확인됐다.
  //
  // 이제 반전(jink)만 단독으로 시험한다. 반전에는 0.38 s(방향 확립) +
  // 0.77 s(반전) = 1.15 s 가 필요하므로 1.85 s 로 되돌린다.
  double break_sec{1.85};     // 이 아래면 급기동 (kBreak)
  // 이 아래로 내려가면 횡기동 방향을 반대로 뒤집는다(jink).
  //
  // 비례항법은 표적의 등속을 전제로 시선각속도를 없앤다. 한 방향으로만
  // 계속 피하면 어뢰가 그 해를 수렴시킨다. 방향을 뒤집으면 횡방향 속도
  // 변화량이 3.0 이 아니라 6.0 m/s 로 두 배가 되고, 어뢰는 그것을
  // tau_g 안에 따라잡아야 한다.
  //
  // 0 이면 반전하지 않는다.
  //
  // 노리는 것은 어뢰의 가속 한계가 아니라 '전환 지연'이다. 어뢰 코드
  // (torpedo_control_v2/png_controller.cpp)를 보면 핀 명령이
  //   fin = clamp(소요가속도 x 1.0, +-0.50 rad)
  // 이라 소요 가속도가 0.5 m/s^2 만 넘어도 핀이 포화한다. 종말 구간의
  // 어뢰는 사실상 on/off 로 최대 타각에 붙어 있다. 게다가 롤이 거의
  // 잠겨 있다(roll_limit_rad = 0.10). 그래서 ROV 가 횡방향을 뒤집으면
  // 어뢰는 핀을 +0.5 에서 -0.5 까지 완전히 넘겨야 하고 그 전환에 시간이
  // 든다.
  //
  // 급기동 1.85 s 에서 0.38 s 뒤 방향이 서므로 1.47 s 부터 반전이
  // 가능하다. 반전에 0.77 s 가 걸려 0.70 s (= 1.5 tau_g) 남기고 끝난다.
  double reverse_sec{1.45};
  double dwell_sec{1.5};      // 최소 체류시간
  // 접근속도가 이보다 작으면 TTC를 무한으로 본다(멀어지는 중).
  double minimum_closing{0.2};
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
  TtcSwitchConfig ttc{};
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
  // 유도 모델이 만든 어뢰 곡선 예측 궤적(어댑터가 채운다).
  std::vector<Point3D> torpedo_predicted_path;
  // 이번 틱에 선택된 알고리즘이다(런타임에 바뀔 수 있다).
  PlannerType planner{PlannerType::kHybrid};
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
  // 유도 모델이 만든 곡선 예측 궤적. 비어 있으면 등속 직선으로 되돌아간다.
  std::vector<Point3D> torpedo_predicted_path;
  TorpedoGuidanceLaw torpedo_guidance{TorpedoGuidanceLaw::kUnknown};
  double torpedo_prediction_step_sec{0.2};
  bool torpedo_valid{false};
  // 이 요청을 처리할 알고리즘이다. 요청이 자기 알고리즘을 들고 다니므로
  // worker가 계산하는 동안 사용자가 모드를 바꿔도 일관되게 끝난다.
  PlannerType planner{PlannerType::kHybrid};
  std::string frame_id;
};

// 모든 플래너의 공통 반환 타입이다. 호출부(execute/publishPlan)는 어떤
// 알고리즘이 돌았는지 몰라도 되고, 분기는 runPlanner() 안에만 있다.
// 계획이 실패한 이유다. 순수 계층이 ROS 로거를 부르지 않도록 사유만
// 돌려주고, 어댑터가 이 값을 보고 로그를 낸다.
enum class PlanFailure
{
  kNone,
  kGoalInsideBarrier,   // 목표가 어뢰 배리어 안 (어뢰가 지나가면 해소)
  kStartInsideBarrier,  // 로봇이 어뢰 배리어 안
  kNoAStarPath,         // A*가 경로를 못 찾음
  kNoSafeLocalPath,     // DVO가 안전한 국소 경로를 못 찾음
  kSpaceTimeUnreachable,  // 국소 목표가 시간창 밖 (탐색 안 함)
  kNoSpaceTimePath      // 시공간 A*가 경로를 못 찾음
};

struct PlanResult
{
  std::vector<Point3D> path;
  std::vector<BoxObstacle> obstacles;  // RViz 표시용(A* 예측 통로 등)
  bool valid{false};
  bool vo_active{false};               // 로그 표시용(DVO가 개입했는가)
  PlanFailure failure{PlanFailure::kNone};
  // 계획 실패가 "갈 곳이 없다"는 뜻이라 정지시켜야 하는 경우다.
  // (회피 경로를 못 찾은 상황. A*가 한 번 실패한 것과는 구분한다 —
  //  그때는 기존 경로를 유지하고 다음 틱에 재시도한다.)
  bool stop_requested{false};
  // 시공간 A*가 확장한 노드 수(성능 로그용, 다른 플래너는 0).
  std::size_t expanded_nodes{0};
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
  // 이번 틱의 어뢰 예측 통로다. 계획을 안 돌린 틱에도 채워지므로 어댑터가
  // 매 틱 다시 그려 마커가 어뢰를 따라간다(비어 있으면 지운다).
  std::vector<BoxObstacle> torpedo_corridor;

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

  // 교전이 끝나야 이격거리가 남으므로, 어뢰가 교전반경 안에 계속 붙어
  // 쫓아다니면 판이 끝날 때까지 "얼마나 가까웠는지"가 기록되지 않는다.
  // 실측 48판 중 성공 13판의 절반이 이 경우였다. 진행 중에도 최소값이
  // 갱신될 때마다 남겨, 아슬아슬한 생존과 여유 있는 생존을 구분한다.
  bool engagement_min_improved{false};
  double engagement_min_distance{0.0};

  // 이번 틱의 교전 국면과 그 근거. 어댑터가 이걸 보고 플래너를 고른다.
  // 어뢰가 쓰는 것으로 식별된 유도법(로그·시각화용).
  TorpedoGuidanceLaw torpedo_guidance{TorpedoGuidanceLaw::kUnknown};
  double pn_prediction_error{0.0};
  double pursuit_prediction_error{0.0};

  EngagementPhase phase{EngagementPhase::kCruise};
  bool phase_changed{false};
  double ttc_sec{0.0};       // 무한이면 멀어지는 중
  double closing_speed{0.0};  // m/s, 양수면 접근 중

  Skip skip{Skip::kNone};
  std::string frame_id;  // 이번 틱의 계획 프레임(로봇 frame, 기본 "map")
};

}  // namespace bluerov_integration::team_min
