#pragma once

// 플래너 결과를 속도 명령으로 바꾼다.
//
// 왜 필요한가:
//   지금은 플래너가 위치점을 내보내고 PPID의 PositionP가 그걸 속도로 바꾼다
//   (v = 1.5 x 거리, 3 m/s 클램프). 그 구조에서는 플래너가 고른 "속도 크기"가
//   전달되지 않는다 — DVO가 1 m/s로 천천히 가라고 결정해도 PathFollower가
//   여전히 2 m 앞을 보므로 PositionP는 늘 최대속도를 낸다. 그래서 VO 이론의
//   핵심 회피 수단인 감속을 표현할 수 없다.
//
//   ROS nav 계열(DWA/TEB)과 해양 LOS 유도는 모두 국소 계층이 속도를 직접
//   낸다. 이 파일이 그 계층이다.
//
// 주의 — 위치 피드백을 잃으면 안 된다:
//   PositionP를 우회하면 경로 이탈 보정이 사라져 오차가 누적된다. 그래서
//   보정을 여기서 흡수한다(DWA의 경로거리 비용항, LOS의 cross-track과 같은
//   발상). withTrackCorrection()이 그 역할이다.
//
// ROS 의존 없음(planning_core와 동일 계층).

#include <vector>

#include "bluerov_integration/team_min/astar_planner.hpp"
#include "bluerov_integration/team_min/spacetime_astar.hpp"

namespace bluerov_integration::team_min
{

struct VelocityLimits
{
  double max_horizontal{3.0};    // m/s
  double max_vertical{1.5};      // m/s
  // 경로 이탈 보정 게인. PPID의 PositionP와 같은 값을 쓰면 우회 전후의
  // 추종 거동이 같아진다.
  double track_gain{1.5};
  // 보정이 회피 기동을 덮어쓰지 않도록 상한을 둔다.
  double max_correction{1.5};    // m/s
};

struct VelocityCommand
{
  Point3D velocity{};       // map 프레임, m/s
  bool valid{false};
  // 이 명령이 근거한 계획 구간 길이(초). 어댑터가 워치독 시한으로 쓴다.
  double horizon_sec{0.0};
};

// 시공간 경로의 첫 구간에서 속도를 뽑는다. (p1 - p0) / (t1 - t0).
//
// 시공간 A*는 이웃 자체가 속도 제약을 만족하도록 만들어져 있어, 여기서
// 나오는 속도는 항상 실현 가능한 값이다.
VelocityCommand velocityFromTimedPath(const std::vector<TimedPoint> & path);

// DVO가 고른 속도를 명령으로 감싼다. DVO는 이미 속도공간에서 풀지만
// 지금은 그 결과를 위치점으로 되돌려 버리고 있어, 이 함수로 직접 쓴다.
VelocityCommand velocityFromDynamicVO(
  const Point3D & selected_velocity,
  double rollout_step);

// 경로 이탈 보정을 더하고 속도 한계로 자른다.
//
// reference는 플래너 경로 위에서 로봇이 "지금 있어야 할" 점이다. 이탈이
// 없으면 보정이 0이라 플래너 속도가 그대로 나간다.
VelocityCommand withTrackCorrection(
  VelocityCommand command,
  const Point3D & current_position,
  const Point3D & reference_position,
  const VelocityLimits & limits);

// 속도를 수평·수직 한계 안으로 자른다. 방향은 유지한다.
Point3D clampVelocity(const Point3D & velocity, const VelocityLimits & limits);

}  // namespace bluerov_integration::team_min
