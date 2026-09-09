#pragma once

// 시공간 A*: 노드를 (x, y, z, t)로 두고 "그 시각에 그 칸이 비었는가"로
// 점유를 판정한다.
//
// 기존 3D A*와의 차이는 어뢰를 다루는 방식이다. 기존에는 어뢰의 미래 궤적을
// 정적 통로(박스 19개)로 부풀려 막았고, 그래서 "어뢰가 지나간 뒤 그 자리를
// 통과한다"는 해가 원천적으로 배제됐다. 시공간에서는 시각별로 판정하므로
// 통과·대기·가로지르기가 전부 선택지에 들어온다.
//
// 부수 효과로 속도가 공짜로 나온다 — 연속 waypoint의 Δ위치/Δt가 곧 속도라
// 별도 속도 프로파일 생성이 필요 없다.
//
// 이 파일과 spacetime_astar.cpp에는 ROS 의존이 없다(planning_core와 동일).

#include <cstddef>
#include <vector>

#include "bluerov_integration/team_min/astar_planner.hpp"

namespace bluerov_integration::team_min
{

// 등속으로 움직이는 직육면체 장애물. center는 t=0 기준이다.
struct MovingBox
{
  Point3D center{};
  Point3D velocity{};
  double half_x{1.5};
  double half_y{1.5};
  double half_z{1.5};
};

struct TimedPoint
{
  Point3D point{};
  double time{0.0};   // 출발 시점 기준 경과 초
};

// 오프라인 벤치마크로 정한 기본값이다(scratchpad/st_astar_bench).
//
// 핵심 제약: 분기비율 = max_speed * dt / resolution >= 2 여야 한다.
// 이 값이 1이면 대각선 변위(1.41 x resolution)가 한 스텝 상한을 넘어
// 잘려나가고 축 이동만 남는다. 그러면 우회가 극도로 비싸져 목표에 못 닿고
// A*가 창 전체를 뒤진다(측정: 172 ms, 9.7 MB).
//
// 기본값 1.0 m / 1.0 s는 비율 3.0이고, 측정상 노드 8개 / 0.13 ms / 72 KB로
// 기존 3D A*(0.74 ms)보다 5배 이상 빠르다.
struct SpaceTimeOptions
{
  double resolution{1.0};          // m
  double dt{1.0};                  // s
  double horizon_sec{8.0};         // 시간창 길이
  double half_extent_xy{40.0};     // 로봇 기준 창 반폭
  double half_extent_z{10.0};
  double max_speed{3.0};           // m/s (수평)
  double max_vertical_speed{1.5};  // m/s
  double heuristic_weight{1.2};    // 기존 A*와 동일한 가중치
  double safety_margin{0.5};       // 장애물 여유
  // 국소 목표가 시간창 안에 도달 가능해야 한다. 도달 불가한 목표를 주면
  // A*가 창 전체를 뒤지므로, 이 비율을 넘으면 탐색 전에 포기한다.
  double reachable_fraction{0.8};

  // 회피 방향 관성.
  //
  // 수직 우회와 수평 우회의 비용이 거의 같아(둘 다 배리어 반폭 + 여유 =
  // 2 m) 격자 타이브레이커가 매 재계획마다 흔들린다. 실측에서 어뢰가
  // 60->28 m 구간은 수직, 20~14 m는 수평, 10 m는 다시 수직으로 축이
  // 바뀌었다. 기체는 그때마다 방향을 다시 잡아야 하므로 회피가 누적되지
  // 않고 상쇄된다.
  //
  // previous_bias에 직전 계획이 택한 회피 방향(단위벡터)을 넣으면, 그
  // 반대로 가는 이동에 벌점을 매겨 방향을 유지한다. 길이가 0이면 비활성.
  Point3D previous_bias{};
  // 벌점 크기(초). 한 스텝 비용이 dt이므로 dt의 몇 배인지로 생각하면 된다.
  // 0이면 관성 없음(기존 동작).
  double bias_penalty{0.5};
};

struct SpaceTimeResult
{
  std::vector<TimedPoint> path;
  bool success{false};
  // 시간창 안에 도달할 수 없는 목표였다(탐색을 아예 하지 않았다).
  bool goal_unreachable{false};
  std::size_t expanded{0};   // 확장한 노드 수(성능 측정·로그용)
  // 이번 계획이 택한 회피 방향(단위벡터). 다음 호출에 previous_bias로
  // 넣으면 방향이 유지된다. 회피가 없었으면 길이 0이다.
  Point3D bias{};
};

// 국소 목표까지 시공간 경로를 찾는다. 실패하면 success=false.
//
// local_goal은 전역 경로 위의 lookahead 점이어야 한다. 시간창 안에 도달
// 가능한 거리(max_speed * horizon_sec * reachable_fraction) 밖이면
// goal_unreachable=true로 즉시 돌아온다.
SpaceTimeResult runSpaceTimeAStar3D(
  const Point3D & start,
  const Point3D & local_goal,
  const std::vector<MovingBox> & obstacles,
  const SpaceTimeOptions & options);

// 시공간 경로에서 위치만 뽑는다(기존 nav_msgs/Path 발행 경로용).
std::vector<Point3D> stripTimes(const std::vector<TimedPoint> & path);

}  // namespace bluerov_integration::team_min
