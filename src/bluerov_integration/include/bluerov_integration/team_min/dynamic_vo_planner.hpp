#pragma once

#include <cstddef>
#include <vector>

#include "bluerov_integration/team_min/astar_planner.hpp"

namespace bluerov_integration::team_min
{

// Dynamic VO: planner inputs use the same world frame as Point3D.
struct MovingObstacle
{
  BoxObstacle shape;
  Point3D velocity;
};

// 속도 후보를 고르는 비용 가중치.
//
// 지금까지는 .cpp 안에 상수로 박혀 있었고, 목표 항이 회피 항의 3배라
// DVO가 "목표를 향하되 겨우 안 부딪히게"를 골랐다. 실측에서 A*와 마찬가지로
// 최소 이격만 확보하다 피격됐다(최근접 0.55~0.99 m, 피격반경 1.0 m).
//
// 교전이 임박하면(TTC < break_sec) 가중치를 바꿔 "목표를 잊고 최대한
// 벌어져라"로 전환한다. 비례항법에 대한 회피는 늦게·크게 해야 효과가 있다.
struct DynamicVOWeights
{
  double goal{3.0};        // 국소 목표에 가까울수록 좋다
  double preferred{0.35};  // 목표 지향 속도에서 덜 벗어날수록 좋다
  double continuity{0.175};  // 이전 속도에서 덜 변할수록 좋다(급변 억제)
  double clearance{1.0};   // 장애물 이격이 클수록 좋다
  double beam{0.0};        // 시선에 수직으로 갈수록 좋다(급기동 전용)
};

struct DynamicVOOptions
{
  double prediction_horizon{6.0};
  double rollout_step{0.5};
  std::size_t rollout_steps{20U};
  // 어뢰를 포화시킬 수 있는 거리는 이 속도에 비례한다:
  //   r_sat = N x Vt x v_max / a_max  (실측 a_max = 6.0 m/s^2, N = 3)
  // 3.0 m/s 면 mk48 기준 13.0 m 안까지 붙어야 하고 백상어는 8.0 m 다.
  // 피격반경 1.0 m 를 생각하면 너무 얇아 4.0 m/s 로 올린다(mk48 17.3 m).
  //
  // 4.0 으로 올려 봤으나 소용없었다. 궤적 실측:
  //   상한 3.0 -> 중앙값 2.83, 상위1% 3.13, 최대 3.55 m/s
  //   상한 4.0 -> 중앙값 2.72, 상위1% 3.11, 최대 3.17 m/s
  // 추력이 모자라 실제 속도가 3.1 m/s 에서 막힌다. 상한만 올리면 DVO 가
  // 낼 수 없는 속도로 계획해 예측이 실제보다 앞서 나가므로 되돌린다.
  double max_horizontal_speed{3.0};
  // 수평(3.0)의 절반으로 묶여 있었는데, 이는 차량 한계가 아니라 우리가
  // 정한 값이다. 궤적 실측 ROV 수직 최고속도가 1.52 m/s 로 설정에 정확히
  // 붙어 있었다(수평도 3.06 으로 마찬가지). BlueROV2 는 수직 추진기가
  // 따로 있어 수직이 수평보다 약할 이유가 없다.
  //
  // 3.0 으로 올려 12판 돌려 봤으나 1/12 로 1.5 일 때와 같았고, 최근접은
  // 0.84 -> 0.76 m 로 오히려 나빠졌다(최소 0.25 m). 어뢰가 수직으로 더
  // 민첩하기 때문으로 보인다(실측 99%: 수직 7.93 vs 수평 7.27 m/s^2).
  // 되돌린다.
  double max_vertical_speed{1.5};
  double robot_radius{0.75};
  double safety_margin{0.50};
  double goal_tolerance{0.75};
  // DVO 가 향하는 A* 경로 위 앞점까지의 거리. 이 값이 곧 "A* 경로를
  // 얼마나 따르는가"다. 비용함수에서 목표 항 가중치가 3.0 으로 가장 큰데,
  // 앞점이 가까우면 어뢰를 피해 옆으로 한 발만 빠져도 목표까지 거리가
  // 급등해 곧바로 경로로 끌려 돌아온다(실측: hybrid 8% < dvo 단독 29%).
  // 멀리 두면 같은 회피에도 목표 비용 증가가 작아 깊게 피할 수 있다.
  //
  // 상한은 롤아웃 도달 거리다: 20 스텝 x 0.5 s x 3 m/s = 30 m. 이를 넘으면
  // 절대 도달하지 못한다. 그 2/3 인 20 m 로 둔다.
  //
  // 대가: 멀수록 A* 가 우회한 이유(정적 장애물)를 무시하고 질러갈 수 있다.
  // 지형이 있는 환경에서는 줄여야 하고, 그만큼 회피 성능이 떨어진다.
  double path_lookahead{20.0};
  int heading_samples{36};
  int vertical_samples{7};
  int speed_samples{5};
  // true면 A*에 기대지 않고 단독으로 계획한다. 충돌 위험이 없어도 목표를
  // 향해 롤아웃하고, 목표에 못 닿아도 부분 경로를 성공으로 인정한다
  // (매 틱 다시 뽑는 receding-horizon). false면 기존 하이브리드 동작.
  bool standalone{false};

  // 평상시 가중치. 기존 .cpp 상수와 같은 값이라 동작이 바뀌지 않는다
  // (kGoalWeight 3.0, kVelocityWeight 0.35 x {1.0, 0.5}, kClearanceWeight 1.0).
  DynamicVOWeights weights{3.0, 0.35, 0.175, 1.0};

  // 급기동 가중치.
  //
  // 예전에는 clearance 를 10 으로 올려 "최대한 벌어져라"로 갔는데,
  // clearance 는 예측 이격거리라서 정면에서 오는 어뢰에 대해 이걸
  // 최대화하면 ROV 가 시선 방향으로 도망친다. 그런데 시선 방향 도주는
  // 시선각속도가 0 이라 비례항법이 횡가속을 쓸 필요조차 없는, 어뢰에게
  // 가장 쉬운 표적이다. 실측 순변위 -0.82 m 와 hybrid 8%(<dvo 29%)가
  // 이 때문으로 보인다.
  //
  // 그래서 최대화 대상을 거리에서 시선각속도로 바꾼다(beam 항).
  // 비례항법의 소요 횡가속은 a = N x Vc x lambda_dot 이므로,
  // 시선각속도를 키우면 어뢰의 횡가속 한계를 넘길 수 있다. 실측
  // 어뢰 구심가속도 한계 6.0 m/s^2, ROV 횡속도 상한 3.0 m/s, N=3 이면
  // 포화 시점은 t_go = N x v_e / a_max = 1.5 초로 어뢰 속도와 무관하다.
  //
  // clearance 는 2.0 으로 낮춰 남긴다. 통로 박스로 뛰어드는 것은
  // 여전히 막아야 하지만, beam 항을 덮어서는 안 된다.
  DynamicVOWeights break_weights{0.1, 0.0, 0.3, 2.0, 8.0};

  // true 면 break_weights 를 쓴다. 어댑터가 교전 국면을 보고 정한다.
  bool break_mode{false};

  // true 면 횡기동 방향을 직전 속도의 반대로 잡는다(jink).
  // break_mode 안에서만 의미가 있다.
  bool break_reverse{false};
};

struct DynamicVOResult
{
  bool avoidance_required{false};
  bool success{false};
  Point3D selected_velocity;
  std::vector<Point3D> local_path;
};

// Dynamic VO: returns a short local path toward an A* lookahead point.
DynamicVOResult runDynamicVO3D(
  const Point3D & robot_position,
  const Point3D & robot_velocity,
  const Point3D & local_goal,
  const GridMapConfig & map_config,
  const std::vector<MovingObstacle> & obstacles,
  const DynamicVOOptions & options = DynamicVOOptions{});

}  // namespace bluerov_integration::team_min
