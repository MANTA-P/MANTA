#pragma once

// 속도 PID 제어기 (단일 루프).
//
// 왜 PPID가 아니라 PID인가:
//   기존 team_byung 제어기는 P(위치) -> PID(속도) 캐스케이드다. 플래너가
//   위치점을 주므로 앞단 P가 그걸 속도로 바꿔야 했다.
//   시공간 A*와 DVO가 속도를 직접 내면 그 변환이 필요 없어진다. 위치
//   피드백은 velocity_command.hpp의 withTrackCorrection()이 흡수하므로,
//   제어기는 속도 루프 하나만 있으면 된다.
//
//   루프가 하나 줄면 튜닝할 게이트가 줄고 지연도 준다. ESP32 이식에도
//   유리하다 — 이 파일에는 ROS도 STL 컨테이너도 동적 할당도 없다.
//
// 좌표계:
//   입력 target/current는 같은 프레임이어야 한다. body 프레임을 쓰면
//   출력도 body 프레임 힘이 된다(추력기 믹서가 기대하는 형태).

#include <array>

#include "bluerov_integration/team_min/astar_planner.hpp"

namespace bluerov_integration::team_min
{

// team_byung PPID의 속도 루프와 필드 의미를 동일하게 맞춘다.
// integral_limit은 ki를 곱하기 "전"의 누적값(∫e dt)에 걸린다.
struct VelocityPidGains
{
  double kp{60.0};
  double ki{0.5};
  double kd{1.0};
  double integral_limit{2.0};    // ∫e dt 의 상한
  double output_limit{450.0};    // 축 출력 상한
  // 아래 둘은 원본에 없는 선택적 개선이다. 기본값은 원본과 같은 동작이라
  // (필터 없음 / 상한만으로 windup 억제) 비교 실험의 기준선이 흔들리지 않는다.
  double derivative_alpha{1.0};  // 1.0 = 미분 필터 없음(원본과 동일)
  bool conditional_integration{false};  // false = 원본과 동일
};

struct VelocityPidConfig
{
  // 축별 게인. team_byung PPID의 속도 루프 값을 기본값으로 가져왔다 —
  // 같은 기체·같은 추력기라 출발점이 같아야 비교가 된다.
  std::array<VelocityPidGains, 3> gains{
    VelocityPidGains{60.0, 0.5, 1.0, 2.0, 450.0, 1.0, false},
    VelocityPidGains{60.0, 0.5, 1.0, 2.0, 450.0, 1.0, false},
    VelocityPidGains{70.0, 0.5, 1.0, 2.0, 100.0, 1.0, false}};
  // 속도 피드포워드는 2차다: gain x target x |target|.
  // 항력이 v^2에 비례하므로 정속을 유지하는 추력도 v^2에 비례한다.
  // 선형으로 두면 저속에서 과하고 고속에서 모자란다.
  Point3D feedforward{47.704251916, 76.593806538, 0.0};
  // 부력 상쇄 등 z축 상시 보정.
  double heave_trim{11.0};
  // dt가 이 값을 넘으면 적분·미분을 건너뛴다(틱 누락 대응).
  double max_dt{0.5};
};

struct VelocityPidTelemetry
{
  Point3D target{};
  Point3D current{};
  Point3D error{};
  Point3D proportional{};
  Point3D integral{};
  Point3D derivative{};
  Point3D output{};
  bool saturated{false};
};

// 3축 속도 PID. 상태를 들고 있으므로 인스턴스를 재사용해야 한다.
class VelocityPid3D
{
public:
  explicit VelocityPid3D(VelocityPidConfig config);

  // target_velocity와 current_velocity는 같은 프레임이어야 한다.
  // 반환값은 추력 명령(축별로 output_limit로 잘린 값)이다.
  Point3D update(
    const Point3D & target_velocity,
    const Point3D & current_velocity,
    double dt);

  // 플래너가 멈췄거나 모드가 바뀌었을 때 적분·미분 상태를 지운다.
  void reset();

  const VelocityPidTelemetry & telemetry() const {return telemetry_;}

private:
  // 축 하나의 PID. 배열 인덱스로 x=0, y=1, z=2.
  double updateAxis(
    std::size_t axis, double target, double current, double dt);

  VelocityPidConfig config_;
  std::array<double, 3> integral_{};
  std::array<double, 3> previous_error_{};
  std::array<double, 3> filtered_derivative_{};
  bool initialized_{false};
  VelocityPidTelemetry telemetry_{};
};

}  // namespace bluerov_integration::team_min
