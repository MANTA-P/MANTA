#include "bluerov_integration/team_min/velocity_command.hpp"

#include <algorithm>
#include <cmath>

namespace bluerov_integration::team_min
{

Point3D clampVelocity(const Point3D & velocity, const VelocityLimits & limits)
{
  Point3D limited = velocity;
  const double horizontal =
    std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
  if (horizontal > limits.max_horizontal && horizontal > 1.0e-9) {
    // 방향을 유지한 채 크기만 줄인다. 성분별로 자르면 진행방향이 틀어진다.
    const double scale = limits.max_horizontal / horizontal;
    limited.x *= scale;
    limited.y *= scale;
  }
  limited.z = std::clamp(limited.z, -limits.max_vertical, limits.max_vertical);
  return limited;
}

VelocityCommand velocityFromTimedPath(const std::vector<TimedPoint> & path)
{
  VelocityCommand command;
  if (path.size() < 2U) {
    return command;   // 구간이 없으면 속도를 정의할 수 없다
  }
  const double dt = path[1].time - path[0].time;
  if (dt <= 1.0e-9) {
    return command;
  }
  command.velocity = {
    (path[1].point.x - path[0].point.x) / dt,
    (path[1].point.y - path[0].point.y) / dt,
    (path[1].point.z - path[0].point.z) / dt};
  command.horizon_sec = dt;
  command.valid = true;
  return command;
}

VelocityCommand velocityFromDynamicVO(
  const Point3D & selected_velocity,
  const double rollout_step)
{
  VelocityCommand command;
  const double speed = std::sqrt(
    selected_velocity.x * selected_velocity.x +
    selected_velocity.y * selected_velocity.y +
    selected_velocity.z * selected_velocity.z);
  if (speed <= 1.0e-9 || rollout_step <= 0.0) {
    return command;
  }
  command.velocity = selected_velocity;
  command.horizon_sec = rollout_step;
  command.valid = true;
  return command;
}

VelocityCommand withTrackCorrection(
  VelocityCommand command,
  const Point3D & current_position,
  const Point3D & reference_position,
  const VelocityLimits & limits)
{
  if (!command.valid) {
    return command;
  }

  // PositionP를 우회하면서 잃어버리는 위치 피드백을 여기서 되살린다.
  Point3D correction{
    (reference_position.x - current_position.x) * limits.track_gain,
    (reference_position.y - current_position.y) * limits.track_gain,
    (reference_position.z - current_position.z) * limits.track_gain};

  // 보정이 회피 기동을 덮어쓰지 않도록 크기를 제한한다. 이탈이 커도
  // 플래너가 고른 방향이 우선이어야 한다.
  const double magnitude = std::sqrt(
    correction.x * correction.x + correction.y * correction.y +
    correction.z * correction.z);
  if (magnitude > limits.max_correction && magnitude > 1.0e-9) {
    const double scale = limits.max_correction / magnitude;
    correction.x *= scale;
    correction.y *= scale;
    correction.z *= scale;
  }

  command.velocity = clampVelocity(
    Point3D{
      command.velocity.x + correction.x,
      command.velocity.y + correction.y,
      command.velocity.z + correction.z},
    limits);
  return command;
}

}  // namespace bluerov_integration::team_min
