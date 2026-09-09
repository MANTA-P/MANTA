#include "bluerov_integration/team_min/velocity_pid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bluerov_integration::team_min
{
namespace
{

double axisOf(const Point3D & point, const std::size_t axis)
{
  return axis == 0U ? point.x : (axis == 1U ? point.y : point.z);
}

void setAxis(Point3D & point, const std::size_t axis, const double value)
{
  if (axis == 0U) {
    point.x = value;
  } else if (axis == 1U) {
    point.y = value;
  } else {
    point.z = value;
  }
}

}  // namespace

VelocityPid3D::VelocityPid3D(VelocityPidConfig config)
: config_(config)
{
  for (const auto & gains : config_.gains) {
    if (gains.derivative_alpha <= 0.0 || gains.derivative_alpha > 1.0 ||
      gains.integral_limit < 0.0 || gains.output_limit <= 0.0)
    {
      throw std::invalid_argument("Velocity PID gains are invalid");
    }
  }
  if (config_.max_dt <= 0.0) {
    throw std::invalid_argument("Velocity PID max_dt must be positive");
  }
}

void VelocityPid3D::reset()
{
  integral_.fill(0.0);
  previous_error_.fill(0.0);
  filtered_derivative_.fill(0.0);
  initialized_ = false;
}

double VelocityPid3D::updateAxis(
  const std::size_t axis,
  const double target,
  const double current,
  const double dt)
{
  const VelocityPidGains & gains = config_.gains[axis];
  const double error = target - current;
  const double limit = gains.output_limit;

  const double proportional = gains.kp * error;

  // 적분은 ki를 곱하기 전의 누적값을 들고 있다(team_byung과 동일).
  double integral = integral_[axis];
  bool accumulate = true;
  if (gains.conditional_integration) {
    // 선택적 개선: 이미 포화됐고 오차가 같은 방향이면 더 쌓지 않는다.
    const double unsaturated = proportional + gains.ki * integral;
    accumulate = !((unsaturated >= limit && error > 0.0) ||
      (unsaturated <= -limit && error < 0.0));
  }
  if (accumulate) {
    integral += error * dt;
    integral = std::clamp(integral, -gains.integral_limit,
      gains.integral_limit);
  }
  integral_[axis] = integral;

  // 미분: 첫 틱에는 이전 오차가 없어 건너뛴다(시작 시 킥 방지).
  double derivative = 0.0;
  if (initialized_ && dt > 1.0e-6) {
    const double raw = (error - previous_error_[axis]) / dt;
    filtered_derivative_[axis] =
      gains.derivative_alpha * raw +
      (1.0 - gains.derivative_alpha) * filtered_derivative_[axis];
    derivative = gains.kd * filtered_derivative_[axis];
  }
  previous_error_[axis] = error;

  // 2차 피드포워드 — 항력이 v^2이므로 정속 유지 추력도 v^2에 비례한다.
  const double feedforward =
    axisOf(config_.feedforward, axis) * target * std::abs(target);
  const double integral_term = gains.ki * integral;
  const double output =
    proportional + integral_term + derivative + feedforward;

  setAxis(telemetry_.proportional, axis, proportional);
  setAxis(telemetry_.integral, axis, integral_term);
  setAxis(telemetry_.derivative, axis, derivative);
  setAxis(telemetry_.error, axis, error);

  return std::clamp(output, -limit, limit);
}

Point3D VelocityPid3D::update(
  const Point3D & target_velocity,
  const Point3D & current_velocity,
  const double dt)
{
  telemetry_.target = target_velocity;
  telemetry_.current = current_velocity;

  // 틱이 길게 비면(정지·재시작) 적분과 미분이 튄다. 그때는 비례항만 쓴다.
  const double step = (dt > 0.0 && dt <= config_.max_dt) ? dt : 0.0;

  Point3D output;
  setAxis(output, 0U, updateAxis(0U, target_velocity.x,
    current_velocity.x, step));
  setAxis(output, 1U, updateAxis(1U, target_velocity.y,
    current_velocity.y, step));
  setAxis(output, 2U, updateAxis(2U, target_velocity.z,
    current_velocity.z, step));
  output.z += config_.heave_trim;

  telemetry_.output = output;
  telemetry_.saturated =
    std::abs(output.x) >= config_.gains[0].output_limit ||
    std::abs(output.y) >= config_.gains[1].output_limit ||
    std::abs(output.z - config_.heave_trim) >= config_.gains[2].output_limit;

  if (step > 0.0) {
    initialized_ = true;
  }
  return output;
}

}  // namespace bluerov_integration::team_min
