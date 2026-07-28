#include "bluerov_integration/team_byung/ppid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace bluerov_integration::team_byung
{
namespace
{

double clampSymmetric(const double value, const double limit)
{
  return limit > 0.0 ? std::clamp(value, -limit, limit) : value;
}

double axis(const Vector3 & vector, const std::size_t index)
{
  return index == 0 ? vector.x : (index == 1 ? vector.y : vector.z);
}

void setAxis(Vector3 & vector, const std::size_t index, const double value)
{
  if (index == 0) {
    vector.x = value;
  } else if (index == 1) {
    vector.y = value;
  } else {
    vector.z = value;
  }
}

}  // namespace

Vector3 FrameTransformer::worldToBody(
  const Vector3 & vector,
  const Quaternion & quaternion) const
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm <= 1e-12) {
    return vector;
  }

  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return {
    (1.0 - 2.0 * (y * y + z * z)) * vector.x +
      2.0 * (x * y + w * z) * vector.y +
      2.0 * (x * z - w * y) * vector.z,
    2.0 * (x * y - w * z) * vector.x +
      (1.0 - 2.0 * (x * x + z * z)) * vector.y +
      2.0 * (y * z + w * x) * vector.z,
    2.0 * (x * z + w * y) * vector.x +
      2.0 * (y * z - w * x) * vector.y +
      (1.0 - 2.0 * (x * x + y * y)) * vector.z};
}

PositionPController::PositionPController(
  Vector3 gain,
  const double max_horizontal_speed)
: gain_(gain),
  max_horizontal_speed_(max_horizontal_speed)
{
}

Vector3 PositionPController::update(
  const Vector3 & target,
  const Vector3 & current) const
{
  Vector3 velocity{
    gain_.x * (target.x - current.x),
    gain_.y * (target.y - current.y),
    gain_.z * (target.z - current.z)};
  const double horizontal_speed = std::hypot(velocity.x, velocity.y);
  if (max_horizontal_speed_ > 0.0 &&
    horizontal_speed > max_horizontal_speed_)
  {
    const double scale = max_horizontal_speed_ / horizontal_speed;
    velocity.x *= scale;
    velocity.y *= scale;
  }
  return velocity;
}

VelocityPIDController::VelocityPIDController(
  std::array<PIDGains, 3> gains,
  Vector3 output_limit,
  Vector3 feedforward_gain)
: gains_(std::move(gains)),
  output_limit_(output_limit),
  feedforward_gain_(feedforward_gain)
{
}

Vector3 VelocityPIDController::update(
  const Vector3 & target,
  const Vector3 & current,
  double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
    dt = 0.01;
  }
  const Vector3 error{
    target.x - current.x,
    target.y - current.y,
    target.z - current.z};
  Vector3 output;

  for (std::size_t index = 0; index < 3; ++index) {
    const PIDGains & gains = gains_[index];
    double integral = axis(integral_, index) + axis(error, index) * dt;
    integral = clampSymmetric(integral, gains.integral_limit);
    const double derivative = initialized_ ?
      (axis(error, index) - axis(previous_error_, index)) / dt : 0.0;
    const double target_axis = axis(target, index);
    const double feedforward =
      axis(feedforward_gain_, index) * target_axis * std::abs(target_axis);
    const double raw = feedforward +
      gains.kp * axis(error, index) +
      gains.ki * integral +
      gains.kd * derivative;
    const double limit = gains.output_limit > 0.0 ?
      gains.output_limit : axis(output_limit_, index);
    setAxis(integral_, index, integral);
    setAxis(output, index, clampSymmetric(raw, limit));
  }
  previous_error_ = error;
  initialized_ = true;
  return output;
}

void VelocityPIDController::reset()
{
  integral_ = {};
  previous_error_ = {};
  initialized_ = false;
}

YawHoldController::YawHoldController(
  PIDGains gains,
  const double command_limit)
: gains_(gains),
  command_limit_(command_limit)
{
}

double YawHoldController::normalizeAngle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}

void YawHoldController::setTargetYaw(const double yaw)
{
  target_yaw_ = normalizeAngle(yaw);
  integral_ = 0.0;
  has_target_ = true;
}

double YawHoldController::update(
  const double current_yaw,
  const double current_yaw_rate,
  double dt)
{
  if (!has_target_) {
    return 0.0;
  }
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
    dt = 0.01;
  }
  const double error = normalizeAngle(target_yaw_ - current_yaw);
  integral_ = clampSymmetric(
    integral_ + error * dt, gains_.integral_limit);
  const double command =
    gains_.kp * error + gains_.ki * integral_ - gains_.kd * current_yaw_rate;
  const double limit =
    gains_.output_limit > 0.0 ? gains_.output_limit : command_limit_;
  return clampSymmetric(command, limit);
}

void YawHoldController::reset()
{
  integral_ = 0.0;
  has_target_ = false;
}

ThrusterMixer::ThrusterMixer(ThrusterMixerConfig config)
: config_(std::move(config))
{
}

std::array<double, 6> ThrusterMixer::mix(
  const Vector3 & command,
  const double yaw_command) const
{
  std::array<double, 6> motors{};
  for (std::size_t index = 0; index < motors.size(); ++index) {
    motors[index] =
      config_.x_coefficients[index] * command.x +
      config_.y_coefficients[index] * command.y +
      config_.z_coefficients[index] * command.z +
      config_.yaw_coefficients[index] * yaw_command;
  }

  if (config_.command_limit > 0.0) {
    const auto desaturate = [this, &motors](
      const std::size_t begin,
      const std::size_t end)
      {
        double maximum = 0.0;
        for (std::size_t index = begin; index < end; ++index) {
          maximum = std::max(maximum, std::abs(motors[index]));
        }
        if (maximum > config_.command_limit) {
          const double scale = config_.command_limit / maximum;
          for (std::size_t index = begin; index < end; ++index) {
            motors[index] *= scale;
          }
        }
      };
    desaturate(0, 4);
    desaturate(4, 6);
  }
  return motors;
}

BlueRovPPIDController::BlueRovPPIDController(BlueRovControllerConfig config)
: config_(std::move(config)),
  position_controller_(config_.position_gain, config_.max_horizontal_speed),
  velocity_controller_(
    config_.velocity_gains,
    config_.velocity_output_limit,
    config_.velocity_feedforward),
  yaw_controller_(config_.yaw_gains, config_.yaw_command_limit),
  mixer_(config_.mixer)
{
}

void BlueRovPPIDController::setTargetPosition(
  const Vector3 & target_position_world)
{
  target_position_world_ = target_position_world;
}

double BlueRovPPIDController::quaternionToYaw(const Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

std::array<double, 6> BlueRovPPIDController::update(
  const OdometryState & odometry,
  const double dt)
{
  const double yaw = quaternionToYaw(odometry.orientation);
  if (!have_initial_yaw_) {
    yaw_controller_.setTargetYaw(yaw);
    have_initial_yaw_ = true;
  }

  Vector3 target_velocity_world = position_controller_.update(
    target_position_world_, odometry.position_world);
  if (config_.max_vertical_speed > 0.0) {
    target_velocity_world.z = std::clamp(
      target_velocity_world.z,
      -config_.max_vertical_speed,
      config_.max_vertical_speed);
  }
  const Vector3 target_velocity_body = frame_transformer_.worldToBody(
    target_velocity_world, odometry.orientation);
  Vector3 body_command = velocity_controller_.update(
    target_velocity_body, odometry.velocity_body, dt);
  body_command.z += config_.heave_trim;

  const double yaw_command =
    yaw_controller_.update(yaw, odometry.yaw_rate, dt);
  const auto motor_commands = mixer_.mix(body_command, yaw_command);

  telemetry_.control_dt_sec = dt;
  telemetry_.target_position_world = target_position_world_;
  telemetry_.current_position_world = odometry.position_world;
  telemetry_.target_velocity_body = target_velocity_body;
  telemetry_.current_velocity_body = odometry.velocity_body;
  telemetry_.body_command = body_command;
  telemetry_.yaw = yaw;
  telemetry_.yaw_rate = odometry.yaw_rate;
  telemetry_.yaw_command = yaw_command;
  telemetry_.motor_commands = motor_commands;
  return motor_commands;
}

const ControlTelemetry & BlueRovPPIDController::telemetry() const
{
  return telemetry_;
}

void BlueRovPPIDController::reset()
{
  velocity_controller_.reset();
  yaw_controller_.reset();
  have_initial_yaw_ = false;
}

}  // namespace bluerov_integration::team_byung
