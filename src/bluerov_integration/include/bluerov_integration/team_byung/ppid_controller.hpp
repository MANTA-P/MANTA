#pragma once

#include <array>
#include <cstddef>

namespace bluerov_integration::team_byung
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct PIDGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_limit{0.0};
  double output_limit{0.0};
};

struct ThrusterMixerConfig
{
  std::array<double, 6> x_coefficients{
    -0.25, -0.25, 0.25, 0.25, 0.0, 0.0};
  std::array<double, 6> y_coefficients{
    -0.25, 0.25, -0.25, 0.25, 0.0, 0.0};
  std::array<double, 6> z_coefficients{
    0.0, 0.0, 0.0, 0.0, -0.5, -0.5};
  std::array<double, 6> yaw_coefficients{
    -0.25, 0.25, 0.25, -0.25, 0.0, 0.0};
  double command_limit{120.0};
};

struct BlueRovControllerConfig
{
  Vector3 position_gain{1.5, 1.5, 1.5};
  double max_horizontal_speed{3.0};
  double max_vertical_speed{1.5};
  std::array<PIDGains, 3> velocity_gains{
    PIDGains{60.0, 0.5, 1.0, 2.0, 450.0},
    PIDGains{60.0, 0.5, 1.0, 2.0, 450.0},
    PIDGains{70.0, 0.5, 1.0, 2.0, 100.0}};
  Vector3 velocity_output_limit{100.0, 100.0, 100.0};
  Vector3 velocity_feedforward{47.704251916, 76.593806538, 0.0};
  PIDGains yaw_gains{80.0, 2.0, 10.0, 5.0, 100.0};
  double yaw_command_limit{100.0};
  double heave_trim{11.0};
  ThrusterMixerConfig mixer{};
};

class FrameTransformer
{
public:
  Vector3 worldToBody(
    const Vector3 & world_vector,
    const Quaternion & body_orientation_world) const;
};

class PositionPController
{
public:
  PositionPController(Vector3 gain, double max_horizontal_speed);

  Vector3 update(
    const Vector3 & target_position,
    const Vector3 & current_position) const;

private:
  Vector3 gain_;
  double max_horizontal_speed_;
};

class VelocityPIDController
{
public:
  VelocityPIDController(
    std::array<PIDGains, 3> gains,
    Vector3 output_limit,
    Vector3 feedforward_gain);

  Vector3 update(
    const Vector3 & target_velocity,
    const Vector3 & current_velocity,
    double dt);
  void reset();

private:
  std::array<PIDGains, 3> gains_;
  Vector3 output_limit_;
  Vector3 feedforward_gain_;
  Vector3 integral_{};
  Vector3 previous_error_{};
  bool initialized_{false};
};

class YawHoldController
{
public:
  YawHoldController(PIDGains gains, double command_limit);

  void setTargetYaw(double yaw);
  double update(double current_yaw, double current_yaw_rate, double dt);
  void reset();

private:
  static double normalizeAngle(double angle);

  PIDGains gains_;
  double command_limit_;
  double target_yaw_{0.0};
  double integral_{0.0};
  bool has_target_{false};
};

class ThrusterMixer
{
public:
  explicit ThrusterMixer(ThrusterMixerConfig config);

  std::array<double, 6> mix(
    const Vector3 & body_command,
    double yaw_command) const;

private:
  ThrusterMixerConfig config_;
};

struct OdometryState
{
  Vector3 position_world{};
  Vector3 velocity_body{};
  Quaternion orientation{};
  double yaw_rate{0.0};
};

struct ControlTelemetry
{
  double control_dt_sec{0.0};
  Vector3 target_position_world{};
  Vector3 current_position_world{};
  Vector3 target_velocity_body{};
  Vector3 current_velocity_body{};
  Vector3 body_command{};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_command{0.0};
  std::array<double, 6> motor_commands{};
};

class BlueRovPPIDController
{
public:
  explicit BlueRovPPIDController(BlueRovControllerConfig config);

  void setTargetPosition(const Vector3 & target_position_world);
  std::array<double, 6> update(const OdometryState & odometry, double dt);
  const ControlTelemetry & telemetry() const;
  void reset();

private:
  static double quaternionToYaw(const Quaternion & quaternion);

  BlueRovControllerConfig config_;
  PositionPController position_controller_;
  VelocityPIDController velocity_controller_;
  YawHoldController yaw_controller_;
  FrameTransformer frame_transformer_;
  ThrusterMixer mixer_;
  Vector3 target_position_world_{};
  bool have_initial_yaw_{false};
  ControlTelemetry telemetry_{};
};

}  // namespace bluerov_integration::team_byung
