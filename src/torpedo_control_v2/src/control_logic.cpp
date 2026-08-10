#include "torpedo_control_v2/control_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace torpedo_control_v2
{

ControlLogic::ControlLogic(ControllerConfig config)
: config_(std::move(config)),
  thrust_level_(std::clamp(config_.thrust_initial, config_.thrust_min, config_.thrust_max))
{
}

ControlResult ControlLogic::initial_result() const
{
  ControlResult result;
  result.command = make_command();
  result.thrust_level = thrust_level_;
  result.auto_track = auto_track_mode_;
  result.events = "START";
  return result;
}

ControlResult ControlLogic::step(
  const StateSnapshot & state, const std::vector<KeyEvent> & events)
{
  std::string event_text;
  for (const auto & event : events) {
    handle_key(event.key, event_text);
  }
  return result_from_state(state, std::move(event_text));
}

ControlResult ControlLogic::emergency_stop()
{
  thrust_level_ = std::clamp(0.0, config_.thrust_min, config_.thrust_max);
  pitch_target_rad_ = 0.0;
  yaw_target_rad_ = 0.0;
  roll_correction_rad_ = 0.0;
  auto_track_mode_ = false;

  ControlResult result;
  result.command = make_command();
  result.thrust_level = thrust_level_;
  result.auto_track = false;
  result.events = "SHUTDOWN";
  return result;
}

ControlLogic::Vector3 ControlLogic::rotate_inverse(
  const Vector3 & vector, const double qx, const double qy, const double qz, const double qw)
{
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    return vector;
  }

  const double x = qx / norm;
  const double y = qy / norm;
  const double z = qz / norm;
  const double w = qw / norm;

  // q_inverse = (w, -x, -y, -z), applied as q_inverse * v * q.
  const Vector3 qv{-x, -y, -z};
  const Vector3 t{
    2.0 * (qv.y * vector.z - qv.z * vector.y),
    2.0 * (qv.z * vector.x - qv.x * vector.z),
    2.0 * (qv.x * vector.y - qv.y * vector.x)};
  return {
    vector.x + w * t.x + (qv.y * t.z - qv.z * t.y),
    vector.y + w * t.y + (qv.z * t.x - qv.x * t.z),
    vector.z + w * t.z + (qv.x * t.y - qv.y * t.x)};
}

double ControlLogic::roll_angle_from_orientation(const OdometryState & odometry)
{
  const Vector3 world_up_in_body = rotate_inverse(
    {0.0, 0.0, 1.0},
    odometry.orientation_x,
    odometry.orientation_y,
    odometry.orientation_z,
    odometry.orientation_w);

  // Roll is rotation about the model's +Y forward axis. The gravity/up
  // vector gives a stable zero-roll reference even when yaw is changing.
  return std::atan2(-world_up_in_body.x, world_up_in_body.z);
}

std::string ControlLogic::append_event(std::string current, const std::string & event)
{
  if (!current.empty()) {
    current += "+";
  }
  current += event;
  return current;
}

double ControlLogic::limited_fin_command(const double value) const
{
  return std::clamp(value, -config_.fin_limit_rad, config_.fin_limit_rad);
}

double ControlLogic::clamp_fin_output(const double value) const
{
  return std::clamp(value, config_.fin_min_rad, config_.fin_max_rad);
}

double ControlLogic::limited_roll_command(const double value) const
{
  return std::clamp(value, -config_.roll_limit_rad, config_.roll_limit_rad);
}

void ControlLogic::handle_key(const char key, std::string & events)
{
  if (key == config_.auto_track_key) {
    auto_track_mode_ = !auto_track_mode_;
    events = append_event(std::move(events), auto_track_mode_ ? "AUTO_ON" : "AUTO_OFF");
    return;
  }

  // Manual fin input is disabled while automatic tracking is active. Thrust
  // remains available in both modes.
  if (key == config_.nose_push_key && !auto_track_mode_) {
    pitch_target_rad_ = limited_fin_command(pitch_target_rad_ + config_.fin_command_rad);
    events = append_event(std::move(events), "W");
  } else if (key == config_.nose_pull_key && !auto_track_mode_) {
    pitch_target_rad_ = limited_fin_command(pitch_target_rad_ - config_.fin_command_rad);
    events = append_event(std::move(events), "S");
  } else if (key == config_.yaw_left_key && !auto_track_mode_) {
    yaw_target_rad_ = limited_fin_command(yaw_target_rad_ + config_.fin_command_rad);
    events = append_event(std::move(events), "A");
  } else if (key == config_.yaw_right_key && !auto_track_mode_) {
    yaw_target_rad_ = limited_fin_command(yaw_target_rad_ - config_.fin_command_rad);
    events = append_event(std::move(events), "D");
  } else if (key == config_.thrust_increase_key) {
    thrust_level_ = std::clamp(
      thrust_level_ + config_.thrust_step, config_.thrust_min, config_.thrust_max);
    events = append_event(std::move(events), "R");
  } else if (key == config_.thrust_decrease_key) {
    thrust_level_ = std::clamp(
      thrust_level_ - config_.thrust_step, config_.thrust_min, config_.thrust_max);
    events = append_event(std::move(events), "F");
  } else if (key == config_.thrust_stop_key) {
    thrust_level_ = std::clamp(0.0, config_.thrust_min, config_.thrust_max);
    events = append_event(std::move(events), "SPACE");
  }
}

ActuatorCommand ControlLogic::make_command() const
{
  const double pitch_command = pitch_target_rad_ * config_.pitch_input_sign;
  const double yaw_command = yaw_target_rad_ * config_.yaw_input_sign;
  const double requested_roll_command = roll_correction_rad_ * config_.roll_input_sign;

  // First preserve the common Yaw command within the physical joint range.
  // Roll is then added differentially only as far as both Top and Bottom can
  // accept it. This prevents one fin from saturating and silently changing
  // the requested Yaw/Roll mix.
  const double top_yaw_base = clamp_fin_output(
    config_.fin_top_neutral_rad + config_.yaw_top_sign * yaw_command);
  const double bottom_yaw_base = clamp_fin_output(
    config_.fin_bottom_neutral_rad + config_.yaw_bottom_sign * yaw_command);
  const double top_roll_delta = config_.roll_top_sign * requested_roll_command;
  const double bottom_roll_delta = config_.roll_bottom_sign * requested_roll_command;
  double roll_scale = 1.0;

  const auto constrain_scale = [this, &roll_scale](
    const double base, const double delta) {
      if (delta > 0.0) {
        roll_scale = std::min(roll_scale, (config_.fin_max_rad - base) / delta);
      } else if (delta < 0.0) {
        roll_scale = std::min(roll_scale, (config_.fin_min_rad - base) / delta);
      }
    };
  constrain_scale(top_yaw_base, top_roll_delta);
  constrain_scale(bottom_yaw_base, bottom_roll_delta);
  roll_scale = std::clamp(roll_scale, 0.0, 1.0);

  ActuatorCommand command;
  command.thrust = config_.thrust_output_sign * thrust_level_;
  command.pitch_target_rad = pitch_target_rad_;
  command.yaw_target_rad = yaw_target_rad_;
  command.fin_top = clamp_fin_output(top_yaw_base + roll_scale * top_roll_delta);
  command.fin_bottom = clamp_fin_output(bottom_yaw_base + roll_scale * bottom_roll_delta);
  command.fin_left = clamp_fin_output(
    config_.fin_left_neutral_rad + config_.pitch_left_sign * pitch_command);
  command.fin_right = clamp_fin_output(
    config_.fin_right_neutral_rad + config_.pitch_right_sign * pitch_command);
  command.roll_correction_applied_rad = roll_correction_rad_ * roll_scale;
  command.roll_mix_scale = roll_scale;
  return command;
}

ControlResult ControlLogic::result_from_state(
  const StateSnapshot & state, std::string events)
{
  double pitch_error = kNaN;
  double yaw_error = kNaN;
  double roll_angle = kNaN;
  double roll_rate = kNaN;
  roll_correction_rad_ = 0.0;

  if (state.odometry.valid) {
    roll_angle = roll_angle_from_orientation(state.odometry);
    roll_rate = state.odometry.angular_y;
    if (config_.roll_control_enabled && std::isfinite(roll_angle) &&
      std::isfinite(roll_rate))
    {
      const double roll_error = -roll_angle;  // desired roll is zero
      const double raw_correction =
        config_.roll_kp * roll_error - config_.roll_kd * roll_rate;
      roll_correction_rad_ = limited_roll_command(raw_correction);
    }
  }

  if (state.odometry.valid && state.target.valid) {
    const Vector3 world_vector{
      state.target.x - state.odometry.position_x,
      state.target.y - state.odometry.position_y,
      state.target.z - state.odometry.position_z};
    const Vector3 body_vector = rotate_inverse(
      world_vector,
      state.odometry.orientation_x,
      state.odometry.orientation_y,
      state.odometry.orientation_z,
      state.odometry.orientation_w);

    // Body frame convention: +Y forward, +X right, +Z up.
    yaw_error = std::atan2(-body_vector.x, body_vector.y);
    pitch_error = std::atan2(body_vector.z, body_vector.y);

    if (auto_track_mode_) {
      pitch_target_rad_ = limited_fin_command(pitch_error * config_.p_gain_pitch);
      // The retained SITL logs show fin command + -> body wz -. The
      // yaw_input_sign in YAML compensates that actuator direction, so the
      // feedback target keeps the same sign as the geometric yaw error.
      yaw_target_rad_ = limited_fin_command(yaw_error * config_.p_gain_yaw);
    }
  }

  ControlResult result;
  result.command = make_command();
  result.thrust_level = thrust_level_;
  result.pitch_error_rad = pitch_error;
  result.yaw_error_rad = yaw_error;
  result.roll_angle_rad = roll_angle;
  result.roll_rate_rad_per_sec = roll_rate;
  result.roll_correction_rad = roll_correction_rad_;
  result.auto_track = auto_track_mode_;
  result.events = events.empty() ? "NONE" : std::move(events);
  return result;
}

}  // namespace torpedo_control_v2
