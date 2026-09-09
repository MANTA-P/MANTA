#include "bluerov_integration/team_min/torpedo_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace bluerov_integration::team_min
{
namespace
{

Point3D add(const Point3D & a, const Point3D & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Point3D subtract(const Point3D & a, const Point3D & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Point3D scale(const Point3D & value, const double factor)
{
  return {value.x * factor, value.y * factor, value.z * factor};
}

double dot(const Point3D & a, const Point3D & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3D cross(const Point3D & a, const Point3D & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x};
}

double norm(const Point3D & value)
{
  return std::sqrt(dot(value, value));
}

Point3D normalized(const Point3D & value)
{
  const double magnitude = norm(value);
  return magnitude > 1.0e-9 ? scale(value, 1.0 / magnitude) : Point3D{};
}

Point3D clampMagnitude(const Point3D & value, const double maximum)
{
  const double magnitude = norm(value);
  if (maximum <= 0.0 || magnitude <= maximum || magnitude <= 1.0e-9) {
    return value;
  }
  return scale(value, maximum / magnitude);
}

Point3D turnToward(
  const Point3D & heading,
  const Point3D & desired,
  const double max_angle)
{
  const double cosine = std::clamp(dot(heading, desired), -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (angle <= max_angle || angle <= 1.0e-9) {
    return desired;
  }
  // 정규화 선형 보간은 작은 제어 주기에서 안정적이고 3D 특이축이 없다.
  return normalized(add(
      scale(heading, 1.0 - max_angle / angle),
      scale(desired, max_angle / angle)));
}

std::vector<Point3D> constantVelocityPath(
  const TorpedoGuidancePredictor::KinematicState & state,
  const PredictionConfig & config)
{
  std::vector<Point3D> path;
  const std::size_t steps = static_cast<std::size_t>(
    std::ceil(config.guidance_horizon_sec / config.guidance_step_sec));
  path.reserve(steps + 1U);
  for (std::size_t step = 0; step <= steps; ++step) {
    const double elapsed =
      std::min(config.guidance_horizon_sec,
        static_cast<double>(step) * config.guidance_step_sec);
    path.push_back(add(
        state.torpedo_position, scale(state.torpedo_velocity, elapsed)));
  }
  return path;
}

}  // namespace

TorpedoGuidancePredictor::TorpedoGuidancePredictor(PredictionConfig config)
: config_(std::move(config))
{
  if (config_.guidance_horizon_sec <= 0.0 ||
    config_.guidance_step_sec <= 0.0 ||
    config_.pn_navigation_constant <= 0.0 ||
    config_.pursuit_turn_rate <= 0.0 ||
    config_.guidance_max_lateral_acceleration <= 0.0 ||
    config_.classification_alpha <= 0.0 ||
    config_.classification_alpha > 1.0 ||
    config_.classification_ratio <= 0.0 ||
    config_.classification_ratio >= 1.0 ||
    config_.classification_min_samples == 0U)
  {
    throw std::invalid_argument("Torpedo guidance prediction parameters are invalid");
  }
}

double TorpedoGuidancePredictor::sampleTime(const VehicleState & sample)
{
  return sample.stamp_sec > 0.0 ? sample.stamp_sec : sample.received_sec;
}

TorpedoGuidancePredictor::KinematicState
TorpedoGuidancePredictor::propagate(
  KinematicState state,
  const TorpedoGuidanceLaw law,
  const double dt,
  const PredictionConfig & config)
{
  const double speed = norm(state.torpedo_velocity);
  if (speed <= 1.0e-6 || dt <= 0.0) {
    state.target_position =
      add(state.target_position, scale(state.target_velocity, dt));
    return state;
  }

  const Point3D relative_position =
    subtract(state.target_position, state.torpedo_position);
  const double range = norm(relative_position);
  if (range <= 1.0e-6) {
    return state;
  }
  const Point3D line_of_sight = scale(relative_position, 1.0 / range);
  const Point3D heading = scale(state.torpedo_velocity, 1.0 / speed);
  Point3D next_velocity = state.torpedo_velocity;

  if (law == TorpedoGuidanceLaw::kPurePursuit) {
    const Point3D next_heading = turnToward(
      heading, line_of_sight, config.pursuit_turn_rate * dt);
    next_velocity = scale(next_heading, speed);
  } else if (law == TorpedoGuidanceLaw::kProportionalNavigation) {
    const Point3D relative_velocity =
      subtract(state.target_velocity, state.torpedo_velocity);
    const double closing_speed = std::max(0.0, -dot(relative_velocity, line_of_sight));
    if (closing_speed <= 0.0) {
      // 실제 torpedo_control_v2 PNG도 멀어질 때 단순추적으로 폴백한다.
      next_velocity = scale(
        turnToward(heading, line_of_sight, config.pursuit_turn_rate * dt),
        speed);
    } else {
      const Point3D los_rate =
        scale(cross(relative_position, relative_velocity), 1.0 / (range * range));
      Point3D lateral_acceleration = scale(
        cross(los_rate, line_of_sight),
        config.pn_navigation_constant * closing_speed);
      lateral_acceleration = clampMagnitude(
        lateral_acceleration, config.guidance_max_lateral_acceleration);
      next_velocity = add(state.torpedo_velocity, scale(lateral_acceleration, dt));
      const double next_speed = norm(next_velocity);
      if (next_speed > 1.0e-9) {
        next_velocity = scale(next_velocity, speed / next_speed);
      }
    }
  }

  // 반암시적 적분: 이번 step에서 계산한 새 heading으로 위치를 전진한다.
  state.torpedo_velocity = next_velocity;
  state.torpedo_position =
    add(state.torpedo_position, scale(state.torpedo_velocity, dt));
  state.target_position =
    add(state.target_position, scale(state.target_velocity, dt));
  return state;
}

std::vector<Point3D> TorpedoGuidancePredictor::predictPath(
  KinematicState state,
  const TorpedoGuidanceLaw law,
  const PredictionConfig & config)
{
  std::vector<Point3D> path;
  const std::size_t steps = static_cast<std::size_t>(
    std::ceil(config.guidance_horizon_sec / config.guidance_step_sec));
  path.reserve(steps + 1U);
  path.push_back(state.torpedo_position);
  double elapsed = 0.0;
  for (std::size_t step = 0; step < steps; ++step) {
    const double dt = std::min(
      config.guidance_step_sec, config.guidance_horizon_sec - elapsed);
    state = propagate(state, law, dt, config);
    path.push_back(state.torpedo_position);
    elapsed += dt;
  }
  return path;
}

GuidancePrediction TorpedoGuidancePredictor::update(
  const VehicleState & torpedo,
  const Point3D & torpedo_velocity,
  const bool torpedo_velocity_valid,
  const VehicleState & target,
  const Point3D & target_velocity,
  const bool target_velocity_valid)
{
  GuidancePrediction result;
  if (!config_.guidance_enabled || !torpedo.valid || !target.valid ||
    !torpedo_velocity_valid)
  {
    return result;
  }

  const KinematicState current{
    torpedo.position, torpedo_velocity, target.position,
    target_velocity_valid ? target_velocity : Point3D{}};
  const double current_time = sampleTime(torpedo);
  if (have_previous_ && torpedo.sequence != previous_sequence_) {
    const double dt = current_time - previous_time_;
    if (dt > 1.0e-3 && dt < 2.0) {
      const auto pn = propagate(
        previous_, TorpedoGuidanceLaw::kProportionalNavigation, dt, config_);
      const auto pursuit = propagate(
        previous_, TorpedoGuidanceLaw::kPurePursuit, dt, config_);
      const double pn_sample = distance3D(pn.torpedo_position, torpedo.position);
      const double pursuit_sample =
        distance3D(pursuit.torpedo_position, torpedo.position);
      const double alpha = config_.classification_alpha;
      if (scored_samples_ == 0U) {
        pn_error_ = pn_sample;
        pursuit_error_ = pursuit_sample;
      } else {
        pn_error_ = alpha * pn_sample + (1.0 - alpha) * pn_error_;
        pursuit_error_ =
          alpha * pursuit_sample + (1.0 - alpha) * pursuit_error_;
      }
      ++scored_samples_;
    } else {
      scored_samples_ = 0U;
      pn_error_ = 0.0;
      pursuit_error_ = 0.0;
    }
  }

  if (scored_samples_ >= config_.classification_min_samples) {
    if (pn_error_ < pursuit_error_ * config_.classification_ratio) {
      law_ = TorpedoGuidanceLaw::kProportionalNavigation;
    } else if (pursuit_error_ < pn_error_ * config_.classification_ratio) {
      law_ = TorpedoGuidanceLaw::kPurePursuit;
    }
  }

  // 오차가 잠시 비슷해져도 확정 모델을 유지해 DVO 궤적이 흔들리지 않는다.
  result.law = law_;
  result.proportional_navigation_error = pn_error_;
  result.pure_pursuit_error = pursuit_error_;
  result.proportional_navigation_path = predictPath(
    current, TorpedoGuidanceLaw::kProportionalNavigation, config_);
  result.pure_pursuit_path = predictPath(
    current, TorpedoGuidanceLaw::kPurePursuit, config_);
  if (result.law == TorpedoGuidanceLaw::kProportionalNavigation) {
    result.selected_path = result.proportional_navigation_path;
  } else if (result.law == TorpedoGuidanceLaw::kPurePursuit) {
    result.selected_path = result.pure_pursuit_path;
  } else {
    result.selected_path = constantVelocityPath(current, config_);
  }

  if (!have_previous_ || torpedo.sequence != previous_sequence_) {
    previous_ = current;
    previous_sequence_ = torpedo.sequence;
    previous_time_ = current_time;
    have_previous_ = true;
  }
  return result;
}

void TorpedoGuidancePredictor::reset()
{
  have_previous_ = false;
  previous_sequence_ = 0U;
  previous_time_ = 0.0;
  previous_ = {};
  scored_samples_ = 0U;
  pn_error_ = 0.0;
  pursuit_error_ = 0.0;
  law_ = TorpedoGuidanceLaw::kUnknown;
}

}  // namespace bluerov_integration::team_min
