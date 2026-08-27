#include "torpedo_control_v2/png_controller.hpp"
#include <cmath>

namespace torpedo_control_v2
{

PngController::PngController(const ControlConfig & config) : config_(config) {}

void PngController::reset()
{
    thrust_ = 0.0;
    closing_speed_ = 0.0;
}

const char * PngController::guidance_mode_name() const
{
    return closing_speed_ > 0.0 ? "PNG" : "PURSUIT";
}

double PngController::closing_speed() const
{
    return closing_speed_;
}

ControlDemand PngController::update(const SensorData & sensor_data, InputCommand command)
{
    update_thrust(command);
    if (thrust_ == 0.0) {
        return ControlDemand{};
    }
    if (!sensor_data.torpedo_odometry.valid || !sensor_data.target_odometry.valid) {
        return ControlDemand{};
    }

    closing_speed_ = 0.0;
    const util::Vector3 acceleration_world = calculate_png_acceleration_world(sensor_data, closing_speed_);

    ControlDemand demand;
    demand.thrust = thrust_;
    if (closing_speed_ <= 0.0) {
        const util::Vector3 world_vector{
            sensor_data.target_odometry.position_x - sensor_data.torpedo_odometry.position_x,
            sensor_data.target_odometry.position_y - sensor_data.torpedo_odometry.position_y,
            sensor_data.target_odometry.position_z - sensor_data.torpedo_odometry.position_z};
        const util::Vector3 body_vector = util::rotate_inverse(
            world_vector,
            sensor_data.torpedo_odometry.orientation_x,
            sensor_data.torpedo_odometry.orientation_y,
            sensor_data.torpedo_odometry.orientation_z,
            sensor_data.torpedo_odometry.orientation_w);
        demand.pitch_rad = limit_fin_command(std::atan2(body_vector.z, body_vector.y) * config_.simple_tracking_pitch_kp);
        demand.yaw_rad = limit_fin_command(std::atan2(-body_vector.x, body_vector.y) * config_.simple_tracking_yaw_kp);
        return demand;
    }

    const util::Vector3 acceleration_body = util::rotate_inverse(
        acceleration_world,
        sensor_data.torpedo_odometry.orientation_x,
        sensor_data.torpedo_odometry.orientation_y,
        sensor_data.torpedo_odometry.orientation_z,
        sensor_data.torpedo_odometry.orientation_w);
    demand.pitch_rad = limit_fin_command(acceleration_body.z * config_.png_acceleration_to_fin_gain);
    demand.yaw_rad = limit_fin_command(-acceleration_body.x * config_.png_acceleration_to_fin_gain);
    return demand;
}

util::Vector3 PngController::calculate_png_acceleration_world(const SensorData & sensor_data, double & closing_speed) const
{
    if (!sensor_data.torpedo_odometry.valid || !sensor_data.target_odometry.valid) {
        return {};
    }

    const util::Vector3 torpedo_position{
        sensor_data.torpedo_odometry.position_x,
        sensor_data.torpedo_odometry.position_y,
        sensor_data.torpedo_odometry.position_z};
    const util::Vector3 target_position{
        sensor_data.target_odometry.position_x,
        sensor_data.target_odometry.position_y,
        sensor_data.target_odometry.position_z};
    const util::Vector3 relative_position = util::subtract(target_position, torpedo_position);
    const double range = util::length(relative_position);
    if (!std::isfinite(range) || range < 1.0e-6) {
        return {};
    }

    const util::Vector3 torpedo_velocity_world = util::rotate(
        util::Vector3{
            sensor_data.torpedo_odometry.linear_x,
            sensor_data.torpedo_odometry.linear_y,
            sensor_data.torpedo_odometry.linear_z},
        sensor_data.torpedo_odometry.orientation_x,
        sensor_data.torpedo_odometry.orientation_y,
        sensor_data.torpedo_odometry.orientation_z,
        sensor_data.torpedo_odometry.orientation_w);
    const util::Vector3 target_velocity_world = util::rotate(
        util::Vector3{
            sensor_data.target_odometry.linear_x,
            sensor_data.target_odometry.linear_y,
            sensor_data.target_odometry.linear_z},
        sensor_data.target_odometry.orientation_x,
        sensor_data.target_odometry.orientation_y,
        sensor_data.target_odometry.orientation_z,
        sensor_data.target_odometry.orientation_w);
    const util::Vector3 relative_velocity = util::subtract(target_velocity_world, torpedo_velocity_world);
    const util::Vector3 los_direction = util::multiply(relative_position, 1.0 / range);
    closing_speed = -util::dot(relative_velocity, los_direction);
    if (!std::isfinite(closing_speed)) {
        closing_speed = 0.0;
        return {};
    }
    if (closing_speed <= 0.0) {
        return {};
    }

    const util::Vector3 los_rate = util::multiply(
        util::cross(relative_position, relative_velocity),
        1.0 / (range * range));
    const util::Vector3 acceleration_world = util::multiply(
        util::cross(los_rate, los_direction),
        config_.png_navigation_constant * closing_speed);

    if (!std::isfinite(util::length(acceleration_world))) {
        return {};
    }
    return acceleration_world;
}

void PngController::update_thrust(InputCommand command)
{
    if (command == InputCommand::ThrottleUp) {
        thrust_ = limit_thrust(thrust_ + config_.thrust_step);
    } else if (command == InputCommand::ThrottleDown) {
        thrust_ = limit_thrust(thrust_ - config_.thrust_step);
    } else if (command == InputCommand::ThrottleStop) {
        thrust_ = 0.0;
    }
}

double PngController::limit_thrust(double value) const
{
    if (value > config_.thrust_max) {
        return config_.thrust_max;
    }

    if (value < config_.thrust_min) {
        return config_.thrust_min;
    }

    return value;
}

double PngController::limit_fin_command(double value) const
{
    if (value > config_.fin_limit_rad) {
        return config_.fin_limit_rad;
    }

    if (value < -config_.fin_limit_rad) {
        return -config_.fin_limit_rad;
    }

    return value;
}

}
