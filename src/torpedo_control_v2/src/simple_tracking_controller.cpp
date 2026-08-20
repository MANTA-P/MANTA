#include "torpedo_control_v2/simple_tracking_controller.hpp"

#include <cmath>

#include "torpedo_control_v2/util/vector3.hpp"

namespace torpedo_control_v2
{

SimpleTrackingController::SimpleTrackingController(const ControlConfig & config) : config_(config) {}

void SimpleTrackingController::reset()
{
    thrust_ = 0.0;
}

ControlDemand SimpleTrackingController::update(const SensorData & sensor_data, InputCommand command)
{
    update_thrust(command);
    if (!sensor_data.torpedo_odometry.valid || !sensor_data.target_odometry.valid) {
        return ControlDemand{};
    }

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

    ControlDemand demand;
    demand.thrust = thrust_;
    demand.pitch_rad = limit_fin_command(std::atan2(body_vector.z, body_vector.y) * config_.simple_tracking_pitch_kp);
    demand.yaw_rad = limit_fin_command(std::atan2(-body_vector.x, body_vector.y) * config_.simple_tracking_yaw_kp);
    return demand;
}

void SimpleTrackingController::update_thrust(InputCommand command)
{
    if (command == InputCommand::ThrottleUp) {
        thrust_ = limit_thrust(thrust_ + config_.thrust_step);
    } else if (command == InputCommand::ThrottleDown) {
        thrust_ = limit_thrust(thrust_ - config_.thrust_step);
    } else if (command == InputCommand::ThrottleStop) {
        thrust_ = 0.0;
    }
}

double SimpleTrackingController::limit_thrust(double value) const
{
    if (value > config_.thrust_max) {
        return config_.thrust_max;
    }

    if (value < config_.thrust_min) {
        return config_.thrust_min;
    }

    return value;
}

double SimpleTrackingController::limit_fin_command(double value) const
{
    if (value > config_.fin_limit_rad) {
        return config_.fin_limit_rad;
    }

    if (value < -config_.fin_limit_rad) {
        return -config_.fin_limit_rad;
    }

    return value;
}

}  // namespace torpedo_control_v2
