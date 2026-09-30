#include "torpedo_control_v2/roll_controller.hpp"

#include <cmath>

#include "torpedo_control_v2/util/vector3.hpp"

namespace torpedo_control_v2
{

RollController::RollController(const ControlConfig & config) : config_(config) {}

double RollController::update(const OdometryData & torpedo_odometry) const
{
    if (!torpedo_odometry.valid) {
        return 0.0;
    }

    const util::Vector3 world_up_in_body = util::rotate_inverse(
        {0.0, 0.0, 1.0},
        torpedo_odometry.orientation_x,
        torpedo_odometry.orientation_y,
        torpedo_odometry.orientation_z,
        torpedo_odometry.orientation_w);

    const double roll_angle_rad = std::atan2(-world_up_in_body.x, world_up_in_body.z);
    const double roll_error_rad = -roll_angle_rad;
    const double roll_command_rad =
        config_.roll_kp * roll_error_rad - config_.roll_kd * torpedo_odometry.angular_y;
    return limit(roll_command_rad);
}

double RollController::limit(double value) const
{
    if (value > config_.roll_limit_rad) {
        return config_.roll_limit_rad;
    }

    if (value < -config_.roll_limit_rad) {
        return -config_.roll_limit_rad;
    }

    return value;
}

}
