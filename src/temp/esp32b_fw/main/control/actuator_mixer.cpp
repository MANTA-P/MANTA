#include "torpedo_control_v2/actuator_mixer.hpp"

#include <algorithm>
#include <cmath>

namespace torpedo_control_v2
{

ActuatorMixer::ActuatorMixer(const ControlConfig & config)
: config_(config)
{
}

double ActuatorMixer::limit(double value, double limit_value) const
{
    if (value > limit_value) {
        return limit_value;
    }

    if (value < -limit_value) {
        return -limit_value;
    }

    return value;
}

ActuatorCommand ActuatorMixer::mix(const ControlDemand & demand, double roll_command_rad) const
{
    const double fin_limit_rad = config_.fin_limit_rad;
    const double requested_pitch_rad = limit(demand.pitch_rad, fin_limit_rad);
    const double requested_yaw_rad = limit(demand.yaw_rad, fin_limit_rad);
    const double roll_rad = limit(roll_command_rad, fin_limit_rad);
    const double pitch_margin_rad = fin_limit_rad - std::abs(requested_pitch_rad);
    const double yaw_margin_rad = fin_limit_rad - std::abs(requested_yaw_rad);

    double roll_pitch_magnitude_rad = 0.0;
    double roll_yaw_magnitude_rad = 0.0;
    double remaining_roll_rad = std::abs(roll_rad);

    if (pitch_margin_rad > yaw_margin_rad) {
        const double first_allocation_rad = std::min(remaining_roll_rad, pitch_margin_rad - yaw_margin_rad);
        roll_pitch_magnitude_rad += first_allocation_rad;
        remaining_roll_rad -= first_allocation_rad;
    } else {
        const double first_allocation_rad = std::min(remaining_roll_rad, yaw_margin_rad - pitch_margin_rad);
        roll_yaw_magnitude_rad += first_allocation_rad;
        remaining_roll_rad -= first_allocation_rad;
    }

    roll_pitch_magnitude_rad += remaining_roll_rad * 0.5;
    roll_yaw_magnitude_rad += remaining_roll_rad * 0.5;

    const double roll_direction = roll_rad < 0.0 ? -1.0 : 1.0;
    const double roll_pitch_rad = roll_pitch_magnitude_rad * roll_direction;
    const double roll_yaw_rad = roll_yaw_magnitude_rad * roll_direction;
    const double pitch_rad = limit(requested_pitch_rad, fin_limit_rad - roll_pitch_magnitude_rad);
    const double yaw_rad = limit(requested_yaw_rad, fin_limit_rad - roll_yaw_magnitude_rad);

    ActuatorCommand command;
    command.thrust = demand.thrust;
    command.fin_left = -pitch_rad + roll_pitch_rad;
    command.fin_right = -pitch_rad - roll_pitch_rad;
    command.fin_top = -yaw_rad - roll_yaw_rad;
    command.fin_bottom = -yaw_rad + roll_yaw_rad;
    return command;
}

}  // namespace torpedo_control_v2
