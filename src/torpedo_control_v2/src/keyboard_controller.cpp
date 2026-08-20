#include "torpedo_control_v2/keyboard_controller.hpp"

namespace torpedo_control_v2
{

KeyboardController::KeyboardController(const ControlConfig & config) : config_(config) {}

void KeyboardController::reset()
{
    demand_ = ControlDemand{};
}

ControlDemand KeyboardController::update(const SensorData &, InputCommand command)
{
    if (command == InputCommand::ThrottleUp) {
        demand_.thrust = limit(demand_.thrust + config_.thrust_step, config_.thrust_min, config_.thrust_max);
    } else if (command == InputCommand::ThrottleDown) {
        demand_.thrust = limit(demand_.thrust - config_.thrust_step, config_.thrust_min, config_.thrust_max);
    } else if (command == InputCommand::ThrottleStop) {
        demand_.thrust = 0.0;
    }

    if (command == InputCommand::PitchUp) {
        demand_.pitch_rad = limit(demand_.pitch_rad + config_.pitch_step_rad, -config_.fin_limit_rad, config_.fin_limit_rad);
    } else if (command == InputCommand::PitchDown) {
        demand_.pitch_rad = limit(demand_.pitch_rad - config_.pitch_step_rad, -config_.fin_limit_rad, config_.fin_limit_rad);
    } else if (command == InputCommand::YawLeft) {
        demand_.yaw_rad = limit(demand_.yaw_rad + config_.yaw_step_rad, -config_.fin_limit_rad, config_.fin_limit_rad);
    } else if (command == InputCommand::YawRight) {
        demand_.yaw_rad = limit(demand_.yaw_rad - config_.yaw_step_rad, -config_.fin_limit_rad, config_.fin_limit_rad);
    }
    return demand_;
}

double KeyboardController::limit(double value, double minimum, double maximum) const
{
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

}  // namespace torpedo_control_v2
