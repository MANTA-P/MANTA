#include "torpedo_control_v2/png_controller.hpp"

namespace torpedo_control_v2
{

PngController::PngController(const ControlConfig & config) : config_(config) {}

void PngController::reset()
{
    thrust_ = 0.0;
}

ControlDemand PngController::update(const SensorData &, InputCommand command)
{
    update_thrust(command);
    return ControlDemand{};
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

}
