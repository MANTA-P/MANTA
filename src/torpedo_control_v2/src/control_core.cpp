#include "torpedo_control_v2/control_core.hpp"

namespace torpedo_control_v2
{

ControlCore::ControlCore(const ControlConfig & config) : roll_controller_(config), mixer_(config) {}

ActuatorCommand ControlCore::update(const ControlDemand & demand, const SensorData & sensor_data) const
{
    if (demand.thrust == 0.0) {
        return ActuatorCommand{};
    }

    const double roll_command_rad = roll_controller_.update(sensor_data.torpedo_odometry);
    return mixer_.mix(demand, roll_command_rad);
}

}
