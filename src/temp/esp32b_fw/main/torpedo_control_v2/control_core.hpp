#pragma once

#include "torpedo_control_v2/actuator_mixer.hpp"
#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/roll_controller.hpp"
#include "torpedo_control_v2/types.hpp"

namespace torpedo_control_v2
{

class ControlCore
{
public:
    explicit ControlCore(const ControlConfig & config);

    ActuatorCommand update(const ControlDemand & demand, const SensorData & sensor_data) const;

private:
    RollController roll_controller_;
    ActuatorMixer mixer_;
};

}
