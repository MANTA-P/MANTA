#pragma once

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/types.hpp"

namespace torpedo_control_v2
{

class ActuatorMixer
{
public:
    explicit ActuatorMixer(const ControlConfig & config);

    ActuatorCommand mix(const ControlDemand & demand, double roll_command_rad) const;

private:
    double limit(double value, double limit_value) const;

    ControlConfig config_;
};

}  // namespace torpedo_control_v2
