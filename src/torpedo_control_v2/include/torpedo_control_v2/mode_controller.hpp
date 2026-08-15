#pragma once

#include "torpedo_control_v2/types.hpp"

namespace torpedo_control_v2
{

class ModeController
{
public:
    virtual ~ModeController() = default;

    virtual void reset() = 0;
    virtual ControlDemand update(const SensorData & sensor_data, InputCommand command) = 0;
};

}
