#pragma once

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/types.hpp"

namespace torpedo_control_v2
{

class RollController
{
public:
    explicit RollController(const ControlConfig & config);

    double update(const OdometryData & torpedo_odometry) const;

private:
    double limit(double value) const;

    ControlConfig config_;
};

}
