#pragma once

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/mode_controller.hpp"

namespace torpedo_control_v2
{

class KeyboardController : public ModeController
{
public:
    explicit KeyboardController(const ControlConfig & config);

    void reset() override;
    ControlDemand update(const SensorData & sensor_data, InputCommand command) override;

private:
    double limit(double value, double minimum, double maximum) const;

    ControlConfig config_;
    ControlDemand demand_;
};

}  // namespace torpedo_control_v2
