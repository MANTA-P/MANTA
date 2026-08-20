#pragma once

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/mode_controller.hpp"

namespace torpedo_control_v2
{

class SimpleTrackingController : public ModeController
{
public:
    explicit SimpleTrackingController(const ControlConfig & config);

    void reset() override;
    ControlDemand update(const SensorData & sensor_data, InputCommand command) override;

private:
    void update_thrust(InputCommand command);
    double limit_thrust(double value) const;
    double limit_fin_command(double value) const;

    ControlConfig config_;
    double thrust_{0.0};
};

}  // namespace torpedo_control_v2
