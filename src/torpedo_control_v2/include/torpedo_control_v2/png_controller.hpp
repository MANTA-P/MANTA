#pragma once

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/mode_controller.hpp"
#include "torpedo_control_v2/util/vector3.hpp"

namespace torpedo_control_v2
{

class PngController : public ModeController
{
public:
    explicit PngController(const ControlConfig & config);

    void reset() override;
    ControlDemand update(const SensorData & sensor_data, InputCommand command) override;
    const char * guidance_mode_name() const;
    double closing_speed() const;

private:
    util::Vector3 calculate_png_acceleration_world(const SensorData & sensor_data, double & closing_speed) const;
    void update_thrust(InputCommand command);
    double limit_thrust(double value) const;
    double limit_fin_command(double value) const;

    ControlConfig config_;
    double thrust_{0.0};
    double closing_speed_{0.0};
};

}
