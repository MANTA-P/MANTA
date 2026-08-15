#pragma once

#ifdef __linux__

#include "torpedo_control_v2/types.hpp"

namespace torpedo_control_v2
{

void cli_display(ControlMode mode, const SensorData & sensor_data, const ActuatorCommand & actuator_command, double fin_limit_rad, double thrust_max, const char * png_guidance_mode, double png_closing_speed);

}

#endif
