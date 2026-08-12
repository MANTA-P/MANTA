#pragma once

namespace torpedo_control_v2
{

struct ControlConfig
{
    float update_rate_hz;
    float thrust_step;
    float thrust_min;
    float thrust_max;
    float pitch_step_rad;
    float yaw_step_rad;
};

ControlConfig make_default_control_config() noexcept;

}  // namespace torpedo_control_v2
