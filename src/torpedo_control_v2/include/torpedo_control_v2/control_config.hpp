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
    float fin_limit_rad;
    float simple_tracking_pitch_kp;
    float simple_tracking_yaw_kp;
    float roll_kp;
    float roll_kd;
    float roll_limit_rad;
    float png_navigation_constant;
    float png_acceleration_to_fin_gain;
};

ControlConfig make_default_control_config() noexcept;

}  // namespace torpedo_control_v2
