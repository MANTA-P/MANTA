#include "control_algorithm.h"

#include <cmath>
#include <cstdint>

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/control_core.hpp"
#include "torpedo_control_v2/png_controller.hpp"
#include "torpedo_control_v2/simple_tracking_controller.hpp"
#include "torpedo_control_v2/types.hpp"

namespace {

uint16_t read_u16(const uint8_t *p)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int16_t read_i16(const uint8_t *p)
{
    const uint16_t raw = read_u16(p);
    return static_cast<int16_t>(raw <= INT16_MAX ? raw : static_cast<int32_t>(raw) - 65536);
}

int32_t read_i32(const uint8_t *p)
{
    const uint32_t raw = (static_cast<uint32_t>(p[0]) << 24) |
                         (static_cast<uint32_t>(p[1]) << 16) |
                         (static_cast<uint32_t>(p[2]) << 8) | p[3];
    return static_cast<int32_t>(raw <= INT32_MAX ? raw : static_cast<int64_t>(raw) - 4294967296LL);
}

bool decode_odometry(const uint8_t *data, OdometryData &out)
{
    if (data == nullptr) return false;
    OdometryData next;
    next.position_x = read_i32(data) * 0.001;
    next.position_y = read_i32(data + 4) * 0.001;
    next.position_z = read_i32(data + 8) * 0.001;
    next.orientation_x = read_i16(data + 12) / 16384.0;
    next.orientation_y = read_i16(data + 14) / 16384.0;
    next.orientation_z = read_i16(data + 16) / 16384.0;
    next.orientation_w = read_i16(data + 18) / 16384.0;
    next.linear_x = read_i16(data + 20) * 0.001;
    next.linear_y = read_i16(data + 22) * 0.001;
    next.linear_z = read_i16(data + 24) * 0.001;
    next.angular_x = read_i16(data + 26) * 0.001;
    next.angular_y = read_i16(data + 28) * 0.001;
    next.angular_z = read_i16(data + 30) * 0.001;
    const double quaternion_norm = std::sqrt(
        next.orientation_x * next.orientation_x + next.orientation_y * next.orientation_y +
        next.orientation_z * next.orientation_z + next.orientation_w * next.orientation_w);
    if (!std::isfinite(quaternion_norm) || quaternion_norm < 0.5 || quaternion_norm > 1.5) {
        return false;
    }
    next.valid = true;
    out = next;
    return true;
}

int16_t fin_to_mrad(double value)
{
    return static_cast<int16_t>(std::lround(value * 1000.0));
}

SensorData sensor_data;
uint8_t previous_mode = 0;
const auto config = torpedo_control_v2::make_default_control_config();
torpedo_control_v2::ControlCore core(config);
torpedo_control_v2::SimpleTrackingController simple(config);
torpedo_control_v2::PngController png(config);

}  // namespace

extern "C" bool control_algorithm_set_odometry(uint8_t source, const uint8_t data[32])
{
    if (source == 0) return decode_odometry(data, sensor_data.torpedo_odometry);
    if (source == 1) return decode_odometry(data, sensor_data.target_odometry);
    return false;
}

extern "C" bool control_algorithm_compute(uint8_t mode, uint16_t target_thrust,
                                          control_algorithm_output_t *output)
{
    if (output == nullptr) return false;
    *output = {};
    if (mode != previous_mode) {
        previous_mode = mode;
        simple.reset();
        png.reset();
    }
    if (mode != 1 && mode != 2) return false;
    if (!sensor_data.torpedo_odometry.valid || !sensor_data.target_odometry.valid) return false;

    ControlDemand demand;
    if (mode == 1) {
        simple.set_thrust(target_thrust);
        demand = simple.update(sensor_data, InputCommand::None);
    } else {
        png.set_thrust(target_thrust);
        demand = png.update(sensor_data, InputCommand::None);
    }
    const ActuatorCommand command = core.update(demand, sensor_data);
    if (!std::isfinite(command.thrust) || !std::isfinite(command.fin_top) ||
        !std::isfinite(command.fin_bottom) || !std::isfinite(command.fin_left) ||
        !std::isfinite(command.fin_right) || command.thrust < 0.0 ||
        command.thrust > config.thrust_max ||
        std::abs(command.fin_top) > config.fin_limit_rad + 1e-9 ||
        std::abs(command.fin_bottom) > config.fin_limit_rad + 1e-9 ||
        std::abs(command.fin_left) > config.fin_limit_rad + 1e-9 ||
        std::abs(command.fin_right) > config.fin_limit_rad + 1e-9) {
        return false;
    }
    output->thrust = static_cast<uint16_t>(std::lround(command.thrust));
    output->fin_top = fin_to_mrad(command.fin_top);
    output->fin_bottom = fin_to_mrad(command.fin_bottom);
    output->fin_left = fin_to_mrad(command.fin_left);
    output->fin_right = fin_to_mrad(command.fin_right);
    return true;
}
