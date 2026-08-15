#pragma once

namespace torpedo_control_v2::util
{

struct Vector3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Vector3 rotate_inverse(
    const Vector3 & vector,
    double orientation_x,
    double orientation_y,
    double orientation_z,
    double orientation_w);

}  // namespace torpedo_control_v2::util
