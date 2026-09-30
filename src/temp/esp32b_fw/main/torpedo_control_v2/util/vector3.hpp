#pragma once

namespace torpedo_control_v2::util
{

struct Vector3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Vector3 subtract(const Vector3 & left, const Vector3 & right);
Vector3 multiply(const Vector3 & vector, double value);
double dot(const Vector3 & left, const Vector3 & right);
Vector3 cross(const Vector3 & left, const Vector3 & right);
double length(const Vector3 & vector);

Vector3 rotate(
    const Vector3 & vector,
    double orientation_x,
    double orientation_y,
    double orientation_z,
    double orientation_w);

Vector3 rotate_inverse(
    const Vector3 & vector,
    double orientation_x,
    double orientation_y,
    double orientation_z,
    double orientation_w);

}  // namespace torpedo_control_v2::util
