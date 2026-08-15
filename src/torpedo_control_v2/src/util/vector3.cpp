#include "torpedo_control_v2/util/vector3.hpp"

#include <cmath>

namespace torpedo_control_v2::util
{

Vector3 rotate_inverse(
    const Vector3 & vector,
    double orientation_x,
    double orientation_y,
    double orientation_z,
    double orientation_w)
{
    const double norm = std::sqrt(
        orientation_x * orientation_x +
        orientation_y * orientation_y +
        orientation_z * orientation_z +
        orientation_w * orientation_w);

    if (!std::isfinite(norm) || norm < 1.0e-12) {
        return vector;
    }

    const double x = orientation_x / norm;
    const double y = orientation_y / norm;
    const double z = orientation_z / norm;
    const double w = orientation_w / norm;
    const Vector3 inverse{-x, -y, -z};
    const Vector3 twice_cross{
        2.0 * (inverse.y * vector.z - inverse.z * vector.y),
        2.0 * (inverse.z * vector.x - inverse.x * vector.z),
        2.0 * (inverse.x * vector.y - inverse.y * vector.x)};

    return {
        vector.x + w * twice_cross.x + inverse.y * twice_cross.z - inverse.z * twice_cross.y,
        vector.y + w * twice_cross.y + inverse.z * twice_cross.x - inverse.x * twice_cross.z,
        vector.z + w * twice_cross.z + inverse.x * twice_cross.y - inverse.y * twice_cross.x};
}

}  // namespace torpedo_control_v2::util
