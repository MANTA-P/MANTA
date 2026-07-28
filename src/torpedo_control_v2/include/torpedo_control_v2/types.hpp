#pragma once

#include <string>
#include <vector>

struct OdometryData
{
    bool valid{false};

    double position_x{0.0};
    double position_y{0.0};
    double position_z{0.0};

    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double orientation_w{1.0};

    double linear_x{0.0};
    double linear_y{0.0};
    double linear_z{0.0};

    double angular_x{0.0};
    double angular_y{0.0};
    double angular_z{0.0};
};

struct JointStateData
{
    bool valid{false};
    std::vector<std::string> names;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
};

struct SensorData
{
    OdometryData torpedo_odometry;
    OdometryData target_odometry;
    JointStateData joint_states;
};

struct ActuatorCommand
{
    double thrust{0.0};
    double fin_top{0.0};
    double fin_bottom{0.0};
    double fin_left{0.0};
    double fin_right{0.0};
};
