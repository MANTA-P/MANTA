#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "torpedo_control_v2/types.hpp"
#include "torpedo_control_v2/ros_interface.hpp"

int main(int argc, char * argv[])
{
    auto ros_interface = RosInterface::create(argc, argv);
    ros_interface->start();

    while (ros_interface->ok()) {
        const auto data = ros_interface->latest_sensor_data();

        std::cout
            << "torpedo odometry valid: "
            << (data.torpedo_odometry.valid ? "true" : "false")
            << ", x: " << data.torpedo_odometry.position_x
            << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ros_interface->stop();
    return 0;
}
