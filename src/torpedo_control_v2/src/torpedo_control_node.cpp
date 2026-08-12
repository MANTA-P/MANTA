#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/types.hpp"
#include "torpedo_control_v2/ros_interface.hpp"
#include "torpedo_control_v2/util/tick_tock.hpp"


int main(int argc, char * argv[])
{
    const auto config = torpedo_control_v2::make_default_control_config();

    std::cout << "control config" << "\nupdate_rate_hz: " << config.update_rate_hz << std::endl;

    auto ros_interface = RosInterface::create(argc, argv);
    ros_interface->start();

    torpedo_control_v2::util::TickTock control_ticktock;

    while (ros_interface->ok()) {
        const auto data = ros_interface->latest_sensor_data();

        if (data.torpedo_odometry.valid) {
            std::cout << "torpedo failed" << std::endl;
        }

        if (!data.target_odometry.valid) {
            std::cout << "target failed" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    ros_interface->stop();
    return 0;
}
