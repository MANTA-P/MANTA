#include <chrono>
#include <memory>
#include <thread>
#ifdef __linux__
#include "torpedo_control_v2/cli_display.hpp"
#endif

#include "torpedo_control_v2/control_config.hpp"
#include "torpedo_control_v2/control_core.hpp"
#include "torpedo_control_v2/keyboard_controller.hpp"
#include "torpedo_control_v2/keyboard_input.hpp"
#include "torpedo_control_v2/mode_controller.hpp"
#include "torpedo_control_v2/png_controller.hpp"
#include "torpedo_control_v2/ros_interface.hpp"
#include "torpedo_control_v2/simple_tracking_controller.hpp"
#include "torpedo_control_v2/types.hpp"
#include "torpedo_control_v2/util/tick_tock.hpp"

int main(int argc, char * argv[])
{
    // config init
    const auto config = torpedo_control_v2::make_default_control_config();
    torpedo_control_v2::ControlCore control_core(config);
    torpedo_control_v2::SimpleTrackingController simple_tracking_controller(config);
    torpedo_control_v2::PngController png_controller(config);

    // ros init
    auto ros_interface = RosInterface::create(argc, argv);
    ros_interface->start();

    // keyboard init
    torpedo_control_v2::KeyboardInput keyboard_input;
    torpedo_control_v2::KeyboardController keyboard_controller(config);
    keyboard_input.start();

    // timer init
    torpedo_control_v2::util::TickTock control_ticktock;
#ifdef __linux__
    torpedo_control_v2::util::TickTock status_ticktock;
#endif
    const long long control_period_us = 1'000'000 / config.update_rate_hz;
    ControlMode current_mode{ControlMode::None};
    torpedo_control_v2::ModeController *current_controller = nullptr;

    while (keyboard_input.ok() && ros_interface->ok()) {
        const long long elapsed_us = control_ticktock.tock();
        if (elapsed_us < control_period_us) {
            std::this_thread::sleep_for(std::chrono::microseconds(control_period_us - elapsed_us));
        }
        control_ticktock.tick();

        const auto mode = keyboard_input.mode();
        if (mode != current_mode) {
            current_mode = mode;
            current_controller = nullptr;
            if (current_mode == ControlMode::Keyboard) {
                current_controller = &keyboard_controller;
            } else if (current_mode == ControlMode::SimpleTracking) {
                current_controller = &simple_tracking_controller;
            } else if (current_mode == ControlMode::PNG) {
                current_controller = &png_controller;
            }
            if (current_controller != nullptr) {
                current_controller->reset();
            }
        }

        const auto input_command = keyboard_input.take_command();
        const auto sensor_data = ros_interface->latest_sensor_data();

        ActuatorCommand actuator_command;
        if (current_controller != nullptr) {
            const auto demand = current_controller->update(sensor_data, input_command);
            actuator_command = control_core.update(demand, sensor_data);
        }

        ros_interface->publish_command(actuator_command);

#ifdef __linux__
        if (status_ticktock.tock() >= 100000) {
            status_ticktock.tick();
            torpedo_control_v2::cli_display(current_mode, sensor_data, actuator_command, config.fin_limit_rad, config.thrust_max, png_controller.guidance_mode_name(), png_controller.closing_speed());
        }
#endif
    }

    // end
    keyboard_input.stop();
    ros_interface->stop();
    return 0;
}
