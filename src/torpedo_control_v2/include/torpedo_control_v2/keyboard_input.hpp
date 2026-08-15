#pragma once

#include <atomic>
#include <thread>

#include "torpedo_control_v2/types.hpp"

namespace torpedo_control_v2
{

class KeyboardInput
{
public:
    ~KeyboardInput();

    void start();
    void stop();
    bool ok() const;
    ControlMode mode() const;
    const char * mode_name() const;
    InputCommand take_command();

private:
    void input_loop();

    std::atomic<bool> running_{false};
    std::atomic<ControlMode> requested_mode_{ControlMode::None};
    std::atomic<InputCommand> input_command_{InputCommand::None};
    std::thread keyboard_thread_;
};

}
