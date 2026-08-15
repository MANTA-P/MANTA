#include "torpedo_control_v2/keyboard_input.hpp"

#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace
{

void init_terminal(const int tty_fd, termios & original_settings)
{
    // Save the current terminal settings for restoration on exit.
    tcgetattr(tty_fd, &original_settings);

    // Read one key immediately without Enter or terminal echo.
    auto settings = original_settings;
    settings.c_lflag &= ~(ICANON | ECHO);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 1;

    tcsetattr(tty_fd, TCSANOW, &settings);
}

}

namespace torpedo_control_v2
{

KeyboardInput::~KeyboardInput()
{
    stop();
}

void KeyboardInput::start()
{
    running_ = true;
    keyboard_thread_ = std::thread(&KeyboardInput::input_loop, this);
}

void KeyboardInput::stop()
{
    running_ = false;

    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
}

bool KeyboardInput::ok() const
{
    return running_;
}

ControlMode KeyboardInput::mode() const
{
    return requested_mode_;
}

const char * KeyboardInput::mode_name() const
{
    switch (requested_mode_.load()) {
        case ControlMode::None:
            return "None";
        case ControlMode::Keyboard:
            return "Keyboard";
        case ControlMode::SimpleTracking:
            return "SimpleTracking";
        case ControlMode::PNG:
            return "PNG";
    }

    return "Unknown";
}

InputCommand KeyboardInput::take_command()
{
    return input_command_.exchange(InputCommand::None);
}

void KeyboardInput::input_loop()
{
    const int tty_fd = ::open("/dev/tty", O_RDONLY | O_NOCTTY);
    if (tty_fd < 0) {
        std::cout << "keyboard open failed" << std::endl;
        running_ = false;
        return;
    }

    termios original_settings;
    init_terminal(tty_fd, original_settings);

    while (running_) {
        char key{'\0'};
        if (::read(tty_fd, &key, 1) != 1) {
            continue;
        }

        if (key == 'q') {
            running_ = false;
            continue;
        }

        input_command_ = InputCommand::None;
        switch (key) {
            case '0':
                requested_mode_ = ControlMode::None;
                break;
            case '1':
                requested_mode_ = ControlMode::Keyboard;
                break;
            case '2':
                requested_mode_ = ControlMode::SimpleTracking;
                break;
            case '3':
                requested_mode_ = ControlMode::PNG;
                break;
            case 'r':
                input_command_ = InputCommand::ThrottleUp;
                break;
            case 'f':
                input_command_ = InputCommand::ThrottleDown;
                break;
            case ' ':
                input_command_ = InputCommand::ThrottleStop;
                break;
            case 'w':
                input_command_ = InputCommand::PitchUp;
                break;
            case 's':
                input_command_ = InputCommand::PitchDown;
                break;
            case 'a':
                input_command_ = InputCommand::YawLeft;
                break;
            case 'd':
                input_command_ = InputCommand::YawRight;
                break;
            default:
                break;
        }
    }

    tcsetattr(tty_fd, TCSANOW, &original_settings);
    ::close(tty_fd);
}

}
