#include "torpedo_control_v2/cli_display.hpp"

#ifdef __linux__

#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace
{

const char * mode_name(const ControlMode mode)
{
    switch (mode) {
        case ControlMode::None:
            return "NONE";
        case ControlMode::Keyboard:
            return "KEYBOARD";
        case ControlMode::SimpleTracking:
            return "SIMPLE TRACKING";
        case ControlMode::PNG:
            return "PNG";
    }

    return "Unknown";
}

int mode_number(const ControlMode mode)
{
    switch (mode) {
        case ControlMode::None:
            return 0;
        case ControlMode::Keyboard:
            return 1;
        case ControlMode::SimpleTracking:
            return 2;
        case ControlMode::PNG:
            return 3;
    }
    return -1;
}

void print_controls(const ControlMode mode)
{
    std::cout << " MODE     0 None | 1 Keyboard | 2 Tracking | 3 PNG" << std::endl;
    std::cout << " THROTTLE r Up   | f Down     | Space Stop" << std::endl;

    if (mode == ControlMode::Keyboard) {
        std::cout << " STEERING w/s Pitch | a/d Yaw" << std::endl;
    } else if (mode == ControlMode::SimpleTracking) {
        std::cout << " STEERING Automatic simple tracking" << std::endl;
    } else if (mode == ControlMode::PNG) {
        std::cout << " STEERING Automatic proportional navigation" << std::endl;
    } else {
        std::cout << " STEERING Disabled" << std::endl;
    }

    std::cout << " QUIT     q" << std::endl;
}

}

namespace
{

constexpr int canvas_width = 35;
constexpr int canvas_height = 17;
constexpr int canvas_center_x = canvas_width / 2;
constexpr int canvas_center_y = canvas_height / 2;

struct Point3
{
    double x;
    double y;
    double z;
};

struct Point2
{
    int x;
    int y;
};

using Canvas = std::array<std::string, canvas_height>;
using Panel = std::array<Point3, 4>;

Point3 rotate_x(const Point3 & point, const double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return Point3{point.x, cosine * point.y - sine * point.z, sine * point.y + cosine * point.z};
}

Point3 rotate_z(const Point3 & point, const double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return Point3{cosine * point.x - sine * point.y, sine * point.x + cosine * point.y, point.z};
}

Point2 project_to_canvas(const Point3 & point)
{
    return Point2{canvas_center_x + static_cast<int>(std::lround(point.x * 3.0)), canvas_center_y - static_cast<int>(std::lround(point.z * 1.5))};
}

void put_character(Canvas & canvas, const int x, const int y, const char character)
{
    if (x >= 0 && x < canvas_width && y >= 0 && y < canvas_height) {
        canvas[y][x] = character;
    }
}

char line_character(const Point2 & start, const Point2 & end)
{
    const int dx = end.x - start.x;
    const int dy = end.y - start.y;
    if (std::abs(dx) > std::abs(dy) * 2) {
        return static_cast<char>(45);
    }
    if (std::abs(dy) > std::abs(dx) * 2) {
        return static_cast<char>(124);
    }
    return (dx >= 0) == (dy >= 0) ? static_cast<char>(92) : static_cast<char>(47);
}

void draw_line(Canvas & canvas, const Point2 & start, const Point2 & end)
{
    const int steps = std::max(std::abs(end.x - start.x), std::abs(end.y - start.y));
    const char character = line_character(start, end);
    if (steps == 0) {
        put_character(canvas, start.x, start.y, character);
        return;
    }

    for (int step = 0; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const int x = static_cast<int>(std::lround(start.x + (end.x - start.x) * ratio));
        const int y = static_cast<int>(std::lround(start.y + (end.y - start.y) * ratio));
        put_character(canvas, x, y, character);
    }
}

void draw_panel(Canvas & canvas, const Panel & panel)
{
    std::array<Point2, 4> projected;
    for (std::size_t index = 0; index < panel.size(); ++index) {
        projected[index] = project_to_canvas(panel[index]);
    }

    const int fill_steps = std::max(std::abs(projected[1].x - projected[0].x), std::abs(projected[1].y - projected[0].y));
    for (int step = 0; step <= fill_steps; ++step) {
        const double ratio = fill_steps == 0 ? 0.0 : static_cast<double>(step) / static_cast<double>(fill_steps);
        const Point2 start{static_cast<int>(std::lround(projected[0].x + (projected[1].x - projected[0].x) * ratio)), static_cast<int>(std::lround(projected[0].y + (projected[1].y - projected[0].y) * ratio))};
        const Point2 end{static_cast<int>(std::lround(projected[3].x + (projected[2].x - projected[3].x) * ratio)), static_cast<int>(std::lround(projected[3].y + (projected[2].y - projected[3].y) * ratio))};
        draw_line(canvas, start, end);
    }

    for (std::size_t index = 0; index < projected.size(); ++index) {
        draw_line(canvas, projected[index], projected[(index + 1) % projected.size()]);
    }
}

enum class StrengthColor
{
    Default,
    Yellow,
    Orange,
    Red
};

StrengthColor strength_color(const double value, const double limit)
{
    if (limit <= 0.0) {
        return StrengthColor::Default;
    }
    const double ratio = std::min(std::abs(value) / limit, 1.0);
    if (ratio < 0.05) {
        return StrengthColor::Default;
    }
    if (ratio < 0.33) {
        return StrengthColor::Yellow;
    }
    if (ratio < 0.66) {
        return StrengthColor::Orange;
    }
    return StrengthColor::Red;
}

const char * color_code(const StrengthColor color)
{
    switch (color) {
        case StrengthColor::Yellow:
            return "[93m";
        case StrengthColor::Orange:
            return "[38;5;208m";
        case StrengthColor::Red:
            return "[91m";
        case StrengthColor::Default:
            return "[0m";
    }
    return "[0m";
}

const char * direction_arrow(const double pitch, const double yaw, const double limit)
{
    constexpr double pi = 3.141592653589793;
    const double direction_x = -yaw;
    const double direction_z = pitch;
    if (std::hypot(direction_x, direction_z) < limit * 0.05) {
        return "O";
    }

    const double angle = std::atan2(direction_z, direction_x);
    if (angle >= -pi / 8.0 && angle < pi / 8.0) {
        return "→";
    }
    if (angle >= pi / 8.0 && angle < 3.0 * pi / 8.0) {
        return "↗";
    }
    if (angle >= 3.0 * pi / 8.0 && angle < 5.0 * pi / 8.0) {
        return "↑";
    }
    if (angle >= 5.0 * pi / 8.0 && angle < 7.0 * pi / 8.0) {
        return "↖";
    }
    if (angle >= 7.0 * pi / 8.0 || angle < -7.0 * pi / 8.0) {
        return "←";
    }
    if (angle >= -7.0 * pi / 8.0 && angle < -5.0 * pi / 8.0) {
        return "↙";
    }
    if (angle >= -5.0 * pi / 8.0 && angle < -3.0 * pi / 8.0) {
        return "↓";
    }
    return "↘";
}

void print_fin_view(const ActuatorCommand & command, const double fin_limit_rad)
{
    constexpr double inner = 1.0;
    constexpr double outer = 4.0;
    constexpr double chord = 3.0;

    Panel top{{Point3{0.0, 0.0, inner}, Point3{0.0, 0.0, outer}, Point3{0.0, chord, outer}, Point3{0.0, chord, inner}}};
    Panel bottom{{Point3{0.0, 0.0, -inner}, Point3{0.0, 0.0, -outer}, Point3{0.0, chord, -outer}, Point3{0.0, chord, -inner}}};
    Panel left{{Point3{-inner, 0.0, 0.0}, Point3{-outer, 0.0, 0.0}, Point3{-outer, chord, 0.0}, Point3{-inner, chord, 0.0}}};
    Panel right{{Point3{inner, 0.0, 0.0}, Point3{outer, 0.0, 0.0}, Point3{outer, chord, 0.0}, Point3{inner, chord, 0.0}}};

    Canvas reference_canvas;
    Canvas top_canvas;
    Canvas bottom_canvas;
    Canvas left_canvas;
    Canvas right_canvas;
    for (int row = 0; row < canvas_height; ++row) {
        const std::string blank_row(canvas_width, static_cast<char>(32));
        reference_canvas[row] = blank_row;
        top_canvas[row] = blank_row;
        bottom_canvas[row] = blank_row;
        left_canvas[row] = blank_row;
        right_canvas[row] = blank_row;
    }

    draw_panel(reference_canvas, top);
    draw_panel(reference_canvas, bottom);
    draw_panel(reference_canvas, left);
    draw_panel(reference_canvas, right);

    for (auto & point : top) {
        point = rotate_z(point, command.fin_top);
    }
    for (auto & point : bottom) {
        point = rotate_z(point, command.fin_bottom);
    }
    for (auto & point : left) {
        point = rotate_x(point, command.fin_left);
    }
    for (auto & point : right) {
        point = rotate_x(point, command.fin_right);
    }

    draw_panel(top_canvas, top);
    draw_panel(bottom_canvas, bottom);
    draw_panel(left_canvas, left);
    draw_panel(right_canvas, right);
    put_character(reference_canvas, canvas_center_x, canvas_center_y, static_cast<char>(79));
    put_character(reference_canvas, canvas_center_x, 0, static_cast<char>(84));
    put_character(reference_canvas, canvas_center_x, canvas_height - 1, static_cast<char>(66));
    put_character(reference_canvas, 0, canvas_center_y, static_cast<char>(76));
    put_character(reference_canvas, canvas_width - 1, canvas_center_y, static_cast<char>(82));

    const StrengthColor top_color = strength_color(command.fin_top, fin_limit_rad);
    const StrengthColor bottom_color = strength_color(command.fin_bottom, fin_limit_rad);
    const StrengthColor left_color = strength_color(command.fin_left, fin_limit_rad);
    const StrengthColor right_color = strength_color(command.fin_right, fin_limit_rad);
    const double pitch = -(command.fin_left + command.fin_right) * 0.5;
    const double yaw = -(command.fin_top + command.fin_bottom) * 0.5;
    const StrengthColor arrow_color = strength_color(std::hypot(pitch, yaw), fin_limit_rad);
    const char * arrow = direction_arrow(pitch, yaw, fin_limit_rad);
    const char blank = static_cast<char>(32);
    const char escape = static_cast<char>(27);

    for (int row = 0; row < canvas_height; ++row) {
        StrengthColor active_color = StrengthColor::Default;
        for (int column = 0; column < canvas_width; ++column) {
            char actual = blank;
            StrengthColor actual_color = StrengthColor::Default;
            if (top_canvas[row][column] != blank) {
                actual = top_canvas[row][column];
                actual_color = top_color;
            }
            if (bottom_canvas[row][column] != blank) {
                actual = bottom_canvas[row][column];
                actual_color = bottom_color;
            }
            if (left_canvas[row][column] != blank) {
                actual = left_canvas[row][column];
                actual_color = left_color;
            }
            if (right_canvas[row][column] != blank) {
                actual = right_canvas[row][column];
                actual_color = right_color;
            }

            const char reference = reference_canvas[row][column];
            const bool arrow_position = row == canvas_center_y && column == canvas_center_x;
            StrengthColor display_color = actual != blank && reference == blank ? actual_color : StrengthColor::Default;
            if (arrow_position) {
                display_color = arrow_color;
            }
            if (display_color != active_color) {
                std::cout << escape << color_code(display_color);
                active_color = display_color;
            }

            if (arrow_position) {
                std::cout << arrow;
            } else {
                std::cout << (actual != blank ? actual : reference);
            }
        }
        if (active_color != StrengthColor::Default) {
            std::cout << escape << color_code(StrengthColor::Default);
        }
        std::cout << std::endl;
    }
}

}

namespace torpedo_control_v2
{

void cli_display(const ControlMode mode, const SensorData & sensor_data, const ActuatorCommand & actuator_command, const double fin_limit_rad, const double thrust_max)
{
    const char escape = static_cast<char>(27);
    const bool stopped = actuator_command.thrust == 0.0;
    const StrengthColor thrust_color = strength_color(actuator_command.thrust, thrust_max);

    std::cout << escape << "[2J" << escape << "[H";
    std::cout << "===================================================" << std::endl;
    std::cout << "                 TORPEDO CONTROL" << std::endl;
    std::cout << "===================================================" << std::endl;
    std::cout << " MODE    : " << escape << "[96m[" << mode_number(mode) << "] " << mode_name(mode) << escape << "[0m" << std::endl;
    std::cout << " STATE   : " << escape << (stopped ? "[93m" : "[92m") << (stopped ? "STANDBY" : "RUNNING") << escape << "[0m" << std::endl;
    std::cout << " SENSOR  : TORPEDO " << escape << (sensor_data.torpedo_odometry.valid ? "[92m" : "[91m") << "[" << (sensor_data.torpedo_odometry.valid ? "OK" : "FAIL") << "]" << escape << "[0m";
    std::cout << "  TARGET " << escape << (sensor_data.target_odometry.valid ? "[92m" : "[91m") << "[" << (sensor_data.target_odometry.valid ? "OK" : "FAIL") << "]" << escape << "[0m" << std::endl;

    if (stopped) {
        std::cout << "---------------------------------------------------" << std::endl;
        print_controls(mode);
    } else {
        std::cout << " SPEED   : " << sensor_data.torpedo_odometry.linear_y << " m/s" << std::endl;
        std::cout << " THRUST  : " << escape << color_code(thrust_color) << actuator_command.thrust << escape << "[0m" << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
        print_fin_view(actuator_command, fin_limit_rad);
    }

    std::cout << "===================================================" << std::endl;
    std::cout << escape << "[0m" << std::flush;
}

}

#endif
