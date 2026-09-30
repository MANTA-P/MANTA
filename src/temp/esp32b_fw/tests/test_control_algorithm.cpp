#include <cassert>
#include <cstdint>
#include <cstdlib>

#include "main/control_algorithm.h"

static void put16(uint8_t *p, int16_t value)
{
    const uint16_t raw = static_cast<uint16_t>(value);
    p[0] = static_cast<uint8_t>(raw >> 8);
    p[1] = static_cast<uint8_t>(raw);
}

static void put32(uint8_t *p, int32_t value)
{
    const uint32_t raw = static_cast<uint32_t>(value);
    p[0] = static_cast<uint8_t>(raw >> 24);
    p[1] = static_cast<uint8_t>(raw >> 16);
    p[2] = static_cast<uint8_t>(raw >> 8);
    p[3] = static_cast<uint8_t>(raw);
}

int main()
{
    uint8_t torpedo[32] = {};
    uint8_t target[32] = {};
    put16(torpedo + 18, 16384); // identity quaternion
    put16(target + 18, 16384);
    put32(target + 0, 1000);    // +1 m world x
    put32(target + 4, 10000);   // +10 m forward (world y)
    put32(target + 8, 1000);    // +1 m world z
    assert(control_algorithm_set_odometry(0, torpedo));
    assert(control_algorithm_set_odometry(1, target));

    control_algorithm_output_t output = {};
    assert(control_algorithm_compute(1, 500, &output));
    assert(output.thrust == 500);
    assert(std::abs(output.fin_top - 80) <= 1);
    assert(std::abs(output.fin_bottom - 80) <= 1);
    assert(std::abs(output.fin_left + 80) <= 1);
    assert(std::abs(output.fin_right + 80) <= 1);
    assert(std::abs(output.fin_top) <= 500 && std::abs(output.fin_bottom) <= 500);
    assert(std::abs(output.fin_left) <= 500 && std::abs(output.fin_right) <= 500);

    put16(torpedo + 22, 1000); // +1 m/s toward target
    assert(control_algorithm_set_odometry(0, torpedo));
    assert(control_algorithm_compute(2, 500, &output));
    assert(output.thrust == 500);
    assert(control_algorithm_compute(2, 0, &output));
    assert(output.thrust == 0 && output.fin_top == 0 && output.fin_left == 0);

    put16(torpedo + 18, 0); // invalid quaternion cannot replace last good sample
    assert(!control_algorithm_set_odometry(0, torpedo));
    assert(!control_algorithm_compute(0, 500, &output));
    assert(output.thrust == 0);
}
