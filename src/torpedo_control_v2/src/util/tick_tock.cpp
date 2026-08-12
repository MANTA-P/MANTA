#include "torpedo_control_v2/util/tick_tock.hpp"

#include <chrono>

namespace torpedo_control_v2::util
{

TickTock::TickTock()
{
    start();
}

long long TickTock::get_now_us()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

void TickTock::start()
{
    start_time_us = get_now_us();
    tick_time_us = start_time_us;
}

void TickTock::tick()
{
    tick_time_us = get_now_us();
}

long long TickTock::tock() const
{
    return get_now_us() - tick_time_us;
}


long long TickTock::total_us()
{
    return get_now_us() - start_time_us;
}

}  // namespace torpedo_control_v2::util
