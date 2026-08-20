#pragma once

namespace torpedo_control_v2::util
{

class TickTock
{
private:
    long long tick_time_us{0};
    long long start_time_us{0};
    static long long get_now_us();
public:
    TickTock();
    void start();
    void tick();
    long long tock() const;
    long long total_us();
};

}  // namespace torpedo_control_v2::util
