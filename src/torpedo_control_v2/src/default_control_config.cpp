#include "torpedo_control_v2/control_config.hpp"

namespace torpedo_control_v2
{

ControlConfig make_default_control_config() noexcept
{
    return ControlConfig{
        20.0F,
        1500.0F,
        0.0F,
        1500.0F,
        0.05F,
        0.05F,
    };
}

}  // namespace torpedo_control_v2
