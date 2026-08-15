#include "torpedo_control_v2/control_config.hpp"

namespace torpedo_control_v2
{

ControlConfig make_default_control_config() noexcept
{
    return ControlConfig{
        20.0F,
        100.0F,
        0.0F,
        1000.0F,
        0.05F,
        0.05F,
        0.50F,
        0.80F,
        0.80F,
        0.60F,
        0.15F,
        0.10F,
        3.0F,
        1.0F,
    };
}

}  // namespace torpedo_control_v2
