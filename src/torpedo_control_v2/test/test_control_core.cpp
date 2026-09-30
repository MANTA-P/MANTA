#include <gtest/gtest.h>

#include <cmath>

#include "torpedo_control_v2/control_core.hpp"

namespace torpedo_control_v2
{

TEST(ControlCore, SafeOutputsWhenInputOdometryIsExpired)
{
  const auto config = make_default_control_config();
  ControlCore core(config);

  ControlDemand demand;
  demand.thrust = 400.0;
  demand.pitch_rad = 0.1;
  demand.yaw_rad = 0.2;

  SensorData sensor_data;
  sensor_data.torpedo_odometry.valid = false;
  sensor_data.target_odometry.valid = false;

  const auto result = core.update(demand, sensor_data);

  EXPECT_DOUBLE_EQ(result.thrust, 0.0);
  EXPECT_DOUBLE_EQ(result.fin_top, 0.0);
  EXPECT_DOUBLE_EQ(result.fin_bottom, 0.0);
  EXPECT_DOUBLE_EQ(result.fin_left, 0.0);
  EXPECT_DOUBLE_EQ(result.fin_right, 0.0);
}

TEST(ControlCore, PassesNormallyWhenOdometryIsValid)
{
  const auto config = make_default_control_config();
  ControlCore core(config);

  ControlDemand demand;
  demand.thrust = 300.0;
  demand.pitch_rad = 0.05;
  demand.yaw_rad = -0.03;

  SensorData sensor_data;
  sensor_data.torpedo_odometry.valid = true;
  sensor_data.target_odometry.valid = true;

  const auto result = core.update(demand, sensor_data);

  EXPECT_GT(result.thrust, 0.0);
  EXPECT_TRUE(std::isfinite(result.fin_top));
  EXPECT_TRUE(std::isfinite(result.fin_bottom));
  EXPECT_TRUE(std::isfinite(result.fin_left));
  EXPECT_TRUE(std::isfinite(result.fin_right));
}

}  // namespace torpedo_control_v2
