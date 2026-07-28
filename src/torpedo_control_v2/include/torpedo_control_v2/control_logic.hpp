#pragma once

#include <string>
#include <vector>

#include "torpedo_control_v2/control_types.hpp"

namespace torpedo_control_v2
{

class ControlLogic
{
public:
  explicit ControlLogic(ControllerConfig config);

  // This class is owned and called only by the control timer. It deliberately
  // has no mutex: all shared data arrives as a value-copy snapshot.
  ControlResult initial_result() const;
  ControlResult step(const StateSnapshot & state, const std::vector<KeyEvent> & events);
  ControlResult emergency_stop();

private:
  struct Vector3
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  static Vector3 rotate_inverse(const Vector3 & vector, double qx, double qy,
    double qz, double qw);
  static double roll_angle_from_orientation(const OdometryState & odometry);
  static std::string append_event(std::string current, const std::string & event);

  double limited_fin_command(double value) const;
  double clamp_fin_output(double value) const;
  double limited_roll_command(double value) const;
  void handle_key(char key, std::string & events);
  ActuatorCommand make_command() const;
  ControlResult result_from_state(const StateSnapshot & state, std::string events);

  ControllerConfig config_;
  double thrust_level_{0.0};
  double pitch_target_rad_{0.0};
  double yaw_target_rad_{0.0};
  double roll_correction_rad_{0.0};
  bool auto_track_mode_{false};
};

}  // namespace torpedo_control_v2
