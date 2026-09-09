#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "bluerov_integration/team_min/planning_types.hpp"

namespace bluerov_integration::team_min
{

struct GuidancePrediction
{
  TorpedoGuidanceLaw law{TorpedoGuidanceLaw::kUnknown};
  double proportional_navigation_error{0.0};
  double pure_pursuit_error{0.0};
  std::vector<Point3D> proportional_navigation_path;
  std::vector<Point3D> pure_pursuit_path;
  std::vector<Point3D> selected_path;
};

// 위치 이력으로 얻은 map-frame 속도를 이용해 PN과 순수추적의 one-step
// 예측 오차를 온라인 비교하고, 선택된 모델로 미래 곡선 궤적을 만든다.
class TorpedoGuidancePredictor
{
public:
  explicit TorpedoGuidancePredictor(PredictionConfig config);

  GuidancePrediction update(
    const VehicleState & torpedo,
    const Point3D & torpedo_velocity,
    bool torpedo_velocity_valid,
    const VehicleState & target,
    const Point3D & target_velocity,
    bool target_velocity_valid);
  void reset();

  struct KinematicState
  {
    Point3D torpedo_position;
    Point3D torpedo_velocity;
    Point3D target_position;
    Point3D target_velocity;
  };

private:
  static KinematicState propagate(
    KinematicState state,
    TorpedoGuidanceLaw law,
    double dt,
    const PredictionConfig & config);
  static std::vector<Point3D> predictPath(
    KinematicState state,
    TorpedoGuidanceLaw law,
    const PredictionConfig & config);
  static double sampleTime(const VehicleState & sample);

  PredictionConfig config_;
  bool have_previous_{false};
  std::uint64_t previous_sequence_{0};
  double previous_time_{0.0};
  KinematicState previous_{};
  std::size_t scored_samples_{0U};
  double pn_error_{0.0};
  double pursuit_error_{0.0};
  TorpedoGuidanceLaw law_{TorpedoGuidanceLaw::kUnknown};
};

}  // namespace bluerov_integration::team_min
