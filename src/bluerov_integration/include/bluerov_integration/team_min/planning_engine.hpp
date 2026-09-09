#pragma once

// A*, DVO, 시공간 A*의 선택·조합과 전역 경로 캐시를 소유하는 순수 C++
// 실행 계층이다. ROS 메시지, rclcpp, 로거, publisher를 사용하지 않는다.

#include <optional>
#include <vector>

#include "bluerov_integration/team_min/dynamic_vo_planner.hpp"
#include "bluerov_integration/team_min/planning_core.hpp"
#include "bluerov_integration/team_min/spacetime_astar.hpp"

namespace bluerov_integration::team_min
{

struct PlanningEngineConfig
{
  PlanningCoreConfig core{};
  DynamicVOOptions dynamic_vo{};
  SpaceTimeOptions spacetime{};
  double spacetime_lookahead{15.0};
};

// 센서 틱에서 만든 요청과 교전 상태를 worker가 한 묶음으로 처리한다.
struct PlanningWork
{
  EngagementPhase phase{EngagementPhase::kCruise};
  double ttc_sec{0.0};
  PlanRequest request;
  bool global_replan_required{false};
};

struct PlanningEngineOutput
{
  PlanResult result;
  // 전역 A*가 갱신됐을 때만 채운다. ROS 어댑터는 이 경로를
  // PlanningCore의 다음 재계획 판단 입력으로 보관한다.
  std::optional<std::vector<Point3D>> replan_reference_path;
  bool jink_active{false};
};

class PlanningEngine
{
public:
  explicit PlanningEngine(PlanningEngineConfig config);

  PlanningEngineOutput plan(const PlanningWork & work);
  void reset();

private:
  PlanningEngineConfig config_;
  std::vector<Point3D> global_path_;
};

}  // namespace bluerov_integration::team_min
