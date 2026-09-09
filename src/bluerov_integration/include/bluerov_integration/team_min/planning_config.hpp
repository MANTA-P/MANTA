#pragma once

// team_min의 설정 데이터다. ROS 타입을 포함하지 않으므로 순수 계획 코어와
// ROS 어댑터 양쪽에서 공통으로 사용할 수 있다. ROS 파라미터 선언과 로딩은
// planning_params.hpp/.cpp에만 둔다.

#include <optional>
#include <string>

#include "bluerov_integration/team_min/dynamic_vo_planner.hpp"
#include "bluerov_integration/team_min/planning_types.hpp"
#include "bluerov_integration/team_min/spacetime_astar.hpp"

namespace bluerov_integration::team_min
{

struct PlanningConfig
{
  bool enabled{true};
  bool use_dynamic_map{true};
  bool use_target_topic_for_goal{true};
  double goal_offset_x{100.0};
  double goal_offset_y{0.0};
  double goal_offset_z{0.0};
  double map_padding_x{10.0};
  double map_padding_y{15.0};
  double map_padding_z{15.0};
  double torpedo_replan_distance{0.5};
  double robot_replan_distance{1.0};
  double goal_replan_distance{0.1};
  GridMapConfig fixed_map{};
  AStarOptions astar{};
  DynamicVOOptions dynamic_vo{};
  BoxObstacle torpedo_barrier{};
  PredictionConfig prediction{};
  ReplanPolicyConfig replan{};
  AvoidConfig avoid{};
  TtcSwitchConfig ttc{};

  // 아래 문자열은 ROS 어댑터가 사용하는 입출력 이름이다. 문자열 자체는
  // ROS 타입이 아니므로 설정 구조체를 순수 C++ 계층에 두어도 ROS 헤더가
  // 전파되지 않는다.
  std::string path_topic{"/uuv/reference_path"};
  std::string current_point_topic{"/uuv/current_position_point"};
  std::string goal_point_topic{"/uuv/goal_point"};
  std::string torpedo_point_topic{"/uuv/torpedo_center_point"};
  std::string marker_topic{"/rviz/uuv_astar_markers"};

  SpaceTimeOptions spacetime{};
  double spacetime_lookahead{15.0};
};

// 평면 설정에서 판단 코어가 사용하는 알고리즘 설정만 뽑는다.
PlanningCoreConfig toCoreConfig(const PlanningConfig & config);

// 파라미터 문자열을 순수 enum으로 변환한다.
std::optional<PlannerType> parsePlanner(const std::string & name);

}  // namespace bluerov_integration::team_min
