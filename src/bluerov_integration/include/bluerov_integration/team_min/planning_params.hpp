#pragma once

// team_min 소유 ROS 파라미터의 선언·로딩만 담당한다. 파라미터 기본값을
// 손볼 때 이 파일만 다시 컴파일되도록 planning_module에서 분리했다.
// 통합 노드(bluerov_integration_node.cpp)의 loadPlanningConfig()가 선언하는
// 기본 파라미터(map/astar/barrier/topics)와는 역할이 겹치지 않는다.

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/team_min/planning_module.hpp"
#include "bluerov_integration/team_min/planning_types.hpp"

namespace bluerov_integration::team_min
{

// planning.prediction.* / replan.* / avoid.* / dynamic_vo.* / planner 를
// 선언하고 config에 채운다. 값이 범위를 벗어나면 std::invalid_argument.
void loadTeamMinParameters(rclcpp::Node & node, PlanningConfig & config);

// 노드가 채운 평면 PlanningConfig에서 코어가 아는 값만 옮긴다.
// ROS 토픽명은 코어로 넘어가지 않는다.
PlanningCoreConfig toCoreConfig(const PlanningConfig & config);

// 파라미터 문자열 -> PlannerType. 알 수 없는 값이면 비어있는 optional을
// 돌려주고 호출부가 거부한다(잘못된 값에 기존 플래너를 유지하기 위해).
std::optional<PlannerType> parsePlanner(const std::string & name);

}  // namespace bluerov_integration::team_min
