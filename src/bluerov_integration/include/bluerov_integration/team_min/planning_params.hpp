#pragma once

// team_min 소유 ROS 파라미터의 선언·로딩만 담당한다. 파라미터 기본값을
// 손볼 때 이 파일만 다시 컴파일되도록 planning_module에서 분리했다.
// 통합 노드(bluerov_integration_node.cpp)의 loadPlanningConfig()가 선언하는
// 기본 파라미터(map/astar/barrier/topics)와는 역할이 겹치지 않는다.


#include <rclcpp/rclcpp.hpp>

#include "bluerov_integration/team_min/planning_config.hpp"

namespace bluerov_integration::team_min
{

// planning.prediction.* / replan.* / avoid.* / dynamic_vo.* / planner 를
// 선언하고 config에 채운다. 값이 범위를 벗어나면 std::invalid_argument.
void loadTeamMinParameters(rclcpp::Node & node, PlanningConfig & config);

}  // namespace bluerov_integration::team_min
