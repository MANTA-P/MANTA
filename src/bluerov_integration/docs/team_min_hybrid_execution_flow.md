# team_min 기본 Hybrid 실행 흐름

## 1. Hybrid의 의미

이 프로젝트에서 `hybrid`는 일반적인 차량 자세 기반의 Hybrid A* 알고리즘을 의미하지 않는다.

`team_min`의 Hybrid는 다음 두 플래너를 결합한 구조다.

- 3D A*: 전체 목표까지의 전역 경로 생성
- Dynamic VO(DVO): 움직이는 어뢰에 대한 짧은 국소 회피 경로 생성

전체 데이터 흐름은 다음과 같다.

```text
센서 및 목표 ROS 메시지
        ↓
PlanningModule::update()
        ↓
PlanningCore::update()       ROS 독립
  - ROV/어뢰 속도 추정
  - 어뢰 탐지
  - TTC 계산
  - Cruise/Approach/Break 결정
  - A* 재계획 필요성 판단
        ↓
계획 worker thread
        ↓
PlanningModule::runPlanner()
  - 전역 3D A*
  - 필요하면 Dynamic VO
  - 전역·국소 경로 병합
        ↓
nav_msgs::msg::Path 발행
        ↓
PathFollower → team_byung PPID → 추진기 명령
```

## 2. 프로그램 시작

진입점은 `src/bluerov_integration_node.cpp`의 `main()`이다.

개념적인 실행 코드는 다음과 같다.

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BlueRovIntegrationNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
```

`BlueRovIntegrationNode` 생성자는 다음 모듈을 생성한다.

```cpp
hub_ = std::make_unique<common::DataHub>(...);
planning_module_ = std::make_unique<team_min::PlanningModule>(...);
path_follower_ = std::make_unique<integration::PathFollower>(...);
control_module_ = std::make_unique<team_byung::ControlModule>(...);
```

그리고 다음 타이머를 생성한다.

- 계획 타이머: 기본 5 Hz
- 제어 타이머: 기본 20 Hz
- 상태 출력 타이머: 기본 1 Hz

`PlanningModule`이 생성될 때 `planning.planner`를 따로 지정하지 않으면 기본값인 `hybrid`가 선택된다. 별도의 계획 worker 스레드도 이 시점에 시작된다.

## 3. 필요한 입력 토픽

기본 Hybrid 계획에 필요한 입력은 다음과 같다.

| 입력 | 기본 토픽 | 메시지 타입 |
|---|---|---|
| BlueROV 위치 | `/model/bluerov2/odometry` | `nav_msgs/msg/Odometry` |
| 어뢰 위치 | `/torpedo/state/odometry` | `nav_msgs/msg/Odometry` |
| 미션 목표 | `/mission/target_position` | `geometry_msgs/msg/PointStamped` |

어뢰 odometry의 `twist` 값은 사용하지 않는다. 연속된 어뢰 위치를 차분해 map 프레임 속도를 자체적으로 추정한다.

```text
어뢰 위치(t-1), 어뢰 위치(t)
            ↓
          속도 추정
            ↓
       미래 궤적 예측
            ↓
       A* + DVO Hybrid
```

BlueROV 위치 또는 미션 목표가 없으면 경로를 생성하지 않는다. BlueROV, 어뢰, 목표의 좌표 프레임도 서로 호환되어야 한다.

## 4. DataHub의 메시지 저장

ROS 구독 콜백은 `src/common/data_hub.cpp`에 있다. 콜백에서는 계획을 수행하지 않고 최신 메시지를 `StateSnapshot`에 저장한다.

예를 들어 어뢰 odometry 콜백은 다음과 같다.

```cpp
void DataHub::torpedoOdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  store(state_.torpedo_odometry, *message);
}
```

저장 관계는 다음과 같다.

```text
/model/bluerov2/odometry
    → state_.bluerov_odometry

/torpedo/state/odometry
    → state_.torpedo_odometry

/mission/target_position
    → state_.mission_goal
```

`DataHub::snapshot()`은 mutex로 보호된 최신 상태의 복사본을 반환한다.

## 5. 5Hz 계획 타이머

`src/bluerov_integration_node.cpp`의 계획 타이머는 기본적으로 0.2초마다 실행된다.

```cpp
planning_timer_ = create_wall_timer(
  std::chrono::duration<double>(1.0 / planning_rate_hz),
  [this]() {
    const auto local_snapshot = hub_->snapshot();
    planning_module_->update(local_snapshot);
  },
  planning_group_);
```

계획 타이머는 최신 센서 상태를 가져와 `PlanningModule::update()`에 전달한다.

## 6. ROS 메시지를 순수 계획 타입으로 변환

`PlanningModule::makePlanningInput()`은 ROS 메시지를 `PlanningInput`으로 변환한다.

```cpp
PlanningInput input;

input.bluerov.position = {
  odometry.pose.pose.position.x,
  odometry.pose.pose.position.y,
  odometry.pose.pose.position.z
};

input.torpedo.position = {
  torpedo.pose.pose.position.x,
  torpedo.pose.pose.position.y,
  torpedo.pose.pose.position.z
};

input.goal.point = {
  goal.point.x,
  goal.point.y,
  goal.point.z
};

input.planner = PlannerType::kHybrid;
```

변환 후 ROS 독립 코드인 `PlanningCore::update()`를 호출한다.

```cpp
const Decision decision = core_->update(input, last_path_copy);
```

## 7. PlanningCore 판단

`PlanningCore`는 `src/team_min/planning_core.cpp`에 있으며 ROS 타입이나 ROS clock을 직접 사용하지 않는다.

### 7.1 속도 추정

BlueROV와 어뢰 속도는 연속된 위치 샘플로 계산한다.

```text
velocity = (current_position - previous_position) / dt
```

어뢰 속도에는 `prediction.velocity_alpha`를 이용한 EMA 필터가 적용된다.

### 7.2 어뢰 탐지

어뢰 odometry가 기본 2초 이내에 갱신됐으면 탐지 상태로 본다.

```cpp
const bool torpedo_detected = sampleFresh(
  input.torpedo,
  input.now_sec,
  config_.avoid.torpedo_timeout_sec);
```

어뢰 메시지가 오래되면 `NORMAL` 모드로 복귀하고 어뢰 장애물 없이 계획한다.

### 7.3 어뢰 유도법과 미래 경로 예측

`TorpedoGuidancePredictor`는 관측된 어뢰 위치와 속도를 이용해 다음 두 모델의 한 단계 예측 오차를 비교한다.

- 비례항법(Proportional Navigation, PN)
- 순수추적(Pure Pursuit)

더 잘 맞는 모델이 식별되면 해당 모델로 미래 곡선 궤적을 만든다. 아직 유도법을 식별하지 못했다면 등속 직선 경로를 사용한다.

### 7.4 TTC와 교전 단계

TTC(Time To Collision)는 거리만 사용하지 않고 상대속도를 LOS(Line of Sight) 방향으로 투영해 계산한다.

```text
상대 위치 = 어뢰 위치 - ROV 위치
상대 속도 = 어뢰 속도 - ROV 속도
접근 속도 = -(상대 속도 · LOS 단위벡터)
TTC       = 거리 / 접근 속도
```

기본 단계 전환은 다음과 같다.

| 조건 | 단계 | 동작 |
|---|---|---|
| `TTC >= 6.0초` 또는 접근하지 않음 | `Cruise` | 전역 A* 중심 |
| `1.85초 <= TTC < 6.0초` | `Approach` | A* + 일반 DVO |
| `TTC < 1.85초` | `Break` | LOS 수직 급기동 강화 |
| `TTC < 1.45초` | `Break + Jink` | 직전 횡기동의 반대 방향 선호 |

위험한 단계로는 즉시 전환한다. 덜 위험한 단계로 돌아갈 때는 기본 1.5초의 최소 체류시간을 적용해 단계가 반복해서 흔들리는 것을 줄인다.

### 7.5 A* 재계획 판단

기본 `collision_only=true`에서는 어뢰가 조금 움직일 때마다 전역 A*를 다시 계산하지 않는다. 다음 조건에서 재계획한다.

- 아직 전역 경로가 없음
- 프레임 변경
- 목표 변경
- ROV가 기존 경로에서 기준 이상 이탈
- 남은 경로가 새 어뢰 예측 통로와 충돌

Hybrid 모드는 어뢰가 탐지되는 동안 매 계획 틱마다 worker 요청을 만들지만, `global_replan_required=false`이면 기존 전역 A* 경로를 재사용한다.

```cpp
case PlannerType::kHybrid:
  if (torpedo_detected || decision.global_replan_required) {
    decision.plan_request = request;
  }
  break;
```

## 8. 계획 worker 스레드

계획 타이머는 A*나 DVO를 직접 실행하지 않는다. `PlanningWork`를 `pending_request_`에 저장하고 worker를 깨운다.

```cpp
pending_request_ = PlanningWork{
  decision.phase,
  decision.ttc_sec,
  *decision.plan_request,
  decision.global_replan_required
};

request_cv_.notify_one();
```

`PlanningModule::workerLoop()`가 요청을 받아 `execute()`를 호출한다.

```cpp
while (running_.load()) {
  waitForRequest();
  PlanningWork work = latestRequest();
  execute(work);
}
```

worker가 계산하는 동안 요청이 여러 번 들어오면 대기 중인 오래된 요청 대신 최신 요청을 남긴다. 센서 콜백과 계획 타이머가 긴 A* 계산 때문에 직접 정지하지 않도록 만든 구조다.

## 9. Hybrid 계획 실행

실제 Hybrid 조합은 `src/team_min/planning_module.cpp`의 `PlanningModule::runPlanner()`에 있다.

### 9.1 어뢰 예측 통로 생성

어뢰의 현재 위치와 예측 경로를 여러 개의 장애물 박스로 변환한다.

```cpp
const auto obstacles = buildTorpedoObstacles(request, core_config_);
```

```text
현재 어뢰 위치 → 예측 위치 → 예측 위치 → 예측 위치
      □              □            □            □
```

A*는 이 박스들의 집합을 하나의 정적 예측 통로처럼 취급한다.

### 9.2 전역 3D A*

전역 경로가 없거나 재계획이 필요하면 `runEnhancedAStar3D()`를 실행한다.

```cpp
if (work.global_replan_required || global_path_.empty()) {
  global_path_ = runEnhancedAStar3D(
    request.start,
    request.goal,
    map,
    obstacles,
    core_config_.astar);
}
```

A*는 박스 장애물을 점유 격자로 바꾸고 3차원에서 최대 26방향 이웃을 탐색한다. 대각선 이동 중 장애물 모서리를 관통하지 않도록 중간 셀도 검사한다.

```text
ROV ───────── 전역 A* 경로 ───────── 미션 목표
```

### 9.3 Cruise

`Cruise` 단계에서는 DVO를 사용하지 않고 전역 A* 경로를 그대로 반환한다.

```text
ROV → 전역 A* 경로 → 목표
```

### 9.4 Approach와 Break

`Approach` 또는 `Break` 단계에서는 전역 경로에서 현재 위치보다 앞에 있는 local target을 선택하고 `runDynamicVO3D()`를 실행한다.

```cpp
const DynamicVOResult vo = runDynamicVO3D(
  request.start,
  request.robot_velocity,
  target.point,
  map,
  moving_obstacles,
  options);
```

DVO는 기본적으로 다음 규모의 속도 후보를 만든다.

```text
수평 방향 36개
× 수직 방향 7개
× 속력 5개
≈ 1,260개 후보
```

각 후보에 대해 미래 6초 동안 움직이는 어뢰와의 최근접 거리를 계산한다. 안전한 후보 가운데 다음 비용을 비교해 하나를 선택한다.

- local target까지의 거리
- 목표 지향 속도와의 차이
- 직전 속도와의 차이
- 어뢰와의 예측 이격거리
- Break 단계의 LOS 수직 속도 성분

안전한 후보가 하나도 없으면 정지하지 않고 예측 충돌 여유가 가장 큰 후보를 선택한다.

선택된 속도로 기본 0.5초씩 최대 20단계를 전개해 국소 경로를 만든다.

### 9.5 전역·국소 경로 병합

DVO가 회피 필요 상태이면 국소 경로 뒤에 전역 A* 경로의 남은 부분을 붙인다.

```cpp
result.path = mergePaths(
  vo.local_path,
  global_path_,
  target.index);
```

```text
현재 위치
   │
   ├── DVO 국소 회피 경로
   │
   └──────── A* 경로에 재합류 ───── 미션 목표
```

## 10. ROS Path 발행

계획에 성공하면 `PlanningModule::publishPlan()`이 자체 `Point3D` 경로를 `nav_msgs::msg::Path`로 변환한다.

```cpp
nav_msgs::msg::Path path_message;

for (const auto & waypoint : result.path) {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = waypoint.z;
  path_message.poses.push_back(std::move(pose));
}

path_pub_->publish(path_message);
```

기본 출력 토픽은 다음과 같다.

```text
/uuv/reference_path
```

계획 실패가 회피 경로 없음처럼 즉시 정지가 필요한 상황이면 빈 Path를 발행한다. `PathFollower`는 빈 Path를 받으면 기존 목표를 무효화한다.

## 11. 20Hz 경로 추종과 제어

계획 타이머와 별도로 제어 타이머가 기본 20Hz로 실행된다.

```cpp
control_timer_ = create_wall_timer(
  std::chrono::duration<double>(1.0 / control_rate_hz),
  [this]() {
    const auto local_snapshot = hub_->snapshot();
    const auto tracking_target =
      path_follower_->selectTarget(local_snapshot);
    control_module_->update(local_snapshot, tracking_target);
  },
  control_group_);
```

`PathFollower`는 발행된 경로에서 현재 ROV 위치보다 기본 2m 앞에 있는 점을 추적 목표로 고른다.

```text
현재 ROV → 경로상 lookahead 지점 → PPID 위치 목표
```

PPID 제어 흐름은 다음과 같다.

```text
목표 위치 - 현재 위치
       ↓
     목표 속도
       ↓
목표 속도 - 현재 속도
       ↓
     속도 PID
       ↓
   축별 제어 출력
       ↓
  6개 추진기 명령 혼합
```

`team_min_planning_test.launch.py`는 `control.enabled=false`로 실행하므로 경로와 RViz 시각화만 확인하고 추진기 명령은 활성화하지 않는다.

## 12. 현재 DVO 속도 전달의 한계

DVO는 `selected_velocity`를 계산하지만 현재 실행 경로에서는 이 속도를 기체에 직접 전달하지 않는다.

```text
DVO가 속도 선택
      ↓
선택한 속도로 가상 경로점 생성
      ↓
nav_msgs::msg::Path로 변환
      ↓
PathFollower가 위치 경로를 보고 목표 속도를 다시 계산
```

따라서 현재 Hybrid는 위치 경로 기반으로 동작한다. `velocity_command.cpp`와 `velocity_pid.cpp`에 속도 직접 제어를 위한 순수 C++ 코드가 작성돼 있지만 아직 실제 ROS 실행 파이프라인에는 연결되지 않았다.

## 13. 기본 Hybrid 실행

워크스페이스를 빌드한다.

```bash
cd /home/minjae/manta_ws
colcon build --packages-select bluerov_integration --symlink-install
source install/setup.bash
```

경로 계획과 RViz를 확인하려면 다음 launch 파일을 실행한다.

```bash
ros2 launch bluerov_integration team_min_planning_test.launch.py
```

`planning.planner`를 지정하지 않았기 때문에 기본값인 `hybrid`가 사용된다.

플래너 선택 상태는 다음 명령으로 확인한다.

```bash
ros2 param get /bluerov_integration_node planning.planner
```

예상 결과는 다음과 같다.

```text
String value is: hybrid
```

입력 토픽 상태를 확인한다.

```bash
ros2 topic hz /model/bluerov2/odometry
ros2 topic hz /torpedo/state/odometry
ros2 topic echo --once /mission/target_position
```

경로 출력은 다음 명령으로 확인한다.

```bash
ros2 topic echo /uuv/reference_path
```

목표가 없다면 BlueROV odometry와 동일한 `frame_id`로 목표를 발행한다. 예를 들어 프레임이 `map`인 경우 다음과 같다.

```bash
ros2 topic pub --once \
  /mission/target_position \
  geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 100.0, y: 0.0, z: -5.0}}"
```

제어까지 활성화해 실제 추진기 명령을 생성하려면 설정 파일로 통합 노드를 실행한다.

```bash
ros2 run bluerov_integration bluerov_integration_node \
  --ros-args \
  --params-file /home/minjae/manta_ws/src/bluerov_integration/config/integration.yaml
```

이 경우 Gazebo, BlueROV odometry, 어뢰 odometry 발행 노드 및 필요한 bridge가 먼저 실행 중이어야 한다.

## 14. 축약한 코드 흐름

전체 구조를 의사 코드로 줄이면 다음과 같다.

```cpp
// 계획 타이머: 5Hz
void planningTick()
{
  StateSnapshot snapshot = hub.snapshot();
  PlanningInput input = convert(snapshot);

  Decision decision = core.update(input, last_path);

  if (decision.plan_request) {
    planning_worker.submit(*decision.plan_request);
  }
}

// 계획 worker thread
void planningWorker(const PlanningWork & work)
{
  Path global = cached_or_new_astar_path(work);

  if (work.phase == EngagementPhase::kCruise) {
    publish(global);
    return;
  }

  LocalPath avoidance = runDynamicVO(work, global);
  Path hybrid = merge(avoidance, global);
  publish(hybrid);
}

// 제어 타이머: 20Hz
void controlTick()
{
  Point3D target = pathFollower.selectTarget();
  ThrusterCommand command = controller.update(target);
  publishThrusters(command);
}
```

