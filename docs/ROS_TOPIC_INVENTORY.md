# MANTA ROS 2 주요 토픽 실측 기록

## 목적

MANTA 프로젝트의 BlueROV SITL 실행 중 생성되는 ROS 2 토픽을 확인하고,
PC-ESP32 HIL 브리지에서 보드로 전달할 입력 데이터와 보드에서 돌려받을
제어 출력을 선정하기 위한 기준으로 사용한다.

## 측정 환경

| 항목 | 값 |
| --- | --- |
| 측정 날짜 | 2026-08-29 |
| ROS 배포판 | ROS 2 Jazzy |
| 작업공간 | `/home/user/manta_ws` |
| 실행 구성 | DAVE/Gazebo BlueROV + `bluerov_integration` + `torpedo_control_v2` |
| 빌드 | `colcon build --symlink-install`, 전체 17개 패키지 성공 |
| 측정 시간 | 토픽별 약 8초, `ros2 topic hz --window 200` |

## 주요 토픽 측정 결과

Hz는 ROS 메시지를 PC에서 실제로 수신한 벽시계 기준이다. 일시적인 discovery와
CPU 정체 구간을 제외하고 안정화된 마지막 측정값을 기록했다.

| 분류 | 토픽 | 메시지 타입 | Publisher | 안정화 실측 Hz | HIL 방향 | 상태/비고 |
| --- | --- | --- | ---: | ---: | --- | --- |
| BlueROV IMU | `/model/bluerov2/imu` | `sensor_msgs/msg/Imu` | 1 | 약 100 | PC -> ESP32 | 정상 발행 |
| BlueROV 위치/속도 | `/model/bluerov2/odometry` | `nav_msgs/msg/Odometry` | 1 | 약 100 | PC -> ESP32 | 정상 발행 |
| 압력 플러그인 원본 | `/model/bluerov2/Pressure` | `sensor_msgs/msg/FluidPressure` | 1 | 약 50 | PC -> ESP32 | 20 ms 주기 정상 확인 |
| 깊이 추정 | `/model/bluerov2/Pressure_depth` | `geometry_msgs/msg/PointStamped` | 1 | 약 50 | PC -> ESP32 | 압력 플러그인에서 계산 |
| 브리지 압력 | `/model/bluerov2/pressure` | `sensor_msgs/msg/FluidPressure` | 1 | 약 50 | PC -> ESP32 | Gazebo-ROS 브리지 토픽 |
| DVL 속도 | `/dvl/velocity` | `dave_interfaces/msg/DVL` | 0 | 측정 불가 | PC -> ESP32 | subscriber만 있고 현재 publisher 없음 |
| 임무 목표점 | `/mission/target_position` | `geometry_msgs/msg/PointStamped` | 1 | 이벤트 기반 | PC -> ESP32 | 목표 입력 시 발행, transient-local |
| 추적 목표점 | `/ppid/tracking_target` | `geometry_msgs/msg/PointStamped` | 1 | 이벤트 기반 | PC -> ESP32 | 측정 중 메시지 없음, transient-local |
| BlueROV 추진기 1~6 | `/model/bluerov2/joint/thruster{1..6}_joint/cmd_thrust` | `std_msgs/msg/Float64` | 각 1 | 유휴 상태 | ESP32 -> PC 후보 | publisher는 있으나 측정 중 명령 없음 |
| 어뢰 상태 | `/torpedo/state/odometry` | `nav_msgs/msg/Odometry` | 1 | 약 100 | SITL 내부 | 최신 제어 로직에서 100 Hz 확인 |
| 어뢰 추진기 출력 | `/torpedo/actuators/thruster/command` | `std_msgs/msg/Float64` | 1 | 약 20 | SITL 내부 | 50 ms 주기 정상 발행 |
| TF | `/tf` | `tf2_msgs/msg/TFMessage` | 1 | 약 100 | SITL/RViz 내부 | 시각화 및 좌표 변환용 |
| 시뮬레이션 시계 | `/clock` | `rosgraph_msgs/msg/Clock` | 0 | 측정 불가 | 내부 | 현재 ROS 그래프에는 subscriber만 존재 |

## 실행 중 확인된 노드

```text
/bluerov_integration_node
/bluerov_integration_rviz
/bluerov_parameter_bridge
/subsea_pressure_sensor
/target_position_input_node
/torpedo_control_node_v2
/torpedo_ros_gz_bridge_v2
/transform_listener_impl_...
```

## 전체 토픽 목록

```text
/clock [rosgraph_msgs/msg/Clock]
/dvl/velocity [dave_interfaces/msg/DVL]
/mission/target_position [geometry_msgs/msg/PointStamped]
/model/bluerov2/Pressure [sensor_msgs/msg/FluidPressure]
/model/bluerov2/Pressure_depth [geometry_msgs/msg/PointStamped]
/model/bluerov2/imu [sensor_msgs/msg/Imu]
/model/bluerov2/joint/thruster1_joint/cmd_thrust [std_msgs/msg/Float64]
/model/bluerov2/joint/thruster2_joint/cmd_thrust [std_msgs/msg/Float64]
/model/bluerov2/joint/thruster3_joint/cmd_thrust [std_msgs/msg/Float64]
/model/bluerov2/joint/thruster4_joint/cmd_thrust [std_msgs/msg/Float64]
/model/bluerov2/joint/thruster5_joint/cmd_thrust [std_msgs/msg/Float64]
/model/bluerov2/joint/thruster6_joint/cmd_thrust [std_msgs/msg/Float64]
/model/bluerov2/odometry [nav_msgs/msg/Odometry]
/model/bluerov2/pressure [sensor_msgs/msg/FluidPressure]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/ppid/tracking_target [geometry_msgs/msg/PointStamped]
/rosout [rcl_interfaces/msg/Log]
/rviz/uuv_astar_markers [visualization_msgs/msg/Marker]
/rviz/uuv_astar_markers_array [visualization_msgs/msg/MarkerArray]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/torpedo/actuators/fins/bottom/command [std_msgs/msg/Float64]
/torpedo/actuators/fins/left/command [std_msgs/msg/Float64]
/torpedo/actuators/fins/right/command [std_msgs/msg/Float64]
/torpedo/actuators/fins/top/command [std_msgs/msg/Float64]
/torpedo/actuators/thruster/command [std_msgs/msg/Float64]
/torpedo/state/odometry [nav_msgs/msg/Odometry]
/uuv/current_position_point [geometry_msgs/msg/PointStamped]
/uuv/goal_point [geometry_msgs/msg/PointStamped]
/uuv/reference_path [nav_msgs/msg/Path]
/uuv/torpedo_center_point [geometry_msgs/msg/PointStamped]
```

## HIL 적용 시 우선 전달할 데이터

초기 PC -> ESP32 브리지 구현은 다음 순서로 진행한다.

1. IMU: orientation, angular velocity, linear acceleration
2. BlueROV odometry: position, orientation, linear/angular velocity
3. 압력 또는 깊이: 같은 원천 데이터이므로 제어 로직 요구에 따라 하나를 선택
4. 목표점: 새 목표가 들어올 때 전송하고 보드에서 마지막 값을 유지
5. DVL: publisher가 활성화된 뒤 필드와 실제 Hz를 추가 측정

보드 -> PC 방향은 6개 추진기 명령을 고정 주기로 보내고, PC 노드가 이를
각 `cmd_thrust` 토픽에 publish하는 구조로 잡는다. 어뢰 제어는 현재 계획대로
SITL에 유지하므로 초기 HIL UART 패킷 대상에서 제외한다.

## 측정 시 발견한 사항

- 압력 센서 소스에는 50 Hz 제한이 적용되어 있으며 재빌드 후 실측도 약 50 Hz다.
- 이전에 관측된 1000 Hz는 재빌드 전 Gazebo 프로세스가 백그라운드에 남아
  구형 라이브러리를 계속 사용했기 때문이다.
- DVL은 토픽 이름과 subscriber는 생성됐지만 publisher가 없어 현재 데이터를
  받을 수 없다. DVL Gazebo 센서/브리지 실행 구성을 별도로 확인해야 한다.
- 추진기 토픽은 publisher와 subscriber가 연결돼 있어도 목표나 제어 조건이
  활성화되지 않으면 측정 구간 동안 메시지가 나오지 않을 수 있다.
- `/clock`은 현재 ROS publisher가 보이지 않으므로 각 노드의 `use_sim_time`
  사용 여부를 확인하기 전까지 UART timestamp 기준으로 사용하지 않는다.

## 재측정 명령

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/manta_ws/install/setup.bash

ros2 topic list -t
ros2 topic info --verbose /model/bluerov2/imu
ros2 topic hz /model/bluerov2/imu --window 200
ros2 topic hz /model/bluerov2/Pressure --window 200
ros2 interface show sensor_msgs/msg/Imu
```

UART payload를 확정하기 전에는 각 메시지 전체를 그대로 직렬화하지 않고,
제어에 필요한 필드만 선정한 뒤 big-endian wire 자료형과 목표 전송 주기를
별도의 패킷 표로 확정한다.
