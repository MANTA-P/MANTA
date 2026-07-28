# MANTA 프로젝트

BlueROV 회피 제어와 위협 어뢰 제어를 DAVE 시뮬레이션에서 시험하는 ROS 2 Jazzy 프로젝트다.
Gazebo GUI와 조이스틱은 사용하지 않고 RViz로 상태를 확인한다.

## 저장소 구조

GitHub 저장소에는 `manta_ws` 폴더가 아니라 다음 내부 파일을 올린다.

```text
GitHub 저장소 루트/
├── README.md
└── src/
    ├── bluerov_integration/
    ├── dave/
    ├── torpedo_control/
    └── torpedo_control_v2/
```

## 설치 및 빌드

### 1. 저장소 받기

로컬에 빈 `manta_ws`를 만들고 저장소 내용을 그 안에 받는다.

```bash
mkdir -p ~/manta_ws
cd ~/manta_ws
git clone <GitHub 저장소 주소> .
```

### 2. 의존성 설치

```bash
source /opt/ros/jazzy/setup.bash
cd ~/manta_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 전체 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/manta_ws
colcon build --symlink-install
source install/setup.bash
```

## 터미널 환경 설정

아래 명령은 모든 실행 터미널에서 먼저 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/manta_ws/install/setup.bash
```

## 실행 순서

아래 1~5를 각각 별도 터미널에서 순서대로 실행한다.

### 1. BlueROV와 Gazebo 실행

```bash
ros2 launch dave_demos dave_robot.launch.py \
  headless:=true \
  z:=-0.5 \
  namespace:=bluerov2 \
  world_name:=dave_ocean_waves \
  paused:=false \
  open_virtual_joystick:=false \
  open_qgc:=false
```

### 2. 위협 어뢰 생성

```bash
ros2 launch dave_demos dave_robot.launch.py \
  gui:=false \
  x:=-30 \
  y:=-30 \
  z:=-5 \
  namespace:=glider_slocum \
  world_name:=dave_ocean_waves \
  paused:=false \
  use_teleop:=false \
  open_qgc:=false
```

### 3. BlueROV 통합 패키지 실행

BlueROV 브리지, DataHub, A*, 경로 추종, PPID, CSV 기록과 RViz가 함께 실행된다.

```bash
ros2 launch bluerov_integration bluerov_integration.launch.py
```

### 4. BlueROV 목표 위치 입력

`map` 기준 절대좌표 `x y z`를 입력한다.

```bash
ros2 run bluerov_integration target_position_input_node
```

입력 예:

```text
target> 10 0 -0.5
```

### 5. 위협 어뢰 상태 및 제어 실행

어뢰의 Odometry와 JointState를 ROS 2로 전달하고 어뢰 조종 또는 유도 기능을 실행한다.

```bash
ros2 launch torpedo_control_v2 torpedo_sitl_v2.launch.py
```

이 명령이 실행되면 통합 패키지가 어뢰 위치를 받고 A* 경로 계산과 회피 제어를 시작한다.
정상 상태에서는 통합 노드 로그가 다음처럼 표시된다.

```text
torpedo=1 mission=1 path=1
```

## 코드 업데이트

```bash
cd ~/manta_ws
git pull --ff-only origin main

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

로컬 수정 때문에 `git pull`이 실패하면 강제로 초기화하지 말고 수정 내용을 먼저 공유한다.
