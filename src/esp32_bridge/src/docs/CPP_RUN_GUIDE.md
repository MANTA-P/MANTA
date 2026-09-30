# `src`의 C++ 파일 빌드·실행 방법

이 디렉터리의 `.cpp` 파일 3개는 ROS 2 실행 파일 하나를 구성한다. `ros_uart_tx_node.cpp`에 `main()`이 있고, 나머지 두 파일은 이 노드에서 사용하는 기능을 제공한다. 따라서 `.cpp` 파일을 각각 실행하지 않고 패키지를 빌드한 뒤 노드를 실행한다.

| 파일 | 역할 | 단독 실행 |
| --- | --- | --- |
| `ros_uart_tx_node.cpp` | ROS 2 토픽과 ESP32 UART 사이의 브리지 노드 | `packet_codec.cpp`, `uart_port.cpp`와 함께 빌드 |
| `packet_codec.cpp` | UART 프레임 인코딩·파싱 | 불가 (`main()` 없음) |
| `uart_port.cpp` | Linux 직렬 포트 입출력 | 불가 (`main()` 없음) |

## 준비

- Linux 환경에 ROS 2 Jazzy, `colcon`, 이 패키지의 ROS 의존성(`ament_cmake`, `rclcpp`, `nav_msgs`, `std_msgs`)이 설치되어 있어야 한다.
- ESP32가 PC에 연결되어 있어야 한다. 기본 장치 경로는 `/dev/ttyACM0`, 기본 통신 속도는 `921600` baud이다.
- 아래 명령은 이 저장소가 `/home/user/manta_ws/src/esp32_bridge`에 있는 현재 작업 공간을 기준으로 한다. 경로가 다르면 자신의 ROS 2 작업 공간 경로로 바꾼다.

장치 경로와 접근 권한은 실행 전에 확인할 수 있다.

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
groups
```

장치는 보이는데 권한 오류가 나면 장치의 소유 그룹을 확인하고, 해당 그룹에 사용자 계정을 추가한 뒤 새 로그인 세션에서 다시 실행한다. 일반적인 Ubuntu 환경에서는 `dialout` 그룹을 사용한다.

## 빌드 및 실행

작업 공간 루트에서 다음을 순서대로 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/user/manta_ws
colcon build --packages-select esp32_bridge --symlink-install
source install/setup.bash
ros2 run esp32_bridge esp32_ros_uart_tx_node
```

다른 ROS 2 배포판을 사용한다면 첫 줄의 `jazzy`를 설치된 배포판 이름으로 바꾼다. 실행 중에는 터미널이 점유되며, 종료할 때는 `Ctrl+C`를 누른다. 새 터미널에서 `ros2` 명령으로 상태를 확인할 때도 ROS 2와 작업 공간의 `setup.bash`를 다시 `source`해야 한다.

직렬 포트가 `/dev/ttyACM1`이라면 다음과 같이 지정한다.

```bash
ros2 run esp32_bridge esp32_ros_uart_tx_node --ros-args \
  -p device:=/dev/ttyACM1 \
  -p baud_rate:=921600
```

제어 명령도 실행 인자로 설정할 수 있다. 다음 값은 실제 구동 명령을 전송하므로 장치 상태를 확인한 후 사용한다.

```bash
ros2 run esp32_bridge esp32_ros_uart_tx_node --ros-args \
  -p control.armed:=true \
  -p control.mode:=1 \
  -p control.target_thrust:=500
```

기본값은 `control.armed=false`, `control.mode=0`, `control.target_thrust=0`이다. `control.mode`는 `0=None`, `1=Simple`, `2=PNG`이고 `control.target_thrust` 범위는 `0..1000`이다.

## 실행 확인과 문제 해결

다른 터미널에서 작업 공간을 설정한 다음 노드를 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/manta_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /torpedo/actuators/thruster/command
```

노드 이름은 `/esp32_torpedo_hil_bridge_node`이다. 입력 odometry 토픽의 기본값은 `/model/bluerov2/odometry`, `/torpedo/state/odometry`이다. ESP32로부터 유효한 actuator 프레임이 200 ms 동안 들어오지 않으면 출력 actuator 토픽에는 안전값 `0.0`이 발행될 수 있으므로, 토픽이 보이거나 값이 출력되는 것만으로 UART 수신 성공을 판단하지 않는다. 실행 터미널의 `UART tx=... rx=...` 통계에서 `rx` 프레임 수가 증가하는지도 확인한다.

- `failed to open /dev/ttyACM0`: 장치 경로, USB 연결, 접근 권한을 확인한다. 실제 경로가 다르면 `-p device:=...`로 지정한다.
- `Package 'esp32_bridge' not found`: 작업 공간에서 빌드가 성공했는지 확인하고 `source /home/user/manta_ws/install/setup.bash`를 다시 실행한다.
- `unsupported baud rate`: `baud_rate`를 코드가 지원하는 값(`9600`, `19200`, `38400`, `57600`, `115200`, `230400`, `460800`, `921600`) 중 하나로 설정한다. ESP32 측 속도와도 일치해야 한다.

프레임 형식과 토픽에 관한 자세한 설명은 [패키지 README](../README.md)와 [구현 명세](TORPEDO_HIL_AI_IMPLEMENTATION_SPEC.md)를 참고한다.
