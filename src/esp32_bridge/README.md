# ESP32 Torpedo HIL UART bridge

ROS 2의 BlueROV/어뢰 odometry와 제어 명령을 ESP32 A에 보내고, ESP32 B에서
돌아온 어뢰 actuator와 controller status를 받는 PC 측 양방향 bridge다.

```text
ROS 2 odometry/control -> /dev/ttyACM0 -> ESP32 A
ROS 2 actuator topics  <- /dev/ttyACM0 <- ESP32 A
```

공식 wire format은 `AA 55 | MESSAGE_ID | LENGTH | PAYLOAD`다. CRC, UART
sequence와 timestamp는 사용하지 않으며 multi-byte 값은 fixed-point
big-endian이다. 세부 ID와 payload는
[`src/TORPEDO_HIL_AI_IMPLEMENTATION_SPEC.md`](src/TORPEDO_HIL_AI_IMPLEMENTATION_SPEC.md)를
따른다.

## 빌드와 실행

```bash
source /opt/ros/jazzy/setup.bash
cd /home/user/manta_ws
colcon build --packages-select esp32_bridge --symlink-install
source install/setup.bash
ros2 run esp32_bridge esp32_ros_uart_tx_node
```

기본 serial 장치는 `/dev/ttyACM0`, host baud parameter는 `921600`이다.

```bash
ros2 run esp32_bridge esp32_ros_uart_tx_node --ros-args \
  -p device:=/dev/ttyACM1 \
  -p control.armed:=true \
  -p control.mode:=1 \
  -p control.target_thrust:=500
```

Control command는 20 Hz로 반복된다.

- `control.armed`: `false`/`true`
- `control.mode`: `0=None`, `1=Simple`, `2=PNG`
- `control.target_thrust`: `0..1000`

구독 토픽:

- `topics.bluerov_odometry` (기본 `/model/bluerov2/odometry`)
- `topics.torpedo_odometry` (기본 `/torpedo/state/odometry`)

출력 토픽:

- `/torpedo/actuators/thruster/command`
- `/torpedo/actuators/fins/top/command`
- `/torpedo/actuators/fins/bottom/command`
- `/torpedo/actuators/fins/left/command`
- `/torpedo/actuators/fins/right/command`

유효한 actuator UART frame이 200 ms 동안 들어오지 않으면 다섯 출력 모두에
안전값 `0.0`을 publish한다.
