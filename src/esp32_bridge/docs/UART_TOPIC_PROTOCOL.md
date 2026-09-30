# ROS 토픽 → ESP32 UART 프로토콜

## 1. 적용 범위

이 문서는 PC의 ROS 2 노드가 ESP32로 상태 토픽을 단방향 전송하는 규칙의
기준 문서다. 기존의
`AA 55 | VER | TYPE | FLAGS | SEQ | LEN | TIME | PAYLOAD | CRC16` 초안은
사용하지 않는다.

현재 단계의 프레임은 **헤더와 고정소수점 데이터만** 포함한다.

```text
AA 55 | TOPIC_ID | DATA_LENGTH | DATA
```

## 2. 공통 프레임

| Offset | 크기 | 이름 | 규칙 |
| ---: | ---: | --- | --- |
| 0 | 2 | Sync | 항상 `0xAA 0x55` |
| 2 | 1 | Topic ID | 아래 메시지 표의 ID |
| 3 | 1 | Data length | DATA의 byte 수 |
| 4 | N | Data | Topic ID별 고정 길이 payload |

`AA 55`, Topic ID, Data length까지 4 byte를 헤더로 정의한다. Version, flags,
sequence, timestamp와 CRC는 넣지 않는다. 수신기는 Sync를 찾은 뒤 Topic ID와
그 ID에 정해진 길이가 일치할 때만 프레임을 받아야 한다.

모든 다중 byte 정수는 **big-endian(MSB first)** 이다. signed 정수는 2의 보수다.
C/C++ 구조체 메모리를 그대로 전송하지 않고 byte 단위로 직렬화한다.

## 3. 고정소수점 공통 규칙

```text
송신 raw = round(물리값 / LSB 단위)
수신 물리값 = raw * LSB 단위
```

| 용도 | Wire type | LSB 단위 | 표현 범위 |
| --- | --- | ---: | ---: |
| 위치·깊이 | `int32` | 0.001 m | -2,147,483.648 ~ 2,147,483.647 m |
| 선속도 | `int16` | 0.001 m/s | -32.768 ~ 32.767 m/s |
| 각속도 | `int16` | 0.001 rad/s | -32.768 ~ 32.767 rad/s |
| 선가속도 | `int16` | 0.001 m/s² | -32.768 ~ 32.767 m/s² |
| quaternion | `int16` Q14 | 1/16384 | -2.0 ~ 1.99993896484375 |
| 압력 | `uint32` | 1 Pa | 0 ~ 4,294,967,295 Pa |

송신 값은 가장 가까운 정수로 반올림한다. NaN, Inf 또는 wire type의 표현 범위를
벗어난 값이 하나라도 있으면 해당 프레임 전체를 보내지 않는다. saturation이나
wrap-around는 허용하지 않는다.

좌표, 축 방향과 quaternion 순서는 ROS 메시지 값을 그대로 따른다. 좌표계 ID는
wire에 싣지 않으므로 송수신 시스템 설정에서 동일한 좌표계를 사용해야 한다.

## 4. Topic ID와 payload

| ID | ROS 토픽 기본값 | ROS type | DATA 길이 |
| ---: | --- | --- | ---: |
| `0x10` | `/model/bluerov2/odometry` | `nav_msgs/msg/Odometry` | 32 |
| `0x11` | `/model/bluerov2/imu` | `sensor_msgs/msg/Imu` | 20 |
| `0x12` | `/model/bluerov2/pressure` | `sensor_msgs/msg/FluidPressure` | 4 |
| `0x13` | `/model/bluerov2/Pressure_depth` | `geometry_msgs/msg/PointStamped` | 4 |
| `0x14` | `/dvl/velocity` | `dave_interfaces/msg/DVL` | 6 |
| `0x20` | `/torpedo/state/odometry` | `nav_msgs/msg/Odometry` | 32 |
| `0x21` | `/mission/target_position` | `geometry_msgs/msg/PointStamped` | 12 |

### 4.1 Odometry (`0x10`, `0x20`)

| DATA offset | 크기 | 필드 | 인코딩 |
| ---: | ---: | --- | --- |
| 0 | 4 | position.x | `int32`, 0.001 m |
| 4 | 4 | position.y | `int32`, 0.001 m |
| 8 | 4 | position.z | `int32`, 0.001 m |
| 12 | 2 | orientation.x | `int16`, Q14 |
| 14 | 2 | orientation.y | `int16`, Q14 |
| 16 | 2 | orientation.z | `int16`, Q14 |
| 18 | 2 | orientation.w | `int16`, Q14 |
| 20 | 2 | linear.x | `int16`, 0.001 m/s |
| 22 | 2 | linear.y | `int16`, 0.001 m/s |
| 24 | 2 | linear.z | `int16`, 0.001 m/s |
| 26 | 2 | angular.x | `int16`, 0.001 rad/s |
| 28 | 2 | angular.y | `int16`, 0.001 rad/s |
| 30 | 2 | angular.z | `int16`, 0.001 rad/s |

### 4.2 IMU (`0x11`)

| DATA offset | 크기 | 필드 | 인코딩 |
| ---: | ---: | --- | --- |
| 0, 2, 4, 6 | 각 2 | orientation x, y, z, w | `int16`, Q14 |
| 8, 10, 12 | 각 2 | angular velocity x, y, z | `int16`, 0.001 rad/s |
| 14, 16, 18 | 각 2 | linear acceleration x, y, z | `int16`, 0.001 m/s² |

covariance 필드는 보내지 않는다.

### 4.3 Pressure (`0x12`)

DATA offset 0에 `fluid_pressure`를 `uint32`, 1 Pa/LSB로 보낸다. variance는
보내지 않는다.

### 4.4 Depth (`0x13`)

DATA offset 0에 `point.z`를 `int32`, 0.001 m/LSB로 보낸다. 현재 DAVE 압력
플러그인은 추정 깊이를 `point.z`에 기록하고 x/y는 사용하지 않는다.

### 4.5 DVL velocity (`0x14`)

DATA offset 0, 2, 4에 `velocity.twist.linear`의 x, y, z를 각각 `int16`,
0.001 m/s/LSB로 보낸다.

### 4.6 Mission goal (`0x21`)

DATA offset 0, 4, 8에 `point.x`, `point.y`, `point.z`를 각각 `int32`,
0.001 m/LSB로 보낸다.

## 5. 수신기 검사 순서

1. byte stream에서 `AA 55`를 찾는다.
2. 다음 2 byte인 Topic ID와 Data length를 읽는다.
3. 알려진 Topic ID인지, 길이가 표에 정의된 고정 길이와 같은지 검사한다.
4. 정해진 DATA byte가 모두 들어올 때까지 기다린다.
5. big-endian 정수를 복원한 뒤 각 LSB 단위를 곱한다.
6. ID 또는 길이가 틀리면 현재 후보의 첫 `AA` 다음 byte부터 Sync를 다시 찾는다.

## 6. 의도적으로 제외한 기능과 한계

CRC가 없으므로 전기적 잡음으로 생긴 payload bit 오류를 응용 계층에서 검출할
수 없다. 현재 PC 연결인 ESP32-S3 USB Serial/JTAG 구간의 단순화를 우선한
결정이다. GPIO UART 또는 긴 케이블로 바꾸거나 오류가 관측되면 CRC16을 별도
프로토콜 버전에서 다시 추가한다.

sequence와 timestamp가 없으므로 누락·지연 측정도 이 프레임만으로는 불가능하다.
ESP32는 각 Topic ID별 마지막 수신시각을 로컬 타이머로 기록하고 timeout된
데이터를 제어에 사용하지 않아야 한다.

이 UART 프레임은 Classical CAN 프레임이 아니다. ESP32 게이트웨이는 DATA를
CAN의 8-byte 제한에 맞춰 별도의 CAN 규약으로 분할해야 한다.
