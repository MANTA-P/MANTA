# Torpedo ESP32 HIL 전환 프로젝트

> 후속 AI의 규범적 세부 구현 명세는
> `docs/current/TORPEDO_HIL_AI_IMPLEMENTATION_SPEC.md`를 우선한다.

## 1. 목적

기존의 “BlueROV 제어 로직을 ESP32로 이식” 계획을 변경하여,
`torpedo_control_v2` 제어 로직만 ESP32 B로 이식하는 새 HIL 계획을 정리한다.
확인된 코드와 아직 사용자 결정이 필요한 항목을 구분한다. 최종 CAN ID와 byte
배치는 미확정 항목을 결정한 뒤 별도 통신 명세로 고정한다.

## 2. 변경된 최종 목표

- BlueROV 물리 모델과 제어는 PC의 Gazebo/SITL 및 ROS 2에 유지한다.
- 어뢰 물리 모델은 Gazebo/SITL에 유지한다.
- PC의 어뢰 제어 계산만 ESP32 B로 옮긴다.
- ESP32 A는 PC USB/UART와 CAN 사이의 양방향 게이트웨이다.
- ESP32 B는 어뢰·표적 상태를 받아 어뢰 actuator 출력을 계산한다.
- ESP32 B 출력을 ESP32 A와 PC를 거쳐 Gazebo 어뢰에 적용한다.
- Teensy는 같은 CAN 버스에서 소나 덤프와 E-stop 실험을 수행한다.

## 3. 장치 명칭과 역할

### PC: Gazebo/SITL 및 ROS 2

- `/torpedo/state/odometry`와 `/model/bluerov2/odometry`를 구독한다.
- 필요한 필드를 encode해 ESP32 A로 전송한다.
- ESP32 A에서 어뢰 actuator 명령을 돌려받아 ROS 토픽에 publish한다.
- HIL 모드에서는 PC의 `torpedo_control_node_v2`가 같은 actuator 토픽을
  동시에 publish하지 않도록 launch에서 제외한다.

### ESP32 A: PC-CAN 게이트웨이

- PC USB/UART byte stream 수신 및 packet 검증
- PC payload를 CAN 8-byte 프레임으로 분할해 ESP32 B로 전송
- ESP32 B의 actuator CAN 프레임 수신 및 재조립
- PC packet으로 encode해 USB/UART로 반환
- 제어 계산과 좌표 변환은 수행하지 않음

현재 전달받은 진행 상태:

- byte stream 수신 완료
- 기본 packet 경계 parsing 완료
- CRC와 이후 검증·CAN 변환·역방향 전송은 미완성

### ESP32 B: 어뢰 제어기

- 어뢰와 BlueROV 상태 CAN 메시지 수신 및 재조립
- 상태 유효성, sequence와 timeout 검사
- `torpedo_control_v2`의 guidance, roll controller와 mixer 실행
- thrust 1개와 fin 4개를 CAN으로 ESP32 A에 반환
- E-stop 또는 필수 상태 timeout 시 안전 출력 적용

### Teensy 4.1: CAN 시험 노드

- CAN1: 버스 포화용 가상 소나 덤프
- CAN2: 물리 스위치 기반 E-stop
- E-stop ID를 최고/최저 우선순위로 바꿔 지연과 starvation 비교

## 4. 전체 데이터 흐름

~~~text
Gazebo/SITL
  | torpedo odometry + BlueROV odometry
  v
PC ROS 2 bridge
  | USB/UART packet
  v
ESP32 A: USB/UART-CAN gateway
  | Classical CAN 500 kbit/s
  v
ESP32 B: torpedo_control_v2
  | thrust + fin top/bottom/left/right
  v
ESP32 A -> USB/UART -> PC ROS 2 bridge -> Gazebo actuator

Teensy CAN1/CAN2 --------------------------> 같은 CAN 버스
~~~

## 5. 실제 torpedo_control_v2 분석 결과

### 제어 입력

현재 `SensorData`는 두 odometry를 사용한다.

| 입력 | ROS 토픽 | 타입 | 관측 주기 |
| --- | --- | --- | ---: |
| 어뢰 자체 상태 | `/torpedo/state/odometry` | `nav_msgs/msg/Odometry` | 약 100 Hz |
| 추적 대상 BlueROV | `/model/bluerov2/odometry` | `nav_msgs/msg/Odometry` | 약 100 Hz |

| 기능 | 실제 필요한 필드 |
| --- | --- |
| Simple Tracking | 양쪽 위치, 어뢰 quaternion |
| PNG | 양쪽 위치·선속도·quaternion |
| Roll 제어 | 어뢰 quaternion, 어뢰 angular velocity Y |
| 상태 검사 | 각 입력의 valid 및 마지막 수신 시각 |

quaternion이 필요한 이유는 현재 코드가 body/world 좌표 사이에서 선속도와
상대 위치를 회전하기 때문이다. 위치와 속도만 보내면 같은 제어가 불가능하다.

### 제어 주기와 timeout

| 항목 | 현재 코드값 |
| --- | ---: |
| 제어 주기 | 20 Hz, 50 ms |
| Torpedo odometry timeout | 30 ms |
| BlueROV odometry timeout | 30 ms |

100 Hz 입력은 SITL 내부에서 30 ms를 만족하지만 CAN 포화 시험에서는 너무
짧을 수 있으므로 통합 측정 후 확정한다.

### 제어 모드

현재 모드는 None, Keyboard, Simple Tracking, PNG다. PC에서는 Linux keyboard로
모드와 thrust up/down/stop을 입력한다. ESP32 B에는 keyboard가 없으므로 HIL
명령 경로를 새로 정해야 한다.

### 기본 파라미터

| 파라미터 | 값 |
| --- | ---: |
| update rate | 20 Hz |
| thrust step/min/max | 100 / 0 / 1000 |
| pitch/yaw step | 0.05 rad |
| fin limit | 0.50 rad |
| tracking pitch/yaw Kp | 0.80 / 0.80 |
| roll Kp/Kd/limit | 0.60 / 0.15 / 0.10 rad |
| PNG navigation constant | 3.0 |
| PNG acceleration-to-fin gain | 1.0 |

ESP32 이식 시 float/double 차이를 고려해 동일 입력에 대한 PC/ESP32 출력을
허용 오차 기준으로 비교한다.

### 제어 출력

| 출력 | ROS 토픽 |
| --- | --- |
| thrust | `/torpedo/actuators/thruster/command` |
| fin_top | `/torpedo/actuators/fins/top/command` |
| fin_bottom | `/torpedo/actuators/fins/bottom/command` |
| fin_left | `/torpedo/actuators/fins/left/command` |
| fin_right | `/torpedo/actuators/fins/right/command` |

Gazebo bridge가 공통 thrust 하나를 전·후방 propeller joint 두 곳에 fan-out한다.
ESP32 B가 추진기 두 개를 따로 계산하는 구조가 아니다.

## 6. 통신 기본 조건

| 항목 | 기준 |
| --- | --- |
| CAN | Classical CAN 2.0A, 500 kbit/s |
| ID/DLC | 표준 11-bit, 최대 8 byte |
| byte order | big-endian |
| transceiver | 각 노드 SN65HVD230 계열 |
| PC 장치 | `/dev/ttyACM0` |

ESP32 A, ESP32 B와 Teensy는 모두 500 kbit/s를 사용한다.

### 확정 핀

| 노드 | 기능 | 핀 |
| --- | --- | --- |
| ESP32 A | TWAI TX / RX | GPIO17 / GPIO18 |
| ESP32 B | TWAI TX / RX | GPIO17 / GPIO18 |
| Teensy CAN1 | Dump TX / RX | 22 / 23 |
| Teensy CAN2 | E-stop TX / RX | 1 / 0 |
| Teensy | 택트 스위치 | Digital 2, INPUT_PULLUP, active LOW |

택트 스위치는 Teensy pin 2와 GND 사이에 연결한다.

## 7. 필요한 메시지

### PC -> ESP32 A -> ESP32 B

| 메시지 | 주기/조건 | 내용 |
| --- | --- | --- |
| Torpedo odometry | 목표 100 Hz | position, quaternion, linear/angular velocity |
| BlueROV target odometry | 목표 100 Hz | position, quaternion, linear/angular velocity |
| Control command | 변경 + 반복 | mode, thrust up/down/stop 또는 목표 thrust |
| Configuration | 시작/변경 | 제어 파라미터 또는 preset/version |
| Heartbeat | 주기 | PC와 ESP32 A 연결 상태 |

### ESP32 B -> ESP32 A -> PC

| 메시지 | 주기/조건 | 내용 |
| --- | --- | --- |
| Torpedo actuator | 20 Hz | thrust, fin top/bottom/left/right |
| Controller status | 저주기 | mode, input valid, timeout, E-stop |
| Communication stats | 저주기 | sequence gap, drop, CAN 오류 |
| Heartbeat | 주기 | ESP32 B 생존 상태 |

### Teensy -> ESP32 B

| 메시지 | 조건 | 내용 |
| --- | --- | --- |
| E-stop | 이벤트 + 반복 | active/released, sequence, 시험 모드 |
| Sonar dump | 연속 | 버스 포화용 latest-only 데이터 |

### 확정 CAN ID

| CAN ID | 메시지 | 송신 -> 수신 |
| ---: | --- | --- |
| 0x001 | E-stop | Teensy -> ESP32 B |
| 0x010 | Mode + target thrust | ESP32 A -> ESP32 B |
| 0x020 | Controller configuration, 추후 확장 | ESP32 A -> ESP32 B |
| 0x021 | Configuration ACK, 추후 확장 | ESP32 B -> ESP32 A |
| 0x110 | BlueROV odometry fragment | ESP32 A -> ESP32 B |
| 0x120 | Torpedo odometry fragment | ESP32 A -> ESP32 B |
| 0x180 | Thrust + top/bottom fin | ESP32 B -> ESP32 A |
| 0x181 | Left/right fin + controller state | ESP32 B -> ESP32 A |
| 0x1F0 | Controller heartbeat/status | ESP32 B -> ESP32 A |
| 0x700 | Sonar dump | Teensy -> bus |

ID 0x001은 정상 운용에서 E-stop 전용이다. 우선순위 역전 시험에서만 dump와
E-stop의 0x001/0x700을 교환한다. 서로 다른 노드가 동시에 다른 payload를
0x001로 송신하지 않는다.

## 8. 기존 구현에서 바꿀 부분

### PC esp32_bridge

현재 `hil_bridge_node.cpp`는 이전 BlueROV-HIL 기준이다.

- 불필요한 IMU, 압력, 깊이, DVL과 임무 목표까지 송신한다.
- 어뢰 payload는 position과 linear velocity만 24 byte로 보내며 quaternion과
  angular velocity가 빠져 있다.
- 역방향 0x80은 float 6개를 BlueROV 추진기 6개에 publish한다.

새 계획에서는:

- 입력을 Torpedo/BlueROV odometry와 control command 중심으로 축소
- 두 odometry에 필요한 quaternion과 angular velocity 포함
- 역방향 출력을 어뢰 thrust 1개 + fin 4개로 변경
- PC controller와 actuator publisher 중복 방지

### ESP32 A

- 현재 parser를 보존하고 최종 UART 규약에 맞춰 검증 완성
- odometry/control command CAN 분할 송신
- actuator/status CAN 재조립 및 PC 반환
- latest-only, timeout, 오류 통계와 bus-off 복구

### ESP32 B

- ROS 및 Linux keyboard 의존성을 제거한 순수 control core 이식
- CAN transport와 control core 분리
- 두 odometry를 각각 최신값으로 보관
- 20 Hz task에서 일관된 snapshot으로 제어
- E-stop/timeout 시 zero output

## 9. 중요한 프로토콜 충돌

저장소에는 호환되지 않는 UART 규약 두 개가 공존한다.

| 항목 | 현재 PC C++ 코드 | 최근 루트 프로토콜 문서 |
| --- | --- | --- |
| header | 13 byte | AA 55 + ID + Length |
| CRC | CRC-16/CCITT-FALSE | 없음 |
| sequence/timestamp | 있음 | UART에는 없음 |
| payload | float32 big-endian | 고정소수점 big-endian |
| baud parameter | 115200 | 921600 |
| 방향 | 양방향 골격 | PC->B 단방향 |

둘은 wire compatible하지 않다. 사용자 결정에 따라 **최근 루트 프로토콜
문서의 단순 규약을 공식 기준으로 사용한다.**

~~~text
AA 55 | TOPIC_ID | DATA_LENGTH | DATA
~~~

- UART CRC, timestamp와 sequence는 사용하지 않는다.
- payload는 고정소수점 big-endian을 사용한다.
- PC C++ 코드는 이 규약에 맞게 변경한다.
- CAN fragment에는 Topic별 sequence와 fragment index를 사용한다.
- 양방향 actuator 반환 규약은 기존 단방향 문서에 추가해야 한다.

최근 문서의 full odometry 32 byte는 ESP32 B 입력에 적합하지만, 현재 PC 코드의
Torpedo payload 24 byte에는 quaternion과 angular velocity가 없다.

## 10. 데이터율과 CAN 부하 추정

odometry 하나를 32 byte, CAN chunk를 6 byte로 보내면 각각 6 frame이다.

~~~text
2 odometry x 6 frame x 100 Hz = 1,200 CAN frame/s
~~~

500 kbit/s의 DLC 8 표준 frame을 약 222~270 us로 보면 두 odometry가 대략
27~32%의 버스 시간을 사용한다. 실제 bit stuffing, frame 간격, 다른 메시지와
오류 재전송을 포함해 실측한다. Teensy dump는 남은 대역폭을 채운다.

## 11. 안전 기준안

- E-stop active 시 mode와 관계없이 출력 5개를 0으로 만든다.
- E-stop은 택트 스위치를 누르는 동안만 active이며 latch하지 않는다.
- Teensy는 누르는 순간 active를 3회 보내고, 누르는 동안 20 ms마다 반복한다.
- 스위치를 놓으면 release를 즉시 3회 전송한다.
- ESP32 B는 release 수신 후 두 odometry와 PC mode/thrust 명령이 모두 유효할
  때만 제어를 재개한다.
- Torpedo 또는 target odometry timeout 시 모든 출력을 0으로 만든다.
- NaN, infinity, 비정상 quaternion과 범위 밖 값은 invalid 처리한다.
- PC/A/B heartbeat timeout을 구분해 기록한다.
- E-stop과 안전 상태는 dump보다 높은 CAN 우선순위를 사용한다.
- 우선순위 역전 시험은 실제 위험한 구동부와 분리한다.

## 12. 구현 순서

1. 최근 단순 UART 규약을 Torpedo HIL 양방향으로 확장
2. Torpedo HIL CAN communication matrix 확정
3. PC 두 odometry 및 control command encode
4. ESP32 A 검증과 CAN fragmentation
5. ESP32 B reassembly 및 decode
6. PC와 ESP32 control core 동일 입력 비교
7. ESP32 B actuator CAN 송신
8. ESP32 A 재조립 및 PC 반환
9. PC 어뢰 actuator 5개 publish
10. E-stop 및 timeout 안전 동작
11. closed-loop HIL 시험
12. Teensy 포화·우선순위 역전 시험

제어 파라미터는 v1에서 ESP32 B의 현재 기본값으로 compile-time 고정한다.
PC는 mode와 target thrust만 ID 0x010으로 전달한다. 실시간 튜닝이 필요해질
때만 ID 0x020/0x021 설정·ACK 프로토콜을 활성화한다.

## 13. 완료 조건

- PC `torpedo_control_node_v2` 없이 ESP32 B가 20 Hz 제어
- 두 odometry 유효 수신과 timeout 처리
- PC/ESP32 control core 결과가 정한 허용 오차 이내
- 출력 5개가 PC로 반환되어 Gazebo에 적용
- E-stop 또는 통신 손실 시 안전 출력
- 모든 CAN 노드가 500 kbit/s로 동작
- sequence, drop, packet 오류, CAN error와 bus-off 통계 제공
- 정상 부하 closed-loop 추적 확인
- 포화 상태에서 E-stop 우선순위별 지연 결과 기록

## 14. 사용자 결정 필요

확정된 결정:

- PC-ESP32 A는 최근 단순 UART 규약을 사용한다.
- ESP32 B에 Simple Tracking과 PNG를 모두 이식한다.
- PC가 ESP32 B에 mode와 목표 thrust를 전달한다.
- Teensy 설정은 USB Serial 문자열 명령으로 runtime 변경한다.
- E-stop은 택트 스위치를 누르는 동안만 active이며 CAN ID 0x001을 사용한다.
- 제어 파라미터는 v1에서 ESP32 B에 compile-time 고정한다.
- ESP32 A/B TWAI는 GPIO17/GPIO18과 SN65HVD230을 사용한다.
- CAN ID는 7장의 표를 기준으로 한다.

남은 결정:

1. 30 ms sensor timeout의 최종값과 timeout 후 재가동 정책
2. ID 0x010, 0x180, 0x181의 정확한 byte 배치와 scale

위 항목이 확정되기 전에는 wire format이나 안전 정책을 임의로 구현하지 않는다.

## 15. 관련 파일

- `src/torpedo_control_v2`
- `src/esp32_bridge`
- `docs/reference/UART_CAN_BASE_PROTOCOL.md`
- `docs/handoff/ESP32_A_GATEWAY_HANDOFF.md`
- `docs/handoff/TEENSY_LOAD_ESTOP_HANDOFF.md`
- `docs/reference/ROS_TOPIC_INVENTORY.md`
