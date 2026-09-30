# Torpedo ESP32 HIL — AI 구현 명세

> 후속 AI용 최우선 구현 문서. 기존 BlueROV-HIL 문서와 충돌하면 본 문서를 따른다.

## 1. 작업 규칙

1. 시작 전에 본 문서, `docs/current/TORPEDO_HIL_PROJECT_OVERVIEW.md`, 현재 코드,
   README, 빌드 설정과 `git status`를 읽는다.
2. 기존 parser, TWAI, FlexCAN, control core와 사용자 변경을 보존한다.
3. 사소한 이름/구조 차이는 기존 코드에 맞추되 다음 중요 차이는 근거·영향·
   선택지를 사용자에게 설명하고 확인한다: bitrate/GPIO/배선, ID/payload/endian,
   actuator 부호·단위, E-stop 안전값, 대규모 재작성.
4. 확인 가능한 정보는 먼저 저장소에서 찾고 추측하지 않는다.
5. 구현 후 단위시험, build와 가능한 실기시험을 수행하고 결과를 문서화한다.

## 2. 목표와 역할

BlueROV 제어는 PC에 유지하고 `torpedo_control_v2`만 ESP32 B로 이식한다.

~~~text
Gazebo/SITL
 -> PC ROS bridge
 -> USB Serial/JTAG
 -> ESP32 A (양방향 USB-CAN gateway)
 -> Classical CAN 500 kbit/s
 -> ESP32 B (Torpedo controller)
 -> thrust + four fins
 -> ESP32 A -> PC -> Gazebo actuator

Teensy CAN1: sonar dump ---+
Teensy CAN2: tact E-stop --+-> 동일 CAN bus
~~~

- PC: 두 odometry encode, mode/thrust 송신, actuator 5개 ROS publish
- ESP32 A: packet 검증, CAN 분할/재조립, 양방향 전달. 제어 계산 금지
- ESP32 B: Simple Tracking/PNG/roll/mixer, 안전 상태, actuator 반환
- Teensy: dump 포화, E-stop, USB Serial 실험 설정

## 3. 현재 상태

- PC 기준 제어: `src/torpedo_control_v2`
- PC bridge: `src/esp32_bridge`. 현재 긴 header+CRC 및 BlueROV 출력 6개라 변경 필요
- ESP32 A: byte 수신과 packet 경계 parser까지만 완료. 이후 검증/CAN/역방향 미완성
- Teensy: `teensy_can_load_estop.ino`, `config.h`에 switch, 두 CAN, latest-only
  dump, 반복 E-stop, callback과 통계 구현됨

기존 코드를 처음부터 다시 만들지 않는다.

## 4. 하드웨어

| 노드 | 핀 | 장치 |
| --- | --- | --- |
| ESP32 A | TWAI TX=17, RX=18 | SN65HVD230 |
| ESP32 B | TWAI TX=17, RX=18 | SN65HVD230 |
| Teensy CAN1 | TX=22, RX=23 | dump용 SN65HVD230 |
| Teensy CAN2 | TX=1, RX=0 | E-stop용 SN65HVD230 |
| Teensy switch | Digital 2-GND | INPUT_PULLUP, active LOW |

모두 Classical CAN 2.0A, 500 kbit/s, 11-bit Standard Data Frame을 사용한다.
CANH/CANL/GND 공통, 120 Ω은 물리 양 끝만, 전원 OFF 저항 약 60 Ω을 확인한다.

## 5. ESP32 B 제어 요구

### 입력

| Topic | Hz | 필드 |
| --- | ---: | --- |
| `/torpedo/state/odometry` | 100 | position, quaternion, linear/angular velocity |
| `/model/bluerov2/odometry` | 100 | position, quaternion, linear/angular velocity |
| Control command | 20 | armed, Simple/PNG mode, target thrust |

PNG는 양쪽 position/velocity/quaternion, roll은 Torpedo quaternion/angular Y가
필요하다. 필드를 제거하지 않는다.

### 출력

20 Hz로 thrust(0..1000)와 fin top/bottom/left/right(각 ±0.50 rad)를 만든다.
ROS topic은 `/torpedo/actuators/thruster/command` 및
`/torpedo/actuators/fins/{top,bottom,left,right}/command`다. 공통 thrust는
Gazebo bridge가 전·후 propeller에 fan-out한다.

### 고정 파라미터

v1은 현재 PC 기본값을 ESP32 B compile-time으로 사용한다:
update rate=20 Hz, thrust step/min/max=100/0/1000, fin limit=0.50 rad,
tracking Kp=0.80/0.80, roll Kp/Kd/limit=0.60/0.15/0.10,
PNG N=3.0, acceleration-to-fin gain=1.0.

Keyboard mode는 이식하지 않는다. Simple Tracking과 PNG를 모두 이식하고 PC가
mode와 target thrust를 보낸다.

## 6. 공식 UART 규약

최근 단순 규약으로 확정됐다.

~~~text
AA 55 | MESSAGE_ID | LENGTH | PAYLOAD
~~~

- CRC, UART sequence, timestamp 없음
- multi-byte big-endian, signed는 2의 보수
- ID별 고정 length 불일치 폐기
- payload 안 AA 55는 data
- frame 시작 후 50 ms 미완성 시 reset, 다음 sync 재탐색
- `/dev/ttyACM0` USB Serial/JTAG, host baud parameter 921600
- binary stream과 debug 문자열을 섞지 않음

| UART ID | 방향 | Length | 의미 |
| ---: | --- | ---: | --- |
| 0x10 | PC→A | 32 | BlueROV full odometry |
| 0x20 | PC→A | 32 | Torpedo full odometry |
| 0x30 | PC→A | 8 | Control command |
| 0x80 | A→PC | 12 | Torpedo actuator |
| 0x81 | A→PC | 8 | Controller status |

## 7. 자료형과 payload

| 값 | wire |
| --- | --- |
| position | int32 BE, 0.001 m/LSB |
| quaternion | int16 BE Q14 |
| linear velocity | int16 BE, 0.001 m/s/LSB |
| angular velocity | int16 BE, 0.001 rad/s/LSB |
| thrust | uint16 BE, 1 unit/LSB |
| fin | int16 BE, 0.001 rad/LSB |

32-byte odometry:

~~~text
0..11 position xyz       3*int32
12..19 quaternion xyzw   4*int16 Q14
20..25 linear xyz        3*int16
26..31 angular xyz       3*int16
~~~

NaN/Infinity를 거부하고 encode는 반올림+saturation한다. decode 후 quaternion
norm을 검사한다. C/C++ 구조체 memory layout을 wire에 사용하지 않는다.

## 8. CAN matrix

| ID | 방향 | 의미 |
| ---: | --- | --- |
| 0x001 | Teensy→B | E-stop(normal) |
| 0x005 | Teensy→B | experiment mode announcement |
| 0x010 | A→B | control command |
| 0x110 | A→B | BlueROV odometry fragments |
| 0x120 | A→B | Torpedo odometry fragments |
| 0x180 | B→A | actuator A |
| 0x181 | B→A | actuator B |
| 0x1F0 | B→A | status/heartbeat |
| 0x700 | Teensy→bus | dump(normal) |

### Odometry CAN frame

모두 DLC 8: byte0=topic sequence, byte1=fragment index 0..5, byte2..7=6-byte
chunk. 마지막 padding은 0. B는 bitmap으로 순서 무관 재조립하고, 새 sequence면
미완성 이전 값을 폐기한다. timeout 50 ms.

### Control 0x010 / UART 0x30

~~~text
byte0 version=1
byte1 sequence
byte2 armed (0/1)
byte3 mode (0=None, 1=Simple, 2=PNG)
byte4..5 target thrust uint16 BE, 0..1000
byte6..7 reserved=0
~~~

PC가 20 Hz 반복 송신한다.

### Actuator 0x180

~~~text
byte0 sequence
byte1 state: 0 DISARMED, 1 RUNNING, 2 TIMEOUT, 3 ESTOP, 4 FAULT
byte2..3 thrust uint16 BE
byte4..5 fin top int16 BE
byte6..7 fin bottom int16 BE
~~~

### Actuator 0x181

~~~text
byte0 same sequence
byte1 active mode
byte2..3 fin left int16 BE
byte4..5 fin right int16 BE
byte6 flags: bit0 torpedo valid, bit1 target valid, bit2 command valid,
             bit3 E-stop, bit4 CAN error, bit5 bus-off
byte7 reserved=0
~~~

B는 20 Hz마다 두 frame을 연속 송신한다. A는 같은 sequence pair만 50 ms 안에
결합한다. UART 0x80은 sequence, state, thrust, top, bottom, left, right 순서의
12-byte payload다.

### Status 0x1F0 / UART 0x81

10 Hz:

~~~text
version, heartbeat_seq, state, mode, flags, last_error,
uptime_seconds_low16_BE
~~~

## 9. Teensy

택트 스위치는 debounce 50 ms, latch 없음이다.

- press 즉시 active 3회(1 ms 간격)
- 누르는 동안 active를 20 ms마다 반복
- release 즉시 release 3회
- B는 명시적 release 전까지 active 유지

E-stop은 dump 오인 방지 signature를 사용한다.

~~~text
byte0 E5, byte1 7A, byte2 state(0/1), byte3 mode(normal=0/inverted=1),
byte4..5 sequence BE, byte6..7 event_time_ms_low16 BE
~~~

Normal: E-stop=0x001, dump=0x700. Inverted: E-stop=0x700, dump=0x001.
Mode 변경 전후 Teensy가 0x005로 `E5 7A 01 mode estop_id_BE dump_id_BE`를 3회
보내 B가 예상 ID를 갱신하게 한다. active 중 mode 변경은 거부한다.

USB Serial runtime 명령:

~~~text
MODE NORMAL | MODE INVERTED
DUMP ON | DUMP OFF | DUMP PERIOD <us>
CONFIG | STATS | STATS RESET | HELP
~~~

고정 char buffer를 사용한다. 기본은 NORMAL/DUMP OFF/200 us이며 EEPROM 자동
저장은 하지 않는다. mode 변경은 dump stop→mailbox 완료→pending 폐기→
announcement→ID 전환→announcement→dump 재개 순서다. 적용 후 OK, 오류는 ERR.

## 10. 안전과 timeout

| 감시 | timeout |
| --- | ---: |
| 각 complete odometry | 50 ms |
| control command | 200 ms |
| PC의 actuator 수신 | 200 ms |
| B heartbeat | 500 ms |

~~~text
DISARMED -> armed + valid mode + inputs valid -> RUNNING
RUNNING -> timeout / numeric fault / E-stop
TIMEOUT -> zero; 각 odometry 5회 정상 + armed 0→1 edge 후 RUNNING
FAULT -> zero; 원인 제거 + armed=0 후 DISARMED
ESTOP -> zero; release + inputs/command valid 후 armed에 따라 복귀
~~~

안전 출력은 thrust=0, fins=0 rad다. PC도 UART actuator를 200 ms 못 받으면
zero를 Gazebo에 publish한다. 오류 frame 한 개는 폐기하고 마지막 완성 정상
데이터 age가 timeout을 넘을 때 failsafe로 전환한다.

## 11. 구현 체크리스트

### PC

- [ ] 불필요한 IMU/pressure/depth/DVL/goal 송신 제거
- [ ] 두 odometry 32-byte fixed-point encode
- [ ] armed/mode/target thrust 20 Hz 송신
- [ ] 4-byte UART header로 전환
- [ ] 0x80 decode 및 actuator 5개 publish
- [ ] 0x81 상태 처리와 200 ms output watchdog
- [ ] HIL launch에서 PC Torpedo controller 제외

### ESP32 A

- [ ] 기존 parser에 known ID/fixed length/50 ms timeout
- [ ] UART 0x10/20 → CAN 0x110/120 분할
- [ ] UART 0x30 → CAN 0x010
- [ ] CAN 0x180/181 → UART 0x80
- [ ] CAN 0x1F0 → UART 0x81
- [ ] queue/drop/sequence/bus-off 통계

### ESP32 B

- [ ] 두 odometry reassembly/decode
- [ ] command validation/lease
- [ ] control core 이식 및 PC 결과 비교
- [ ] 20 Hz 상태 머신/actuator pair
- [ ] 10 Hz heartbeat
- [ ] E-stop signature/mode validation
- [ ] timeout/numeric/bus-off zero output

### Teensy

- [ ] RuntimeConfig와 USB 명령
- [ ] boot dump OFF
- [ ] held-active 20 ms 반복
- [ ] signature payload와 0x005 announcement
- [ ] 기존 latest-only/callback/statistics 보존

## 12. 검증과 완료 조건

필수 시험:

1. fixed-point endian/경계/음수/Q14 vector
2. UART partial/combined/garbage/length/timeout
3. fragment 누락/중복/역순/sequence/padding
4. PC와 ESP32 control core 동일 입력 비교
5. PC→A→B 및 B→A→PC 실기
6. PC controller 없이 Gazebo closed loop
7. cable 분리, malformed, timeout, bus-off
8. tact active/release와 zero output
9. normal/inverted dump에서 E-stop 각 30회 이상

최종 완료:

- 모든 byte vector가 명세와 일치
- Simple/PNG 모두 PC 명령으로 실행
- actuator 5개가 20 Hz로 Gazebo 적용
- 모든 timeout/E-stop/watchdog 동작
- 500 kbit/s 지속·포화 시험 통과
- 최소/평균/최대/p95 지연, starvation/drop/error/bus-off 기록
- 기존 사용자 작업 보존 및 빌드/시험 결과 문서화

## 13. 참고 우선순위

1. 본 문서
2. `docs/current/TORPEDO_HIL_PROJECT_OVERVIEW.md`
3. `docs/reference/UART_CAN_BASE_PROTOCOL.md`
4. `src/torpedo_control_v2`
5. ESP32/Teensy 인수인계 문서

기존 BlueROV-HIL 표는 새 구현 근거로 사용하지 않는다.
