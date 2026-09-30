# ROS UART → ESP32-S3 → CAN 브릿지 설계서

> 문서 상태: v1.0 초안  
> 기준일: 2026-09-12  
> 용도: 사람이 전체 구성과 송신·수신 펜웨어의 역할을 이해하기 위한 문서

## 1. 목표

PC의 ROS 2 노드가 여러 센서 토픽을 ESP32-S3 보드 A에 USB
Serial/JTAG로 전송하고, 보드 A가 받은 프레임을 Classical CAN으로
분할해 ESP32-S3 보드 B에 전달한다. 보드 B는 CAN 프레임을
재조립하고 ROS 데이터의 물리값을 복원한다.

```mermaid
flowchart LR
    ROS[ROS 2 topics] --> CPP[esp32_ros_uart_tx_node]
    CPP -->|USB Serial/JTAG<br/>AA 55 ID LEN DATA| A[ESP32-S3 A<br/>Gateway]
    A --> TA[CAN Transceiver A]
    TA <-->|500 kbit/s CAN bus| TB[CAN Transceiver B]
    TB --> B[ESP32-S3 B<br/>Receiver]
    B --> OUT[Decoded values / application]
```

### 보드별 역할

| 노드 | 역할 |
|---|---|
| PC | ROS 메시지를 고정소수점 UART 프레임으로 변환 |
| ESP32-S3 A | UART stream parsing, payload 검증, CAN fragmentation/송신 |
| ESP32-S3 B | CAN 수신/검증, payload reassembly, 고정소수점 decode |

이 버전은 **PC → A → B 단방향 전송**만 정의한다. 보드 B에서
ROS PC로 보내는 응답 프로토콜은 범위에 포함하지 않는다.

## 2. 기존 코드와의 관계

PC 송신 코드는 다음 세 파일을 기준으로 한다.

- `ros_uart_tx_node.cpp`: ROS 2 토픽 구독 및 payload 생성
- `packet_codec.cpp`: 고정소수점 big-endian 인코딩, UART 프레임 생성
- `uart_port.cpp`: `/dev/ttyACM0`, non-blocking serial I/O

기존 `CAN_CALCULATOR_PROTOCOL.md`와 `CAN_CALCULATOR_AI_SPEC.md`는 두 ESP32-S3의
TWAI 통신이 실제로 동작했다는 참고 구현이다. CAN 토픽 ID와
payload는 본 문서에서 새로 정의하며 계산기 ID `0x001`∼`0x003`을
사용하지 않는다.

## 3. 하드웨어 및 통신 기본값

| 항목 | 규격 |
|---|---|
| MCU | ESP32-S3 2대 |
| PC → 보드 A | ESP32-S3 USB Serial/JTAG CDC-ACM |
| PC 장치 기본값 | `/dev/ttyACM0` |
| host baud 파라미터 | `921600` (현 USB Serial/JTAG에서는 실제 UART clock을 결정하지 않음) |
| CAN | Classical CAN 2.0A, 11-bit Standard Data Frame |
| CAN bitrate | 500 kbit/s |
| CAN DLC | 항상 8 |
| CAN 물리 계층 | 각 보드에 3.3 V 호환 CAN transceiver 필요 |
| 종단 | bus 양 끝에만 120 Ω, 전원 OFF 시 CAN_H–CAN_L 약 60 Ω |

현재 테스트된 보드 예제의 TWAI GPIO는 TX=`GPIO17`, RX=`GPIO18`이다.
실제 구현에서는 Kconfig로 선택 가능하게 하고, 두 보드의 배선과
일치하는지 확인한다.

## 4. PC → 보드 A UART 프로토콜

### 4.1 프레임

```text
AA 55 | TOPIC_ID | DATA_LENGTH | DATA
```

| Offset | 크기 | 설명 |
|---:|---:|---|
| 0 | 1 | Sync `0xAA` |
| 1 | 1 | Sync `0x55` |
| 2 | 1 | Topic ID |
| 3 | 1 | payload byte 길이 |
| 4 | N | topic payload |

- 모든 다중 byte 정수는 **big-endian**이다.
- signed 정수는 2의 보수다.
- UART 프레임에 CRC, timestamp, sequence는 없다.
- 보드 A는 ID와 ID별 고정 길이가 모두 일치할 때만 수신한다.

### 4.2 Topic 표

| Topic ID | 의미 | Payload 크기 | CAN ID | CAN fragment 수 |
|---:|---|---:|---:|---:|
| `0x10` | BlueROV Odometry | 32 | `0x110` | 6 |
| `0x11` | IMU | 20 | `0x111` | 4 |
| `0x12` | Pressure | 4 | `0x112` | 1 |
| `0x13` | Depth | 4 | `0x113` | 1 |
| `0x14` | DVL velocity | 6 | `0x114` | 1 |
| `0x20` | Torpedo Odometry | 32 | `0x120` | 6 |
| `0x21` | Mission goal | 12 | `0x121` | 2 |

CAN ID는 `0x100 + TOPIC_ID`로 정의한다. 이 계산식은 위의 알려진
Topic ID에만 적용하며, 알 수 없는 ID를 임의로 전달하지 않는다.

### 4.3 고정소수점 단위

| 데이터 | Wire type | 단위 |
|---|---|---:|
| 위치, 깊이 | `int32` BE | 0.001 m/LSB |
| 선속도 | `int16` BE | 0.001 m/s/LSB |
| 각속도 | `int16` BE | 0.001 rad/s/LSB |
| 선가속도 | `int16` BE | 0.001 m/s²/LSB |
| quaternion | `int16` BE, Q14 | 1/16384 per LSB |
| 압력 | `uint32` BE | 1 Pa/LSB |

Odometry는 position x/y/z, quaternion x/y/z/w, linear velocity x/y/z,
angular velocity x/y/z 순서다. IMU는 quaternion, angular velocity,
linear acceleration 순서다.

## 5. 보드 A → 보드 B CAN 분할 프로토콜

### 5.1 설계 원칙

UART payload의 바이트를 보드 A에서 float로 decode했다가 다시 encode하지
않는다. **UART에서 받은 big-endian payload를 그대로 잘라 CAN으로
전송**한다. 따라서 endian 변환은 보드 B의 최종 decode 단계에서만
일어난다.

### 5.2 CAN DATA 형식

모든 CAN 프레임은 DLC 8이며 DATA 배치는 다음과 같다.

| Byte | 필드 | 규칙 |
|---:|---|---|
| 0 | `sequence` | Topic별 `uint8`, `0`∼`255` 순환 |
| 1 | `fragment_index` | 0부터 시작 |
| 2–7 | `chunk` | 원본 UART payload의 최대 6 byte |

```text
CAN ID   = 0x100 + UART TOPIC_ID
offset   = fragment_index * 6
count    = ceil(expected_payload_size / 6)
valid    = min(6, expected_payload_size - offset)
```

마지막 fragment의 유효 chunk가 6 byte보다 짧으면 나머지 DATA byte는
`0x00`으로 padding한다. 수신부는 padding이 0이 아니면 프레임을
형식 오류로 폐기한다.

`sequence`는 Topic별로 독립적이다. 보드 A는 유효한 UART 프레임을
하나 받을 때마다 해당 Topic의 sequence를 할당하고, 다음 메시지에서
1 증가시킨다. 255 다음은 0이다. 한 payload의 모든 fragment는
같은 sequence를 사용한다.

### 5.3 예제

Pressure `101325 Pa`:

```text
UART frame: AA 55 12 04 00 01 8B CD

CAN ID:     0x112
CAN DATA:   2A 00 00 01 8B CD 00 00
            SS II [ valid chunk ] [pad]
```

DVL velocity `(1.0, -1.0, 0.5) m/s`:

```text
UART payload: 03 E8 FC 18 01 F4
CAN ID:       0x114
CAN DATA:     2B 00 03 E8 FC 18 01 F4
```

Mission goal `(1.0, -2.0, 3.5) m`:

```text
UART payload: 00 00 03 E8 FF FF F8 30 00 00 0D AC

CAN ID 0x121, fragment 0: 10 00 00 00 03 E8 FF FF
CAN ID 0x121, fragment 1: 10 01 F8 30 00 00 0D AC
```

## 6. 보드 A: UART→CAN Gateway 동작

1. USB Serial/JTAG driver와 TWAI node를 초기화한다.
2. USB byte stream에서 `AA 55`를 찾는다.
3. Topic ID와 length를 읽고 알려진 값인지 확인한다.
4. 정해진 payload가 모두 들어오면 해당 Topic의 sequence를 할당한다.
5. payload를 6 byte 단위로 나눈 CAN 프레임을 index 순서로 보낸다.
6. CAN 송신 실패 시 해당 payload의 나머지 fragment 송신을 중단한다.
7. 다음 UART 프레임은 새 sequence를 사용한다.

UART parser는 state machine으로 구현한다. payload 안의 `AA 55`는
데이터일 수 있으므로 payload를 읽는 중에는 sync 탐색을 다시 시작하지
않는다. 헤더가 틀리거나 50 ms 내에 프레임이 완성되지 않으면
현재 후보를 폐기하고 sync를 다시 찾는다.

## 7. 보드 B: CAN Receiver 동작

1. TWAI RX callback은 프레임을 즉시 FreeRTOS queue에 복사하고 반환한다.
2. 수신 task가 Standard Data Frame, 허용 CAN ID, DLC 8을 검사한다.
3. CAN ID로 Topic과 기대 payload 크기를 결정한다.
4. `fragment_index` 범위와 마지막 fragment의 zero padding을 검사한다.
5. Topic별 reassembly buffer에 chunk를 복사하고 수신 bitmap을 표시한다.
6. 모든 fragment가 들어오면 big-endian decode를 수행한다.
7. 복원한 Topic, sequence, 물리값을 로그로 확인하고 응용 로직에 전달한다.

같은 Topic에 다른 sequence가 도착하면 기존의 미완성 payload를 폐기하고
새 sequence 재조립을 시작한다. 첫 fragment 수신 후 50 ms 이내에
완성되지 않은 payload도 폐기한다. 중복 fragment는 같은 buffer 위치에
덮어쓰되 완성 카운트를 두 번 증가시키지 않는다.

## 8. 오류 처리

| 조건 | 동작 |
|---|---|
| UART sync 손실 | `AA 55`를 다시 탐색 |
| 알 수 없는 UART Topic ID | 프레임 후보 폐기 |
| UART length 불일치 | 프레임 후보 폐기 |
| UART frame timeout | 미완성 프레임 폐기 |
| CAN Extended/RTR | 무시 |
| CAN ID/DLC 오류 | 무시하고 진단 카운터 증가 |
| fragment index/padding 오류 | 해당 fragment 폐기 |
| sequence 변경 | 기존 미완성 reassembly 폐기 후 새로 시작 |
| reassembly timeout | 미완성 payload 폐기 |
| CAN TX 실패 | 현재 payload 송신 중단, 오류 로그 |
| bus-off | 상태 기록 후 ESP-IDF 절차로 복구 |

UART에는 CRC가 없으므로 PC→A 구간의 payload bit corruption은
애플리케이션에서 검출할 수 없다. CAN 구간은 CAN 컨트롤러의 CRC,
ACK, 재전송 기능을 사용한다. v1에서 별도 애플리케이션 ACK는
추가하지 않는다.

## 9. 복원 필드

보드 B는 다음 순서로 복원한다.

| Topic | 필드 순서 |
|---|---|
| BlueROV/Torpedo Odometry | position x,y,z; quaternion x,y,z,w; linear x,y,z; angular x,y,z |
| IMU | quaternion x,y,z,w; angular velocity x,y,z; linear acceleration x,y,z |
| Pressure | `fluid_pressure` |
| Depth | `point.z` |
| DVL | linear velocity x,y,z |
| Mission goal | point x,y,z |

수신 로그에는 최소한 Topic ID, sequence, 복원된 값을 표시한다.
센서 주기가 높으면 로그가 통신 task를 방해할 수 있으므로 Topic별
throttle 또는 주기 통계 로그를 사용한다.

## 10. 통합 시험 순서

1. 보드 B만 연결하고 CAN 분석기에서 문서의 프레임을 주입한다.
2. Pressure, DVL, Mission goal의 예제 바이트와 decode 값을 확인한다.
3. 32-byte Odometry의 6개 fragment 재조립을 확인한다.
4. fragment 누락, 중복, 순서 변경, 잘못된 DLC/padding을 시험한다.
5. 보드 A에 UART 프레임을 직접 입력해 CAN 바이트를 확인한다.
6. PC, 보드 A, 보드 B를 모두 연결한다.
7. ROS 토픽을 하나씩 publish하여 B의 복원 로그를 확인한다.
8. 모든 토픽을 실제 주기로 보내 queue overflow와 bus load를 확인한다.
9. CAN_H/CAN_L 분리 후 bus-off 로그와 복구를 확인한다.

## 11. 현장 확정 항목

| 항목 | 현재 기본값/상태 |
|---|---|
| 보드 A USB 장치 | `/dev/ttyACM0` |
| 보드 A TWAI TX/RX | GPIO17/GPIO18, 실제 배선 확인 필요 |
| 보드 B TWAI TX/RX | GPIO17/GPIO18, 실제 배선 확인 필요 |
| CAN transceiver 모델 | 미정 |
| 보드 B 최종 데이터 사용처 | v1은 decode/로그, 후속 제어 로직 미정 |

## 12. 완료 기준

- PC의 기존 C++ 프레임을 변경하지 않고 보드 A가 parsing한다.
- 모든 Topic이 표의 CAN ID와 fragment 규칙대로 전송된다.
- 보드 B가 누락/오류 fragment를 완성 payload로 오인하지 않는다.
- 보드 B의 decode 값이 원래 ROS 메시지와 양자화 오차 범위에서 일치한다.
- 두 보드가 500 kbit/s에서 지속적으로 동작하고 bus-off 상태가 관리된다.
