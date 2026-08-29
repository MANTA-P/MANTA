# CAN 통신 규약 설계 가이드

## 1. CAN ID와 데이터의 의미

CAN 표준은 payload가 위치, 속도 또는 추진기 출력인지 정하지 않는다. CAN
프레임은 CAN ID, DLC와 최대 8-byte payload를 전달하며, 각 데이터의 의미는
프로젝트 통신 규약에서 정의한다.

CAN은 broadcast 방식이므로 모든 노드가 버스의 프레임을 볼 수 있고, 각 노드는
ID 필터로 필요한 메시지만 처리한다. 표준 11-bit CAN에서는 ID 숫자가 작을수록
중재 우선순위가 높다.

```text
0x001: 높은 우선순위
0x100: 중간 우선순위
0x700: 낮은 우선순위
```

CAN ID는 단순 장치 주소보다 메시지 내용과 우선순위를 나타내도록 설계한다.
하나의 ID는 가능하면 한 송신 노드만 사용한다. 서로 다른 두 노드가 같은 ID로
동시에 다른 payload를 보내면 데이터 구간에서 충돌해 CAN 오류가 발생할 수 있다.

## 2. 통신 매트릭스와 DBC

회사에서는 CAN 규약을 `CAN communication matrix` 또는 CAN 통신 명세 표로
관리한다. 자동차 분야에서는 DBC나 AUTOSAR ARXML 같은 기계 판독 가능한
파일도 사용한다.

각 메시지에는 다음 항목이 필요하다.

- 메시지 이름, 설명, CAN ID와 frame 형식
- 송신 노드, 수신 노드, 송신 주기 또는 이벤트 조건
- DLC와 payload 안의 signal 배치
- signal 자료형, bit 위치와 길이, signed/unsigned
- byte order, scale, offset, 단위와 정상 범위
- 초기값, 오류값, sequence 또는 rolling counter
- timeout과 timeout 발생 시 안전 동작
- 우선순위

통신 규약은 코드와 함께 Git으로 버전 관리한다. 규약이 안정되면 같은 내용의
DBC를 만들어 CAN 분석기와 PC 프로그램이 원시 payload를 물리값으로 해석하게
한다.

## 3. BlueROV 위치 인코딩 예시

BlueROV 위치 `(x, y, z)`는 회피와 제어의 핵심 입력이므로 반드시 CAN 통신
규약에 포함한다.

Classical CAN은 payload가 8 byte이므로 ROS 메시지 전체나 여러 `double` 값을
그대로 보내기 어렵다. 필요한 물리량을 정수와 scale로 변환할 수 있다.

```text
BlueROV X 위치: 12.34 m
scale:           0.01 m/bit
전송 int16 값:   1234
수신 복원식:     position_x = raw_x * 0.01 m
```

위치는 음수가 가능하므로 signed 자료형을 사용한다. 필요한 범위에 따라
`int16_t`, `int32_t` 또는 `float32` 중 하나를 선택한다.

반드시 함께 정할 항목은 다음과 같다.

- 좌표계: 예를 들어 `map`
- 단위: m
- X/Y/Z 축과 부호: ROS/Gazebo 좌표계 기준
- byte order: big-endian

- 위치 범위와 필요한 해상도
- 데이터 생성시각과 timeout
이 프로젝트의 모든 wire format은 big-endian으로 통일한다. CAN DBC에서는
Motorola byte order로 표기한다. 다중 byte 값은 송신 전에 명시적으로
big-endian으로 직렬화하고 수신 후 복원하며, C/C++ 구조체 메모리를 그대로
전송하지 않는다.

- `uint16_t`, `int16_t`: 최상위 byte부터 전송
- `uint32_t`, `int32_t`: 최상위 byte부터 전송
- `float32`: IEEE 754 bit pattern을 `uint32_t`로 변환한 뒤 최상위 byte부터 전송
- 1-byte 값: endian의 영향을 받지 않음

## 4. 프로젝트 CAN 통신 매트릭스 초안

아래 표는 구조를 논의하기 위한 초안이다. CAN ID, 주기, 자료형과 DLC는 실제
값의 범위와 필요한 정밀도를 확인한 뒤 확정한다.

| ID 초안 | 메시지 | 송신 | 수신 | 조건 | DLC | 우선순위 |
| ---: | --- | --- | --- | --- | ---: | --- |
| `0x001` | E-stop | Teensy CAN2 | ESP32 B | 이벤트 + 반복 | TBD | 최고 |
| `0x080` | BlueROV 위치 XY | ESP32 A | ESP32 B | 주기 | 8 | 높음 |
| `0x081` | BlueROV 위치 Z/상태 | ESP32 A | ESP32 B | 주기 | TBD | 높음 |
| `0x090` | BlueROV 선속도 | ESP32 A | ESP32 B | 주기 | TBD | 높음 |
| `0x091` | BlueROV 자세/각속도 | ESP32 A | ESP32 B | 주기 | TBD | 높음 |
| `0x0A0` | 깊이/압력 | ESP32 A | ESP32 B | 주기 | TBD | 높음 |
| `0x0A1` | DVL 속도 | ESP32 A | ESP32 B | 주기 | TBD | 높음 |
| `0x100` | 어뢰 위치 | ESP32 A | ESP32 B | 주기 | TBD | 중간 |
| `0x110` | 임무 목표 위치 | ESP32 A | ESP32 B | 변경 + 반복 | TBD | 중간 |
| `0x200` | 추진기 출력 1 | ESP32 B | ESP32 A | 제어 주기 | TBD | 높음 |
| `0x201` | 추진기 출력 2 | ESP32 B | ESP32 A | 제어 주기 | TBD | 높음 |
| `0x700` | 가상 소나 덤프 | Teensy CAN1 | 측정 노드 | 최대속도 | 8 | 최저 |

BlueROV 위치 XY와 Z를 나눈 구성은 예시다. 최종 자료형과 정밀도에 따라 한
프레임 또는 여러 프레임으로 다시 배치한다. E-stop 우선순위 역전 시험에서는
E-stop과 덤프 ID의 상대 우선순위를 시험 모드로 교환한다.

## 5. 메시지 전송 방식

```text
주기 전송: BlueROV 위치·속도·자세, 추진기 출력, heartbeat
이벤트 전송: 목표 변경, 오류, 운용 모드 변경
이벤트 + 반복: E-stop 발생 즉시 전송 후 짧은 주기로 반복
```

수신 노드는 마지막 수신시각을 저장한다. BlueROV 위치나 속도처럼 필수적인
데이터가 timeout되면 ESP32 B는 오래된 값을 계속 사용하지 않고 안전 출력을
적용한다.

## 6. 큰 데이터와 대표적인 상위 프로토콜

8 byte보다 큰 데이터는 여러 CAN ID, sequence 프레임 또는 ISO-TP를 사용해
분할한다. 최신성이 중요한 상태 데이터는 작은 독립 프레임과 timeout이 적합하다.

| 방식 | 특징 | 이번 프로젝트 적용성 |
| --- | --- | --- |
| 프로젝트 전용 규약 | ID와 payload를 직접 정의 | 초기 HIL과 CAN 학습에 적합 |
| CANopen | PDO, SDO, EMCY, heartbeat, NMT 제공 | 현재 단계에는 복잡함 |
| SAE J1939 | 29-bit ID에 우선순위, 기능, 주소 구조화 | 선박·중장비 확장 시 검토 |
| ISO-TP | 큰 데이터를 여러 CAN 프레임으로 전송 | 설정·진단 데이터에 검토 |
| DBC/AUTOSAR | 신호와 네트워크를 도구로 관리 | 규약 안정 후 DBC 도입 |

이번 프로젝트는 우선 **프로젝트 전용 Classical CAN 규약 + 통신 매트릭스 +
DBC** 방식을 사용한다. 가상 소나 덤프는 재전송하지 않고 최신값만 유지하며,
전송하지 못한 데이터는 폐기한다.

## 7. PC-ESP32 USB 전송 속도

현재 PC에서 보이는 `/dev/ttyACM0`는 GPIO UART가 아니라 ESP32-S3 내부의
USB Serial/JTAG 컨트롤러가 제공하는 CDC serial channel이다. 터미널 API에서는
baud rate를 설정하지만 실제 데이터는 UART baud clock이 아니라 USB 전송으로
이동한다.

500 kbit/s Classical CAN의 물리 선로 최대량은 다음과 같다.

```text
500,000 bit/s ÷ 8 = 62,500 byte/s
```

실제 CAN payload 처리량은 CAN ID, CRC, ACK, frame 간격과 bit stuffing 때문에
이보다 낮다. 따라서 USB CDC가 SITL 상태 데이터를 ESP32 A에 공급하는 속도는
500 kbit/s CAN의 게이트웨이 용도로 충분하다. 다만 ESP32 애플리케이션이 USB
데이터를 제때 읽지 않으면 USB Serial/JTAG 쓰기가 block될 수 있으므로 수신
태스크와 ring buffer가 필요하다.

별도 USB-UART 변환기나 GPIO UART를 실제 115200bps로 사용하면 상황이 다르다.

```text
115200 baud, 8N1
문자 1 byte당 시작 1 + 데이터 8 + 정지 1 = 10 bit

최대 유효 전송량 = 115200 ÷ 10
                  = 11,520 byte/s
```

이는 500 kbit/s CAN 전체 트래픽을 공급하기에 부족하다. 본 프로젝트의 HIL
데이터 경로는 ESP32-S3 USB CDC를 기본으로 하며, GPIO UART를 사용할 경우에는
더 높은 baud rate와 실제 오류율을 별도로 검증한다.

로그를 동일한 USB 채널로 대량 출력하면 HIL 데이터 전송을 방해할 수 있다.
운용 모드에서는 프레임별 로그를 끄고 주기 통계만 전송하거나, 디버그 로그와
HIL 데이터 채널을 분리한다.

## 8. PC-ESP32 이진 패킷 규약 초안

### 8.1 목적

USB CDC는 byte stream이므로 read 한 번과 패킷 하나의 경계가 일치하지 않는다.
한 패킷이 여러 read로 나뉘거나 여러 패킷이 한 read에 같이 들어올 수 있다.
따라서 ESP32 A는 수신 byte를 누적하고 명시적인 헤더와 길이를 기준으로
패킷을 조립해야 한다.

문자열과 JSON은 확인하기 쉽지만 크기가 크고 parsing 시간이 불규칙하다.
실시간 HIL 경로에는 고정 헤더를 가진 binary packet을 사용한다.

### 8.2 기본 패킷 구조

모든 다중 byte 정수와 실수는 big-endian을 사용한다. CAN payload와 USB/UART
패킷에 같은 byte order를 적용하며 DBC에서는 Motorola로 표기한다.

| Offset | 크기 | 필드 | 설명 |
| ---: | ---: | --- | --- |
| 0 | 2 | Magic | 고정값 `0xAA 0x55` |
| 2 | 1 | Version | 초기값 `0x01` |
| 3 | 1 | Message type | payload 종류 |
| 4 | 1 | Flags | 방향, ACK 요구, 오류 등의 bit flag |
| 5 | 2 | Sequence | 메시지 종류별 증가 counter |
| 7 | 2 | Payload length | payload byte 수, 최대 512 |
| 9 | 4 | Timestamp | 송신 측 단조 증가 시간, 단위 μs |
| 13 | N | Payload | 메시지별 데이터 |
| 13+N | 2 | CRC16 | Header부터 payload까지 CRC-16/CCITT-FALSE |

고정 overhead는 CRC를 포함해 15 byte이다.

```text
AA 55 | VER | TYPE | FLAGS | SEQ | LEN | TIME | PAYLOAD | CRC16
```

Magic은 wire 순서 그대로 `AA 55`를 전송한다. timestamp는 `uint32_t`이므로
약 71분마다 wrap된다. 지연 계산은 unsigned 차이를 사용해 wrap을 처리한다.
장시간 절대시각이 필요하면 이후 버전에서 `uint64_t`로 확장한다.

### 8.3 Message type 초안

| Type | 방향 | Payload | 설명 |
| ---: | --- | --- | --- |
| `0x01` | 양방향 | 상태/counter | heartbeat |
| `0x10` | PC→ESP32 B | x, y, z | BlueROV 위치 |
| `0x11` | PC→ESP32 B | vx, vy, vz | BlueROV 선속도 |
| `0x12` | PC→ESP32 B | quaternion/각속도 | BlueROV 자세 및 IMU |
| `0x13` | PC→ESP32 B | pressure, depth | 압력 및 깊이 |
| `0x14` | PC→ESP32 B | DVL velocity | DVL 상태 |
| `0x20` | PC→ESP32 B | 위치/속도 | 어뢰 상태 |
| `0x21` | PC→ESP32 B | x, y, z | 임무 목표 위치 |
| `0x80` | ESP32 B→PC | thruster 1~6 | BlueROV 추진기 출력 |
| `0xE0` | 양방향 | error/counter | 통신 상태 및 통계 |
| `0xF0` | 양방향 | key/value | 설정 및 시험 모드 |

Type 값과 payload 자료형은 최종 ROS-CAN 통신 매트릭스에서 확정한다.

### 8.4 BlueROV 위치 payload 예시

USB 구간은 CAN보다 대역폭 여유가 있으므로 초기 구현에서는 세 축을
`float32`로 한 패킷에 넣을 수 있다.

| Payload offset | 크기 | 자료형 | Signal | 단위 |
| ---: | ---: | --- | --- | --- |
| 0 | 4 | float32 | position_x | m |
| 4 | 4 | float32 | position_y | m |
| 8 | 4 | float32 | position_z | m |

payload 길이는 12 byte이고 전체 USB 패킷은 27 byte이다. ESP32 A는 이를
수신한 뒤 CAN 규약에 맞는 고정소수점 값 또는 여러 CAN 프레임으로 변환한다.
USB와 CAN에서 반드시 같은 인코딩을 쓸 필요는 없지만 변환 규칙은 문서와
테스트 코드에 명시한다.

### 8.5 Parser 동작

ESP32 A의 stream parser는 다음 순서로 동작한다.

1. ring buffer에서 `AA 55` Magic을 찾는다.
2. 고정 header 13 byte가 모일 때까지 기다린다.
3. Version과 Payload length를 검사한다.
4. `13 + payload length + 2` byte가 모일 때까지 기다린다.
5. CRC16을 검증한다.
6. 정상 패킷을 message type별 최신 데이터 저장소로 전달한다.
7. 오류가 있으면 한 byte 전진해 다음 Magic을 다시 탐색한다.

Payload length가 512를 넘거나, 정해진 시간 안에 패킷이 완성되지 않거나,
CRC가 틀리면 해당 패킷을 폐기한다.

### 8.6 신뢰성과 큐 정책

- 위치, 속도, 자세처럼 주기적으로 갱신되는 상태는 ACK 없이 최신값을 유지한다.
- 오래된 주기 데이터가 대기 중이면 같은 type의 새 패킷으로 교체할 수 있다.
- 설정 변경처럼 반드시 적용되어야 하는 명령만 ACK와 재시도를 사용한다.
- sequence 누락으로 USB 또는 내부 큐의 데이터 폐기를 검출한다.
- heartbeat timeout 시 ESP32 B는 안전 출력을 적용한다.
- CRC 오류, 길이 오류, sequence 누락과 queue overflow를 각각 집계한다.
- E-stop은 Teensy의 독립 CAN 경로를 사용하며 USB packet 지연에 의존하지 않는다.

### 8.7 구현 전 검증

USB의 실제 처리량은 명목 USB 속도만으로 확정하지 않고 다음 시험으로 검증한다.

1. PC가 크기가 알려진 binary packet을 연속 송신한다.
2. ESP32가 수신 byte, packet, CRC 오류와 overflow 수를 집계한다.
3. 1분 이상 송신해 평균·최소·최대 처리량을 기록한다.
4. 동시에 CAN 500 kbit/s 송수신과 ESP32 B 제어 주기를 실행한다.
5. HIL 데이터 누락, 제어 주기 jitter 및 USB block 여부를 확인한다.

통과 기준은 실제 HIL 입력 데이터율에 안전 여유를 더한 부하에서 packet 손상과
의도하지 않은 queue overflow가 없고 제어 deadline을 만족하는 것이다.
