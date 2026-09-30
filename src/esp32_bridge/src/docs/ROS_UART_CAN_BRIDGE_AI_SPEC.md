# ROS UART → CAN 두 ESP32-S3 펌웨어 AI 구현 명세

> 이 문서는 후속 AI가 코드를 구현할 때 사용할 규범적 작업 지시서다.  
> Wire protocol의 최종 기준은 `ROS_UART_CAN_BRIDGE_PROTOCOL.md`이다.  
> 두 문서가 충돌하면 `ROS_UART_CAN_BRIDGE_PROTOCOL.md`를 따른다.

## 1. 구현 목표

현재 저장소의 ESP-IDF 프로젝트를 두 역할로 빌드할 수 있게
구현한다.

- `GATEWAY`: PC USB Serial/JTAG → UART frame parser → CAN fragmentation/TX
- `RECEIVER`: CAN RX → reassembly → fixed-point decode → 로그/응용 callback

두 펜웨어는 공통 protocol/codec 소스를 공유해야 하며, Kconfig의
role 선택으로 각 보드에 다른 역할을 flash할 수 있어야 한다.

## 2. 반드시 먼저 읽을 파일

1. `src/ROS_UART_CAN_BRIDGE_PROTOCOL.md`
2. `src/ros_uart_tx_node.cpp`
3. `src/packet_codec.cpp`
4. `include/esp32_bridge/packet_codec.hpp`
5. `main/uart_can_bridge.c`
6. `main/board_config.h`
7. `src/CAN_CALCULATOR_PROTOCOL.md`
8. `src/CAN_CALCULATOR_AI_SPEC.md`

계산기 문서는 확인된 TWAI 하드웨어/오류 처리 패턴의 참고일 뿐이다.
ROS bridge의 CAN ID와 DATA 형식은 반드시 본 명세를 따른다.

## 3. 규범 용어

- **MUST**: 반드시 구현한다.
- **MUST NOT**: 구현하면 안 된다.
- **SHOULD**: 특별한 이유가 없으면 구현한다.
- **MAY**: 선택 구현이다.

## 4. 환경과 API 제약

- Target은 ESP32-S3이다.
- ESP-IDF의 `esp_twai.h`, `esp_twai_onchip.h`를 사용하는 node-handle TWAI
  API를 기준으로 한다.
- 구현 전 반드시 현재 `$IDF_PATH`의 로컬 헤더와 공식 예제를
  확인한다. 저장소의 현 송신 코드는 `twai_new_node_onchip()`,
  `twai_node_enable()`, `twai_node_transmit()`를 사용한다.
- legacy `driver/twai.h` API와 node-handle API를 한 구현에서 혼용하지 말라.
- USB 입력은 `driver/usb_serial_jtag.h`를 사용한다.
- ISR callback에서 block, log, heap allocation, decode, reassembly를 하지 말라.

## 5. 프로토콜 상수

모든 상수는 공통 헤더 하나에서만 정의하고 매직 넘버를
소스 여러 곳에 복제하지 말라.

```text
UART_SYNC_FIRST              = 0xAA
UART_SYNC_SECOND             = 0x55
UART_HEADER_SIZE             = 4
UART_MAX_PAYLOAD_SIZE        = 32
UART_FRAME_TIMEOUT_MS        = 50

CAN_BITRATE                  = 500000
CAN_DLC                      = 8
CAN_FRAGMENT_HEADER_SIZE     = 2
CAN_FRAGMENT_DATA_SIZE       = 6
CAN_REASSEMBLY_TIMEOUT_MS    = 50
CAN_TX_TIMEOUT_MS            = 100

TOPIC_BLUEROV_ODOMETRY       = 0x10
TOPIC_IMU                    = 0x11
TOPIC_PRESSURE               = 0x12
TOPIC_DEPTH                  = 0x13
TOPIC_DVL_VELOCITY           = 0x14
TOPIC_TORPEDO_ODOMETRY       = 0x20
TOPIC_MISSION_GOAL           = 0x21

CAN_ID_BLUEROV_ODOMETRY      = 0x110
CAN_ID_IMU                   = 0x111
CAN_ID_PRESSURE              = 0x112
CAN_ID_DEPTH                 = 0x113
CAN_ID_DVL_VELOCITY          = 0x114
CAN_ID_TORPEDO_ODOMETRY      = 0x120
CAN_ID_MISSION_GOAL          = 0x121
```

Payload 크기 lookup은 다음을 MUST 반환한다.

```text
0x10 -> 32
0x11 -> 20
0x12 -> 4
0x13 -> 4
0x14 -> 6
0x20 -> 32
0x21 -> 12
```

## 6. CAN wire format

모든 프레임은 Classical CAN, Standard ID, Data Frame, DLC 8이다.

```text
Byte 0     sequence          uint8, per-topic counter
Byte 1     fragment_index    uint8, zero based
Byte 2..7  payload_chunk     up to 6 bytes, final unused bytes are zero
```

Topic별 fragment 수:

```text
fragment_count = (payload_size + 5) / 6

0x10: 6   0x11: 4   0x12: 1   0x13: 1
0x14: 1   0x20: 6   0x21: 2
```

이 형식에 별도 little-endian 변환을 적용하지 말라. Byte 2∼7은
PC가 생성한 big-endian UART payload의 부분을 그대로 복사한다.

Gateway는 한 UART payload의 fragment를 다른 payload의 fragment과 섞지 말고
index 0부터 순서대로 송신해야 한다. Receiver는 그래도 순서를
가정하지 말고 bitmap으로 재조립해야 한다.

## 7. 권장 파일 구조

```text
main/
├── app_main.c                 # Kconfig role에 따라 node 실행
├── bridge_protocol.h          # ID, size, DTO, 공개 API
├── bridge_protocol.c          # lookup, fragment, reassembly, BE decode
├── uart_frame_parser.h
├── uart_frame_parser.c        # byte-stream state machine; ESP driver 비의존
├── can_transport.h
├── can_transport.c            # ESP-IDF TWAI adapter
├── gateway_node.h
├── gateway_node.c             # USB RX and CAN TX
├── receiver_node.h
├── receiver_node.c            # CAN queue, reassembly, output
├── board_config.h              # 배선 공통 설정; 가능하면 Kconfig 참조
└── Kconfig.projbuild           # role, GPIO, queue depth
```

기존 `main/uart_can_bridge.c`를 분리/대체해도 되지만, 기존의 검증된
USB Serial/JTAG 초기화와 TWAI node 초기화 패턴은 유지한다.
`main/CMakeLists.txt`에 모든 새 `.c` 파일과 필요 component dependency를
등록한다.

## 8. 공통 데이터 형과 API 요구사항

최소한 다음 개념을 표현하는 형을 만들라. 정확한 이름은 변경해도
되지만 wire format은 변경하지 말라.

```c
typedef struct {
    uint8_t topic_id;
    uint8_t length;
    uint8_t payload[32];
} uart_topic_frame_t;

typedef struct {
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} bridge_can_frame_t;
```

Protocol layer는 최소한 다음 기능을 제공해야 한다.

- Topic ID ↔ CAN ID lookup
- Topic ID/CAN ID가 알려진 값인지 검사
- Topic별 payload 크기와 fragment 수 lookup
- payload의 특정 fragment 생성
- CAN fragment 형식 검사
- `read_be_i16`, `read_be_i32`, `read_be_u32`
- Topic payload decode

C 구조체 memory layout, packed attribute, bit-field, host endian에 wire format을
의존하지 말라. 멀티바이트 정수는 shift/OR로 명시적으로
복원하고 signed 값의 2의 보수 bit pattern을 보존하라.

## 9. UART parser 상태 머신

다음 상태를 갖는 incremental parser를 구현하라.

```text
WAIT_SYNC_AA
  └─ 0xAA → WAIT_SYNC_55

WAIT_SYNC_55
  ├─ 0x55 → READ_TOPIC
  ├─ 0xAA → WAIT_SYNC_55
  └─ other → WAIT_SYNC_AA

READ_TOPIC → READ_LENGTH → READ_PAYLOAD
```

MUST 요구사항:

1. byte를 한 번에 하나씩 주어도, chunk로 주어도 같게 동작해야 한다.
2. Topic을 읽은 즉시 known-topic 검사를 한다.
3. Length가 Topic의 고정 길이와 일치하지 않으면 payload를 읽지 말고 resync한다.
4. payload 수신 중 `AA 55`를 데이터로 처리한다.
5. parser buffer는 정적 32-byte buffer를 사용하고 length에 의한 overflow를 막는다.
6. header 시작 후 50 ms 내에 완성되지 않으면 reset한다.
7. 완성 프레임을 소유권이 명확한 queue/callback으로 Gateway에 전달한다.

USB task는 한 byte씩 로그를 남기지 말고 작은 chunk buffer로
`usb_serial_jtag_read_bytes()`를 호출한 뒤 parser에 전달하라.

## 10. Gateway 송신 규칙

Topic별 `uint8_t sequence[7]` 또는 Topic state table을 유지하라. array index는
원본 Topic ID를 직접 사용하지 말고 lookup table의 dense index를 사용하라.

유효한 UART payload 하나에 대해:

1. 현재 Topic sequence를 local variable에 복사한다.
2. sequence state를 modulo 256으로 증가시킨다. CAN 송신 실패 시에도
   다음 payload는 새 sequence를 사용해 Receiver가 미완성 payload를 버리게 한다.
3. `fragment_index=0..count-1`로 frame을 생성한다.
4. CAN DATA 8 byte를 먼저 0으로 초기화한 뒤 header와 valid chunk를 복사한다.
5. Standard Data Frame, RTR=false, DLC=8로 송신한다.
6. 프레임 버퍼 lifetime을 driver 요구사항보다 짧게 두지 말라.
7. 간단하고 안전한 v1 구현은 각 frame의 `twai_node_transmit()` 후
   `twai_node_transmit_wait_all_done()`을 timeout으로 호출할 수 있다.
8. 한 fragment가 실패하면 나머지 fragment를 보내지 말고 실패 카운터와
   Topic/sequence/index/error를 로그한다.

CAN 송신이 느려져 USB 수신을 놓칠 수 있으면 USB parser task와
CAN TX task 사이에 bounded FreeRTOS queue를 두라. queue가 찼 때는 가장
오래된 데이터를 무한정 대기시키지 말고 drop 카운터를 기록하라.

## 11. Receiver RX 구조

ESP-IDF node-handle API의 RX는 event callback 기반이다. Receiver는 다음을
MUST 수행한다.

1. `on_rx_done` callback에서 payload 8 byte를 가진 지역 `twai_frame_t`로
   `twai_node_receive_from_isr()`를 호출한다.
2. frame header와 payload를 소유 buffer를 가진 queue item에 복사한다.
3. `xQueueSendFromISR()`로 RX task에 전달하고 high-priority task wake 값을
   callback 반환값에 정확히 반영한다.
4. queue full이면 drop counter만 증가시키고 반환한다.
5. callback에서 ESP_LOG, float 연산, mutex wait, heap allocation을 하지 말라.
6. 실제 TWAI 함수 signature와 callback 규칙은 설치된 ESP-IDF 헤더를
   다시 확인한다.

RX task에서는 다음 순서로 검사한다.

```text
standard frame? → data frame? → known ID? → DLC 8?
→ fragment index valid? → final padding zero? → reassembly
```

## 12. Reassembly 상태

7개 Topic에 대해 독립적인 상태를 유지하라.

```c
typedef struct {
    bool active;
    uint8_t sequence;
    uint8_t payload[32];
    uint8_t received_bitmap;
    int64_t first_fragment_time_us;
} reassembly_state_t;
```

최대 fragment 수가 6이므로 `uint8_t` bitmap으로 충분하다.

처리 규칙:

1. inactive state에 frame이 오면 해당 sequence로 시작한다.
2. active state에 다른 sequence가 오면 이전 상태를 incomplete drop하고
   새 sequence로 즉시 시작한다.
3. `offset=index*6`, `valid=min(6,payload_size-offset)`를 계산한다.
4. valid chunk만 payload에 복사하고 bitmap의 index bit를 set한다.
5. 중복 fragment는 덮어쓰되 complete count를 별도로 증가시키지 말라.
6. expected bitmap과 received bitmap이 같으면 payload를 decode/callback에
   전달하고 state를 inactive로 만든다.
7. first fragment 기준 50 ms timeout이면 incomplete drop하라.

Fragment 0이 먼저 오는 것을 재조립 시작 조건으로 요구하지 말라.
모든 fragment는 같은 CAN 송신자에서 오지만, 코드는 index 순서와
무관하게 완성을 판정해야 한다.

## 13. Payload decode

복원된 payload는 다음 정확한 offset으로 decode한다.

### Odometry `0x10`, `0x20`, 32 bytes

```text
0,4,8       int32 BE  position x,y,z       * 0.001 m
12,14,16,18 int16 BE  quaternion x,y,z,w   / 16384.0
20,22,24    int16 BE  linear x,y,z         * 0.001 m/s
26,28,30    int16 BE  angular x,y,z        * 0.001 rad/s
```

### IMU `0x11`, 20 bytes

```text
0,2,4,6     int16 BE  quaternion x,y,z,w   / 16384.0
8,10,12     int16 BE  angular x,y,z        * 0.001 rad/s
14,16,18    int16 BE  acceleration x,y,z   * 0.001 m/s^2
```

### 나머지 Topic

```text
Pressure 0x12: offset 0 uint32 BE * 1.0 Pa
Depth    0x13: offset 0 int32  BE * 0.001 m
DVL      0x14: offset 0,2,4 int16 BE * 0.001 m/s
Goal     0x21: offset 0,4,8 int32 BE * 0.001 m
```

Decode 전에 payload 길이가 Topic의 expected size와 일치하는지 다시
검사하라. float/double은 wire에 직접 싣지 말라.

## 14. Kconfig 및 빌드 요구사항

`Kconfig.projbuild`에서 다음을 선택 가능하게 하라.

- role choice: `GATEWAY` 또는 `RECEIVER` 중 정확히 하나
- `CAN_TX_GPIO`, default 17
- `CAN_RX_GPIO`, default 18
- CAN TX queue depth
- Receiver FreeRTOS queue depth
- optional diagnostic log period

빌드 시 역할이 모호하거나 두 역할이 동시에 선택되면 compile-time
오류가 나야 한다. 시작 로그에 role, bitrate, TX GPIO, RX GPIO,
CAN ID 목록을 표시하라.

보드 A의 USB CDC 포트는 ROS 노드가 점유하므로 같은 포트에
`idf.py monitor`를 동시에 열지 말라. 로그 확인이 필요하면 추가 UART,
JTAG 로그 또는 호스트 프로그램의 RX 로그 기능을 별도로 사용한다.

## 15. TWAI 오류 및 bus-off

- TWAI API의 모든 반환값을 검사하라.
- `on_state_change`, `on_error`, `on_tx_done`이 필요하면 ISR-safe notification으로
  task에 상태를 전달하라.
- bus-off 복구는 ISR에서 직접 blocking API를 호출하지 말고 관리
  task에서 `twai_node_recover()` 등 현 ESP-IDF의 권장 절차를 따라라.
- 복구 중 UART frame은 bounded queue 정책에 따라 drop해도 되지만
  무한정 블록하지 말라.
- 로그에 실패한 송신을 `sent` 또는 성공으로 표시하지 말라.

## 16. 로그와 통계

Gateway 최소 통계:

- UART bytes received
- valid/invalid/timeout UART frames
- Topic별 accepted frame count
- CAN frames sent/failed
- queue overflow drops
- TWAI state/error counters

Receiver 최소 통계:

- CAN frames received
- unknown ID, wrong DLC/type, bad index/padding
- duplicate fragments
- sequence replacement/incomplete timeout
- Topic별 completed payload count
- RX queue overflow
- TWAI state/error counters

매 센서 메시지를 `ESP_LOGI`로 출력해 실시간 처리를 방해하지
말라. 개발 mode의 throttled value log와 1초 이상 주기의 통계 로그를
권장한다.

## 17. 필수 단위 테스트

TWAI driver 없이 실행 가능한 protocol/parser 테스트를 만들라.

### UART parser

- 완전한 프레임 1개
- 1 byte씩 입력
- 한 chunk에 여러 프레임 입력
- sync 앞의 garbage
- `AA AA 55` resync
- 알 수 없는 Topic
- Topic-length mismatch
- payload 내 `AA 55`
- incomplete timeout
- 최대 32-byte payload의 buffer boundary

### Fragmentation

- 4-byte Pressure → 1 frame, 2 zero padding bytes
- 6-byte DVL → 1 frame, padding 없음
- 12-byte Goal → 2 frames
- 20-byte IMU → 4 frames, 마지막 4 zero padding bytes
- 32-byte Odometry → 6 frames, 마지막 4 zero padding bytes
- Topic별 sequence 독립성과 `255 → 0`

### Reassembly

- 정상 index 순서
- index 순서가 바뀐 입력
- 중복 fragment
- fragment 누락 timeout
- 완성 전 새 sequence 도착
- index 범위 초과
- 마지막 fragment의 nonzero padding
- unknown ID, Extended, RTR, DLC != 8

### Decode/test vectors

```text
Pressure 101325 Pa
UART: AA 55 12 04 00 01 8B CD
CAN:  ID 0x112, DATA 2A 00 00 01 8B CD 00 00
Decode: 101325 Pa

DVL (1.0, -1.0, 0.5) m/s
CAN: ID 0x114, DATA 2B 00 03 E8 FC 18 01 F4
Decode: 1.0, -1.0, 0.5

Goal (1.0, -2.0, 3.5) m
CAN fragment 0: ID 0x121, DATA 10 00 00 00 03 E8 FF FF
CAN fragment 1: ID 0x121, DATA 10 01 F8 30 00 00 0D AC
Decode: 1.0, -2.0, 3.5
```

## 18. 실기 검증

1. Gateway와 Receiver를 각각 clean build한다.
2. Receiver에 CAN analyzer로 필수 test vector를 주입한다.
3. Gateway에 host script로 raw UART test vector를 전송한다.
4. CAN analyzer에서 ID, DLC, sequence, index, chunk를 확인한다.
5. 두 보드를 연결하고 test vector end-to-end decode를 확인한다.
6. ROS 2 node를 실행하고 각 Topic을 단독 publish한다.
7. 모든 Topic을 정상 주기로 활성화하고 queue drop/bus load를 측정한다.
8. CAN cable 분리/재연결로 bus-off 및 recovery를 확인한다.

## 19. 구현 금지 사항

- PC C++ UART wire format을 ESP 구현 편의를 위해 변경하지 말라.
- 32-byte payload를 하나의 Classical CAN 프레임에 넣지 말라.
- UART payload를 little-endian으로 바꿔 CAN으로 보내지 말라.
- C struct를 wire payload에 `memcpy` 또는 pointer cast하지 말라.
- DLC, ID, IDE, RTR, fragment index를 검사하기 전에 DATA를 사용하지 말라.
- ISR에서 blocking API, log, malloc/free를 사용하지 말라.
- unbounded queue, `portMAX_DELAY` CAN TX, 무제한 재시도를 사용하지 말라.
- 오류 프레임을 부분 payload로 decode하지 말라.
- GPIO를 여러 소스에 중복 hard-code하지 말라.
- 보드 A와 PC가 사용 중인 같은 CDC port를 monitor가 동시 점유하게
  하지 말라.

## 20. Definition of Done

- [ ] `ROS_UART_CAN_BRIDGE_PROTOCOL.md`와 실제 wire bytes가 일치한다.
- [ ] Gateway/Receiver를 Kconfig에서 각각 선택해 clean build할 수 있다.
- [ ] USB parser가 7개 Topic과 malformed stream 테스트를 통과한다.
- [ ] 1, 2, 4, 6 fragment payload의 fragmentation/reassembly 테스트가 통과한다.
- [ ] Receiver가 누락, 중복, timeout, sequence 변경을 안전하게 처리한다.
- [ ] 모든 Topic의 big-endian fixed-point decode 값이 C++ encoder와 일치한다.
- [ ] TWAI callback이 ISR 제약을 준수한다.
- [ ] API 실패, queue overflow, bus-off가 진단 가능하다.
- [ ] 두 실제 ESP32-S3와 transceiver에서 end-to-end 수신이 확인된다.
- [ ] 실제 GPIO, transceiver 모델, 보드 B의 최종 데이터 사용처가
      사람용 문서에 반영된다.
