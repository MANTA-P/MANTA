# ESP32-S3 CAN/TWAI 프로젝트 분석 및 ROS 연동 인수인계 문서

> 목적: 이 저장소를 이어받는 에이전트가 현재 구현을 빠르게 이해하고, 이후 ROS에서 생성한 데이터를 CAN 버스로 보내는 기능을 설계·구현할 수 있도록 정리한 문서입니다.

## 1. 프로젝트 요약

현재 핵심 프로젝트는 [`sample_project/can_test`](sample_project/can_test)입니다.

- 보드: `YD-ESP32-S3-N16R8` 2대
- CAN 트랜시버: `WCMCU-230` (`SN65HVD230` 계열로 README에 기재)
- MCU 프레임워크: ESP-IDF 6.0.2
- 타깃: `esp32s3`
- 언어: C
- CAN 주변장치: ESP32-S3 내장 TWAI 컨트롤러
- CAN bitrate: `500000` bit/s
- 프레임: 표준 11-bit CAN ID, 최대 8-byte payload
- 현재 기능: CAN 초기화, 모든 표준 프레임 수신, 1초 주기의 카운터 프레임 송신
- 현재 미구현: ROS, UART/USB 네트워크 프로토콜, CAN 신호 디코딩, 명령/응답 프로토콜

상위 [`sample_project`](sample_project)는 ESP-IDF 기본 템플릿이며 `main/main.c`의 `app_main()`이 비어 있습니다. 실제 CAN 코드는 `can_test` 아래에만 있습니다.

## 2. 소스 구조

```text
sample_project/
├── CMakeLists.txt                 # 비어 있는 ESP-IDF 기본 프로젝트(sample_project)
├── main/main.c                    # 빈 app_main()
└── can_test/
    ├── CMakeLists.txt             # ESP-IDF 프로젝트(can_test)
    ├── sdkconfig                  # ESP32-S3 빌드 설정
    ├── README.md                  # 배선 및 빌드 안내
    └── main/
        ├── app_main.c             # CAN 시작 후 애플리케이션 태스크 시작
        ├── can_app.c              # RX/TX FreeRTOS 태스크와 테스트 프레임
        ├── can_app.h
        ├── can_bus.c              # TWAI 드라이버 래퍼, ISR callback, RX queue
        ├── can_bus.h              # can_message_t 및 버스 API
        ├── board_config.h         # GPIO, bitrate, node ID, CAN ID 설정
        └── CMakeLists.txt         # 소스 등록 및 esp_driver_twai 의존성
```

`can_test/build`에는 이미 빌드된 `can_test.elf`, `can_test.bin`, bootloader 및 partition table이 있습니다. 이들은 생성물이며 소스 분석 대상이 아닙니다.

## 3. 실행 흐름

```text
app_main()
  └─ can_bus_start()
       ├─ RX FreeRTOS queue 생성(depth=16)
       ├─ TWAI on-chip node 생성
       ├─ 표준 CAN 전체 수신 필터 등록
       ├─ RX 완료 ISR callback 등록
       └─ twai_node_enable()
  └─ can_app_start()
       ├─ can_receive_task: queue에서 수신 후 로그 출력
       └─ can_transmit_task: 1초마다 테스트 프레임 송신
```

초기화에 실패하면 오류 로그를 출력하고 애플리케이션 태스크를 시작하지 않습니다. 송수신 태스크는 각각 stack `4096`, priority `5`로 생성됩니다.

## 4. 하드웨어 설정 및 배선

현재 코드의 실제 설정은 [`board_config.h`](sample_project/can_test/main/board_config.h) 기준입니다.

| ESP32-S3 | WCMCU-230 | 의미 |
|---|---|---|
| GPIO17 | TXD | MCU가 트랜시버로 보내는 TX 입력 |
| GPIO18 | RXD | 트랜시버가 MCU로 내보내는 RX 출력 |
| 3.3V | VCC | 전원 |
| GND | GND | 공통 접지 |
| - | RS → GND | high-speed mode |

두 트랜시버의 `CANH`, `CANL`, `GND`를 연결하고, 버스 양 끝에 각각 120 Ω 종단저항을 둡니다. 실제 모듈 핀 배치와 종단저항 장착 여부는 하드웨어에서 반드시 재확인해야 합니다.

## 5. 현재 CAN 프로토콜

### 설정값

```c
#define CAN_TX_GPIO          GPIO_NUM_17
#define CAN_RX_GPIO          GPIO_NUM_18
#define CAN_NODE_ID          2       // 현재 소스 값; README 예시는 1
#define CAN_BITRATE          500000
#define CAN_MESSAGE_ID_BASE  0x100
```

노드별 송신 ID는 다음 식으로 계산됩니다.

```text
CAN ID = CAN_MESSAGE_ID_BASE + CAN_NODE_ID
```

따라서 현재 소스의 `CAN_NODE_ID=2`이면 `0x102`입니다. README의 설명처럼 두 번째 보드는 `2`로 빌드하고, 첫 번째 보드는 `1`로 빌드하면 각각 `0x101`, `0x102`를 사용합니다. ROS 연동 시에는 보드별 ID를 하드코딩하지 말고 파라미터로 분리하는 것이 좋습니다.

### 테스트 송신 프레임

`can_transmit_task()`가 1초마다 다음 프레임을 보냅니다.

| 항목 | 값 |
|---|---|
| CAN ID | `0x100 + CAN_NODE_ID` (`0x102` 현재 설정) |
| 표준/확장 | 표준(11-bit) |
| DLC | 8 |
| 주기 | 1000 ms |
| Byte 0~3 | `counter`의 big-endian 32-bit 값 |
| Byte 4 | `CAN_NODE_ID` |
| Byte 5~7 | `0x00` |

예를 들어 counter가 `1`이면 payload는 다음과 같습니다.

```text
00 00 00 01 02 00 00 00
```

송신 성공 시에만 counter가 증가합니다. 송신은 TWAI TX queue에 넣은 뒤 `twai_node_transmit_wait_all_done(..., 1000)`으로 완료를 최대 1초 기다립니다.

### 수신 동작

수신 필터는 다음과 같아 모든 표준 프레임을 허용합니다.

```text
id     = 0x000
mask   = 0x000
is_ext = false
```

RX 완료 ISR callback은 프레임을 읽어 `can_message_t`로 변환한 뒤 FreeRTOS queue(depth 16)에 넣습니다.

```c
typedef struct {
    uint32_t identifier;
    uint8_t  data_length;
    uint8_t  data[8];
    bool     is_extended;
    bool     is_remote;
} can_message_t;
```

애플리케이션 RX 태스크는 최대 1초 동안 queue를 기다리고 다음 형태로 로그를 출력합니다.

```text
RX id=0x<ID> dlc=<DLC> data=<8 bytes in hex>
```

현재 로그는 DLC가 8보다 작아도 `data[0]`부터 `data[7]`까지 출력합니다. 실제 유효 데이터는 `data_length` 바이트뿐입니다.

## 6. CAN bus abstraction API

[`can_bus.h`](sample_project/can_test/main/can_bus.h)가 제공하는 API는 다음과 같습니다.

```c
esp_err_t can_bus_start(void);
esp_err_t can_bus_send(uint32_t identifier,
                       const uint8_t *data,
                       uint8_t data_length);
esp_err_t can_bus_receive(can_message_t *message,
                          TickType_t timeout_ticks);
bool can_bus_is_running(void);
```

현재 `can_bus_send()`의 제한:

- 버스가 시작되지 않았거나 `data == NULL`이면 실패
- payload 길이는 `TWAI_FRAME_MAX_LEN` 이하(일반 CAN에서는 8)
- `identifier <= TWAI_STD_ID_MASK`만 허용하므로 확장 ID 송신 불가
- 송신 호출 timeout은 1000 ms
- 현재 호출부는 항상 8-byte payload를 사용

현재 `can_bus_receive()`는 ISR에서 받은 메시지를 queue에서 꺼내는 단순 blocking API입니다. ROS 노드와 직접 연결되는 API나 외부 transport는 없습니다.

## 7. 빌드 및 실행

```bash
cd /home/user/can/sample_project/can_test
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

실제 포트는 `/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyUSB0` 등 환경에 따라 달라집니다. `sdkconfig`에는 `esp32s3`, 2 MB flash, 115200 baud, FreeRTOS tick 100 Hz 설정이 확인됩니다.

저장소에는 `can_test.elf`와 `can_test.bin`이 존재하므로 과거에 빌드가 수행된 상태로 보입니다. 다만 현재 연결된 보드에서의 실제 CAN 송수신 성공 여부는 이 파일만으로 검증할 수 없습니다.

## 8. ROS 연동을 위해 구현해야 할 것

이 프로젝트는 ROS와 연결되어 있지 않습니다. “ROS 데이터를 이 CAN으로 보낸다”는 목표를 달성하려면 ROS와 ESP32 사이의 transport 및 CAN payload 규격을 먼저 정해야 합니다.

### 권장 구조 A: ROS 컴퓨터가 CAN adapter를 직접 사용

```text
ROS node → SocketCAN (can0) → CAN transceiver → CAN bus
```

이 구조에서는 ESP32 코드 변경 없이 ROS 2 노드가 SocketCAN으로 프레임을 송신할 수 있습니다. Linux CAN adapter와 bitrate 설정이 필요합니다. ROS 쪽에서 `can_msgs/msg/Frame` 또는 프로젝트 전용 메시지를 사용하고, `can0`에 write하는 노드를 구현합니다.

### 권장 구조 B: ESP32를 ROS-CAN gateway로 사용

```text
ROS node → USB CDC/UART protocol → ESP32 → TWAI → CAN bus
CAN bus → TWAI → ESP32 → USB CDC/UART protocol → ROS node
```

현재 ESP32 프로그램에는 USB/UART 브리지 프로토콜이 없으므로 다음을 추가해야 합니다.

1. ROS에서 ESP32로 보낼 명령 프레임의 wire format 정의
2. ESP32의 UART 또는 USB Serial/JTAG 수신 태스크 구현
3. 수신 명령을 `can_bus_send()` 호출로 변환
4. CAN 수신 메시지를 wire format으로 직렬화하여 ROS로 반환
5. sequence, length, checksum/CRC, timeout, 오류 응답 정의
6. ROS 2 노드에서 해당 protocol을 publish/subscribe 또는 service/action으로 연결

### 반드시 결정할 CAN 데이터 규격

- ROS topic/message와 CAN ID의 매핑
- 각 CAN ID의 DLC
- payload endian
- signed/unsigned 여부와 scale/offset
- 주기와 deadline
- node ID 및 송신 권한
- 수신 허용 ID 필터
- heartbeat, timeout, bus-off 복구 정책
- 표준 ID를 유지할지 확장 ID를 사용할지

현재 코드에는 `counter` 테스트 프레임만 정의되어 있으며 차량/센서/제어 데이터의 의미는 정의되어 있지 않습니다. ROS 에이전트는 이 테스트 프로토콜을 실제 신호 프로토콜로 간주하면 안 됩니다.

## 9. 현재 구현의 주의점 및 개선 후보

- `board_config.h`의 현재 `CAN_NODE_ID`는 `2`이지만 README의 기본 예시는 `1`입니다. 문서와 소스 중 하나를 기준으로 통일해야 합니다.
- 수신 callback에서 `xQueueSendFromISR()` 반환값을 검사하지 않습니다. RX queue가 가득 차면 프레임이 버려질 수 있으므로 drop counter와 경고가 필요합니다.
- `s_can_running`은 시작 성공 후에만 true가 되며 stop/deinit API는 없습니다.
- TWAI bus-off, error passive, arbitration lost 등의 상태 복구 로직이 없습니다.
- 수신 필터는 모든 표준 ID를 허용합니다. 실제 ROS 게이트웨이에서는 필요한 ID만 필터링하는 편이 안전합니다.
- `can_message_t`에는 `is_extended`, `is_remote`가 있지만 송신 API는 표준 data frame만 사실상 지원합니다.
- ISR에서 로컬 `rx_data[8]`를 사용하고 곧바로 queue 구조체에 복사하므로 해당 buffer lifetime 문제는 없습니다.
- `twai_node_transmit()` 호출에 원본 `data` 포인터를 전달하므로 API가 비동기 복사를 보장하는지 ESP-IDF 6.0.2 문서를 확인해야 합니다. 현재 호출은 transmit 완료 대기까지 수행합니다.
- 실제 배선, 120 Ω 종단, CANH/CANL 극성, 트랜시버 전원은 코드만으로 검증되지 않습니다.
- 상위 `sample_project`와 `sample_project/can_test`가 별도 ESP-IDF 프로젝트입니다. ROS 작업은 `can_test`를 기준으로 진행해야 합니다.

## 10. ROS 작업자에게 전달할 최소 요구사항

다음 순서로 작업하면 됩니다.

1. ROS 2 버전과 실행 컴퓨터의 CAN 연결 방식(SocketCAN 직접 연결 또는 ESP32 UART/USB gateway)을 확정한다.
2. 사용할 CAN ID/DLC/payload 신호표를 문서와 테스트 벡터로 만든다.
3. 현재 counter 프레임(`0x102`, 8 bytes, 1 Hz)을 이용해 먼저 송신·수신 경로를 검증한다.
4. ROS 노드의 parameter로 interface, bitrate, CAN ID, 주기를 노출한다.
5. 실제 데이터에 대해 encode/decode 단위 테스트와 `candump`/시리얼 로그 기반 통합 테스트를 추가한다.
6. ESP32 gateway를 선택했다면 UART/USB packet protocol과 reconnect/error handling을 먼저 구현한다.

## 11. 분석 기준 파일

- [`sample_project/can_test/main/app_main.c`](sample_project/can_test/main/app_main.c)
- [`sample_project/can_test/main/can_app.c`](sample_project/can_test/main/can_app.c)
- [`sample_project/can_test/main/can_bus.c`](sample_project/can_test/main/can_bus.c)
- [`sample_project/can_test/main/can_bus.h`](sample_project/can_test/main/can_bus.h)
- [`sample_project/can_test/main/board_config.h`](sample_project/can_test/main/board_config.h)
- [`sample_project/can_test/README.md`](sample_project/can_test/README.md)
- [`sample_project/can_test/main/CMakeLists.txt`](sample_project/can_test/main/CMakeLists.txt)
- [`sample_project/can_test/sdkconfig`](sample_project/can_test/sdkconfig)

