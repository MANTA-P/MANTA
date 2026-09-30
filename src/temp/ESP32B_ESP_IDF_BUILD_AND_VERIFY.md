# ESP32 B ESP-IDF 펌웨어 생성 및 검증 가이드

이 문서는 현재 저장소 기준으로 ESP32 B용 펌웨어를 실제로 만들어서 빌드/플래시/검증하는 절차를 정리한 문서다. 최우선 기준은 [docs/current/TORPEDO_HIL_AI_IMPLEMENTATION_SPEC.md](docs/current/TORPEDO_HIL_AI_IMPLEMENTATION_SPEC.md)이며, 이전 문서의 30 ms timeout과 “남은 결정”은 적용하지 않는다.

## 1. 목표

ESP32 B는 다음 역할을 수행한다.

- CAN 수신: 0x010 control command, odometry fragment 0x110/0x120, Teensy E-stop 0x001
- 제어 계산: torpedo_control_v2 기준의 control core와 safety logic
- CAN 송신: 0x180 actuator A, 0x181 actuator B, 0x1F0 heartbeat/status
- timeout / E-stop / fault 시 안전 출력(zero thrust, zero fin)

핵심 규격 요약:

- CAN bitrate: 500 kbit/s
- ID: 0x010, 0x180, 0x181
- big-endian, standard 11-bit CAN
- odometry timeout: 50 ms
- control command timeout: 200 ms
- heartbeat timeout: 500 ms
- 안전 출력: thrust=0, fins=0 rad

---

## 2. 시작 전 준비

### 2.1 ESP-IDF 설치

ESP32-S3 기반으로 작업할 경우 아래 절차를 따른다.

```bash
sudo apt update
sudo apt install -y git curl wget cmake ninja-build python3 python3-pip
sudo apt install -y libusb-1.0-0-dev

mkdir -p ~/esp-idf
cd ~/esp-idf
git clone -b v5.4.1 --depth 1 https://github.com/espressif/esp-idf.git .
./install.sh
source export.sh
```

설치 후 쉘마다 환경을 올리려면:

```bash
source ~/esp-idf/export.sh
```

### 2.2 확인

```bash
idf.py --version
python3 --version
```

---

## 3. 현재 저장소 기준으로 프로젝트 만들기

현재 워크스페이스 루트에서 ESP32 B 프로젝트를 별도 폴더로 만든다.

```bash
cd ~/manta_ws
mkdir -p esp32b_fw/main
cd esp32b_fw
```

프로젝트 파일을 만들고 타깃을 esp32s3로 설정한다.

```bash
idf.py create-project --path . esp32b_fw
idf.py set-target esp32s3
```

> ESP32 B는 USB Serial/JTAG를 사용할 수 있으므로 ESP32-S3가 가장 자연스럽다. 현재 문서는 S3를 기준으로 작성한다.

---

## 4. 기본 폴더 구조

권장 구조는 다음과 같다.

```text
~/manta_ws/esp32b_fw/
├── CMakeLists.txt
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   ├── app_main.c
│   ├── can_protocol.h
│   ├── can_protocol.c
│   ├── controller_state.h
│   ├── controller_state.c
│   ├── watchdog.h
│   └── watchdog.c
├── components/
│   └── torpedo_logic/   (선택)
│       ├── CMakeLists.txt
│       ├── torpedo_logic.h
│       └── torpedo_logic.c
└── build/
```

---

## 5. 프로젝트 메인 설정

`CMakeLists.txt`는 다음처럼 구성한다.

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32b_fw)
```

`main/CMakeLists.txt`는 다음처럼 만든다.

```cmake
idf_component_register(
    SRCS
        "app_main.c"
        "can_protocol.c"
        "controller_state.c"
        "watchdog.c"
    INCLUDE_DIRS "."
)
```

---

## 6. ESP32 B 구현 포인트

### 6.1 핀 설정

현재 문서 기준으로 ESP32 B는 다음을 사용한다.

```text
TWAI TX = GPIO17
TWAI RX = GPIO18
500 kbit/s, Classical CAN 2.0A
```

`app_main.c`에서 아래와 같이 초기화한다.

```c
#include "driver/gpio.h"
#include "driver/twai.h"

#define CAN_TX_GPIO 17
#define CAN_RX_GPIO 18
#define CAN_BITRATE TWAI_TIMING_CONFIG_500KBITS()
```

### 6.2 CAN 초기화

```c
static void can_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_init(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}
```

### 6.3 상태 머신

대표 상태는 다음과 같이 정의한다.

```c
enum torpedo_state_t {
    TORPEDO_STATE_DISARMED = 0,
    TORPEDO_STATE_RUNNING = 1,
    TORPEDO_STATE_TIMEOUT = 2,
    TORPEDO_STATE_ESTOP = 3,
    TORPEDO_STATE_FAULT = 4,
};
```

최종 규격에 따라 다음 흐름을 지킨다.

```text
DISARMED -> armed + valid mode + inputs valid -> RUNNING
RUNNING -> timeout / numeric fault / E-stop
TIMEOUT -> zero; 각 odometry 5회 정상 + armed 0->1 edge 후 RUNNING
FAULT -> zero; 원인 제거 + armed=0 후 DISARMED
ESTOP -> zero; release + inputs/command valid 후 armed에 따라 복귀
```

### 6.4 CAN frame 구조

최종 스펙에 맞춘 기본 레이아웃은 다음과 같다.

#### 0x010 control command

```text
byte0 version=1
byte1 sequence
byte2 armed (0/1)
byte3 mode (0=None, 1=Simple, 2=PNG)
byte4..5 target thrust uint16 BE, 0..1000
byte6..7 reserved=0
```

#### 0x180 actuator A

```text
byte0 sequence
byte1 state: 0 DISARMED, 1 RUNNING, 2 TIMEOUT, 3 ESTOP, 4 FAULT
byte2..3 thrust uint16 BE
byte4..5 fin top int16 BE
byte6..7 fin bottom int16 BE
```

#### 0x181 actuator B

```text
byte0 same sequence
byte1 active mode
byte2..3 fin left int16 BE
byte4..5 fin right int16 BE
byte6 flags: bit0 torpedo valid, bit1 target valid, bit2 command valid,
             bit3 E-stop, bit4 CAN error, bit5 bus-off
byte7 reserved=0
```

#### 0x1F0 status/heartbeat

```text
version, heartbeat_seq, state, mode, flags, last_error,
 uptime_seconds_low16_BE
```

---

## 7. 구현 예시 코드 구조

### 7.1 `main/app_main.c`

```c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "can_protocol.h"
#include "controller_state.h"
#include "watchdog.h"

static const char *TAG = "esp32b_fw";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 B starting");

    can_init();
    controller_init();
    watchdog_init();

    while (1) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            can_handle_message(&rx_msg);
        }

        controller_tick_20hz();
        watchdog_tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### 7.2 `main/can_protocol.h`

```c
#pragma once

#include "driver/twai.h"

void can_init(void);
void can_handle_message(const twai_message_t *msg);
void can_send_actuator_frame(uint8_t seq, uint16_t thrust, int16_t fin_top, int16_t fin_bottom);
void can_send_actuator_b_frame(uint8_t seq, int16_t fin_left, int16_t fin_right, uint8_t flags);
void can_send_status_frame(uint8_t seq, uint8_t state, uint8_t mode, uint8_t flags, uint8_t last_error);
```

### 7.3 `main/controller_state.h`

```c
#pragma once
#include <stdint.h>

void controller_init(void);
void controller_tick_20hz(void);
void controller_set_cmd(uint8_t armed, uint8_t mode, uint16_t target_thrust);
void controller_set_odometry_valid(uint8_t valid);
void controller_set_estop(uint8_t active);
```

### 7.4 `main/watchdog.h`

```c
#pragma once
#include <stdint.h>

void watchdog_init(void);
void watchdog_tick(void);
void watchdog_note_control(uint8_t valid);
void watchdog_note_heartbeat(uint8_t valid);
```

---

## 8. 빌드 및 플래시 절차

현재 디렉토리에서 프로젝트를 빌드한다.

```bash
cd ~/manta_ws/esp32b_fw
source ~/esp-idf/export.sh
idf.py build
```

빌드가 끝나면 보드에 플래시한다.

```bash
idf.py -p /dev/ttyUSB0 flash
```

실제 포트가 ttyACM0 또는 ttyUSB0일 수 있으니 확인한다.

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

모니터링한다.

```bash
idf.py -p /dev/ttyUSB0 monitor
```

종료는 `Ctrl+]`를 누른다.

---

## 9. 검증 방법

### 9.1 빌드 검증

```bash
idf.py build
```

목표: exit code 0, no compiler errors.

### 9.2 플래시 검증

```bash
idf.py -p /dev/ttyUSB0 flash
```

목표: flash 성공, board boot logs 정상.

### 9.3 시리얼 로그 검증

```bash
idf.py -p /dev/ttyUSB0 monitor
```

 확인 항목:

- 시작 로그 출력
- CAN 초기화 성공
- idle 상태에서 heartbeat 전송
- control command 수신 시 상태 전환
- control timeout 시 안전 출력(zero)

### 9.4 CAN 검증

실제 CAN 버스를 관찰하려면 `can-utils`를 사용할 수 있다.

```bash
sudo apt install -y can-utils
candump -c any
```

검증 조건:

- 0x010 수신 시 armed/mode/target_thrust 값이 올바른지 확인
- 0x180, 0x181 송신 주기 20 Hz 확인
- E-stop 0x001 수신 시 state 전환 확인
- timeout 발생 시 thrust=0, fin=0 확인

### 9.5 timeout 검증

검증 시나리오:

1. 정상 상태: control command 20 Hz 반복 수신
2. 200 ms 이상 미수신: state=TIMEOUT
3. 안전 출력 확인: thrust=0, fins=0
4. 0x010 재수신 후 armed=1, valid mode이면 RUNNING 복귀

### 9.6 재가동 정책 검증

정책은 다음 흐름으로 검증한다.

```text
TIMEOUT -> zero -> 정상 odometry 5회 + armed 0->1 edge -> RUNNING
FAULT -> zero -> 원인 제거 + armed=0 -> DISARMED
ESTOP -> zero -> release + inputs valid -> armed에 따라 복귀
```

---

## 10. 공통 문제 해결

### 10.1 `idf.py: command not found`

```bash
source ~/esp-idf/export.sh
```

### 10.2 `No such file or directory` for port

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

보드가 연결되지 않았거나 USB 케이블 문제가 없는지 확인한다.

### 10.3 CAN TX/RX failure

다음 항목을 점검한다.

- GPIO17/18 맞는지
- CANH/CANL 연결 상태
- 공통 GND 연결
- 120 Ω 종단 저항 여부
- 500 kbit/s 설정

### 10.4 timeout이 계속 발생

- control command주기를 20 Hz로 맞추는지
- 0x010 payload의 endian과 값 범위가 올바른지
- odometry fragment 수신이 완료되었는지
- 메시지 sequence 및 연속성 확인

---

## 11. 권장 개발 순서

1. CAN 초기화와 receive 루프 구현
2. 0x010 parse 및 armed/mode/target_thrust 반영
3. odometry timeout, control timeout, heartbeat timeout 구현
4. 0x180/0x181 송신 구현
5. state machine 적용
6. 안전 출력 zero 정책 구현
7. 실제 CAN 버스와 Teensy E-stop 시험
8. build/flash/monitor를 포함한 end-to-end 검증

---

## 12. 추천 다음 단계

이 문서를 기준으로 다음 파일들을 만들면 실제 구현이 빠르게 진행된다.

- `main/app_main.c`
- `main/can_protocol.c`
- `main/controller_state.c`
- `main/watchdog.c`
- `main/CMakeLists.txt`

실제 ESP32 B 펌웨어는 이 문서의 절차대로 만들고, 빌드/플래시/모니터링/대역 검증을 반복하면서 검증을 완료하면 된다.

---

## 13. 바로 실행할 최소 명령

```bash
cd ~/manta_ws
mkdir -p esp32b_fw/main
cd esp32b_fw
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

> 포트는 `/dev/ttyUSB0` 대신 실제 보드 포트로 변경한다. `ls /dev/ttyACM* /dev/ttyUSB*`로 확인한다.
