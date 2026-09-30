# ESP32-S3 CAN 계산 노드 배선 및 플래시 가이드

이 프로젝트는 Classical CAN 500 kbit/s 계산 노드이다. ESP32-S3의 TWAI
컨트롤러는 CAN 물리 계층을 내장하지 않으므로 **외부 CAN 트랜시버가 반드시
필요하다.** ESP32-S3 GPIO를 `CAN_H` 또는 `CAN_L`에 직접 연결하면 안 된다.

## 준비물

- ESP32-S3 개발 보드 1대
- 3.3 V 로직 호환 CAN 트랜시버 1개
  - 예: SN65HVD230(3.3 V)
  - 5 V 전용 모듈은 RXD 출력이 5 V일 수 있으므로 데이터시트와 모듈 회로를
    확인한다.
- 상대 CAN 요청 노드 또는 CAN 분석기
- 버스 양 끝용 120 Ω 종단저항 2개

## 기본 핀 연결

기본 설정은 TX=`GPIO4`, RX=`GPIO5`이다. 사용하는 ESP32-S3 보드에서 이 핀이
플래시/PSRAM 또는 다른 주변장치에 예약되어 있지 않은지 보드 회로도를 먼저
확인한다. 핀은 `idf.py menuconfig`의 `CAN calculator node configuration`에서
변경할 수 있다.

| ESP32-S3 | CAN 트랜시버 | 설명 |
|---|---|---|
| `3V3` | `VCC` | 3.3 V 트랜시버 전원. 모듈 사양 우선 |
| `GND` | `GND` | 로직 및 CAN 버스 공통 기준 전위 |
| `GPIO4` (TWAI TX) | `TXD` 또는 `D` | ESP32-S3에서 트랜시버로 송신 |
| `GPIO5` (TWAI RX) | `RXD` 또는 `R` | 트랜시버에서 ESP32-S3로 수신 |
| 연결 없음 | `CAN_H` | 상대 노드의 `CAN_H`와 연결 |
| 연결 없음 | `CAN_L` | 상대 노드의 `CAN_L`과 연결 |

트랜시버에 `STB`, `S`, `RS`, `EN` 핀이 있으면 데이터시트에 따라 normal mode로
고정한다. 예를 들어 모듈에 따라 `STB`를 Low로 내려야 할 수 있다.

```text
ESP32-S3              CAN transceiver                 CAN bus
 GPIO4 (TX) --------> TXD                         CAN_H -------- CAN_H
 GPIO5 (RX) <-------- RXD                         CAN_L -------- CAN_L
 3V3 ---------------- VCC                           GND -------- GND
 GND ---------------- GND
```

버스의 물리적인 양 끝에만 `CAN_H`와 `CAN_L` 사이 120 Ω을 하나씩 둔다. 전원을
끈 상태에서 `CAN_H`-`CAN_L` 저항을 재면 병렬값인 약 60 Ω이 정상이다. 두 노드는
반드시 GND를 공유하고, 모두 500 kbit/s로 설정한다.

## ESP-IDF 6.1 빌드 및 플래시

ESP-IDF 6.1 환경을 먼저 활성화한다. 설치 위치는 자신의 환경에 맞게 바꾼다.

```bash
source ~/esp/esp-idf/export.sh
cd /home/user/manta_ws/src/esp32_bridge/can_calculator_node
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

`menuconfig`에서 아래 항목을 실제 배선과 일치시킨다.

```text
CAN calculator node configuration
  TWAI TX GPIO
  TWAI RX GPIO
```

USB 포트를 확인한 뒤 플래시하고 로그를 연다. Linux 예시는 다음과 같다.

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

보드에 따라 포트는 `/dev/ttyUSB0`일 수 있다. monitor 종료 키는 `Ctrl+]`이다.
자동 다운로드가 되지 않으면 보드의 `BOOT`를 누른 채 `RESET`을 한 번 누르고,
플래시가 시작되면 `BOOT`를 놓는다.

정상 부팅 시 다음 설정을 포함한 로그가 출력된다.

```text
role=CALCULATOR bitrate=500000 TX_GPIO=4 RX_GPIO=5
CAN calculator node ready
```

## 통신 확인

요청 노드에서 표준 Data Frame, DLC 8로 아래 요청을 보내면 계산 노드는 결과를
ID `0x001`로 응답한다.

```text
TX request: ID=0x002 DATA=01 01 01 00 02 00 00 00
RX result:  ID=0x001 DATA=01 01 02 00 03 00 00 00
```

응답이 없다면 다음 순서로 점검한다.

1. 두 장치의 bitrate가 모두 500 kbit/s인지 확인한다.
2. `TXD/RXD`가 뒤바뀌지 않았는지, `CAN_H/CAN_L`가 같은 이름끼리 연결됐는지 확인한다.
3. 공통 GND와 양 끝의 120 Ω 종단을 확인한다.
4. 트랜시버가 normal mode인지 확인한다.
5. 부팅 로그의 GPIO 번호가 실제 배선과 같은지 확인한다.
6. CAN 버스에 ACK를 보낼 다른 활성 노드가 있는지 확인한다.

## codec 단위 테스트

호스트에 CMake와 C 컴파일러가 있으면 ESP32 보드 없이 프로토콜 벡터를 검사할
수 있다.

```bash
cmake -S test -B /tmp/can_calculator_test
cmake --build /tmp/can_calculator_test
ctest --test-dir /tmp/can_calculator_test --output-on-failure
```
