# ESP32 UART → CAN bridge

노트북이 UART로 보낸 문자 1개를 ESP32가 받아 CAN 표준 데이터 프레임으로 송신하는 ESP-IDF C 펌웨어입니다.

## 데이터 규격

| 항목 | 기본값 |
| --- | --- |
| USB | ESP32-S3 USB Serial/JTAG (`/dev/ttyACM0`) |
| 입력 | USB CDC-ACM으로 수신한 1 byte |
| CAN bitrate | 500 kbit/s |
| CAN TX/RX | GPIO17 / GPIO18 |
| CAN ID | `0x200` (표준 11-bit) |
| DLC | `1` |
| CAN payload | `payload[0] = UART로 받은 문자 1 byte` |

예를 들어 노트북이 `A`를 보내면 ASCII 값 `0x41`이 CAN ID `0x200`, DLC `1`의 `payload[0]`으로 송신됩니다. 줄바꿈 문자도 별도 필터 없이 그대로 송신합니다.

핀과 bitrate, CAN ID는 [`main/board_config.h`](main/board_config.h)에서 변경합니다.

## 배선

### 노트북 USB

- 노트북과 ESP32-S3의 USB 포트를 USB 데이터 케이블로 연결한다.
- 노트북의 `/dev/ttyACM0`를 ROS 송신 프로그램의 device로 사용한다.
- 이 구성에서는 GPIO15/GPIO16이나 별도 USB-UART 어댑터를 사용하지 않는다.

### CAN

- ESP32 GPIO17 → CAN transceiver TXD
- ESP32 GPIO18 ← CAN transceiver RXD
- transceiver CANH/CANL을 수신 보드와 연결하고 공통 GND를 연결한다.
- CAN 버스 양 끝에 120 Ω 종단저항이 필요하다.

ESP32의 TWAI 신호는 CANH/CANL에 직접 연결할 수 없다. SN65HVD230 같은 CAN transceiver를 반드시 사용한다.

## 빌드와 플래시

ESP-IDF 6.0.2와 `esp32s3` 타깃을 기준으로 한다. 새 터미널마다 먼저 ESP-IDF 환경을 활성화해야 한다.

```bash
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
cd ~/manta_ws/src/esp32_bridge
idf.py set-target esp32s3
idf.py build
```

빌드가 성공하면 `build/esp32_bridge.bin`이 생성된다. ESP32의 flash/console 포트를 확인한 뒤 다음처럼 플래시하고 로그를 연다.

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

`/dev/ttyACM0`은 ESP32-S3 USB Serial/JTAG 포트다. 플래시 후에는 ROS 송신 프로그램이 이 포트를 열어 USB로 문자를 보낸다. `idf.py monitor`와 ROS 송신 프로그램을 동시에 실행하면 같은 포트를 서로 점유하므로 함께 사용하지 않는다. `build/`, `sdkconfig` 같은 빌드 생성물은 Git에서 제외된다.

## 수신 보드 확인

다른 CAN 보드는 bitrate를 500 kbit/s로 맞추고 `0x200` 프레임을 수신한다. 수신한 DLC가 1인지 확인한 뒤 `payload[0]`을 `char`로 출력하면 된다. CAN 송신에는 다른 활성 CAN 노드의 ACK가 필요하므로 수신 보드 또는 CAN 분석기를 연결한 상태에서 시험한다.
