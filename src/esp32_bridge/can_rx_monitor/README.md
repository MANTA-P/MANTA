# ESP-IDF CAN(TWAI) 수신 모니터

ESP32-S3의 TWAI(CAN) 버스로 들어오는 모든 Classic CAN 데이터 프레임을 수신해
`idf.py monitor` 로그로 출력하는 독립 프로젝트입니다. 이 프로그램은 CAN 데이터 프레임을
송신하지 않지만, 정상 CAN 노드로 동작해 다른 송신 노드에 ACK를 반환합니다.

## 기본 설정

| 항목 | 값 |
| --- | --- |
| CAN bitrate | 500 kbit/s |
| ESP32 TX GPIO | GPIO17 |
| ESP32 RX GPIO | GPIO18 |
| 수신 필터 | 전체 ID |

핀과 bitrate는 [`main/can_rx_config.h`](main/can_rx_config.h)에서 변경합니다.
CANH/CANL에는 SN65HVD230 등 3.3 V CAN transceiver가 필요하며, 버스 양 끝에는 120 Ω 종단저항이 필요합니다.

## 빌드, 플래시, 모니터

```bash
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
cd can_rx_monitor
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

`/dev/ttyACM0`은 보드의 실제 포트로 바꿉니다. 수신 프레임은 다음과 같이 표시됩니다.

```text
I (1234) can_rx_monitor: RX STD ID=0x200 DLC=3 DATA=[41 42 0A] ASCII="AB."
```

CAN 버스에 다른 활성 노드가 있어야 실제 프레임과 ACK가 정상적으로 발생합니다.
