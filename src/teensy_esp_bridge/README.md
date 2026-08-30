# ESP32-S3 to Teensy 4.1 CAN example

ESP32-S3가 ESP-IDF 6의 TWAI 드라이버로 Classical CAN 프레임을 송신하고,
Teensy 4.1이 해당 프레임을 수신하는 최소 프로젝트다.

## 기본 설정

| 항목 | ESP32-S3 | Teensy 4.1 |
|---|---|---|
| CAN 속도 | 500 kbit/s | 500 kbit/s |
| CAN ID | Standard `0x123` | `0x123` 확인 |
| 주기 | 100 ms | 수신 즉시 출력 |
| CAN TX | GPIO 5 | CAN1 TX, pin 22 |
| CAN RX | GPIO 4 | CAN1 RX, pin 23 |

ESP32-S3 핀은 범용 기본값이다. 사용하는 보드 회로도와 대조한 후 필요하면
`idf.py menuconfig`의 `Teensy CAN bridge configuration`에서 변경한다.

## Payload

| Byte | 의미 |
|---|---|
| 0..3 | 32-bit sequence counter, little-endian |
| 4 | `0xA5` |
| 5 | `0x5A` |
| 6 | 상태 예약 영역, 현재 `0x00` |
| 7 | byte 0..6 XOR checksum |

## 배선

두 보드 모두 MCU와 CAN 버스 사이에 별도의 3.3 V 로직 호환 CAN
트랜시버가 필요하다.

```text
ESP32 GPIO5 (TWAI TX) -> ESP transceiver TXD
ESP32 GPIO4 (TWAI RX) <- ESP transceiver RXD

Teensy pin 22 (CAN1 TX) -> Teensy transceiver TXD
Teensy pin 23 (CAN1 RX) <- Teensy transceiver RXD

ESP transceiver CAN_H <-> Teensy transceiver CAN_H
ESP transceiver CAN_L <-> Teensy transceiver CAN_L
ESP/Teensy/transceiver GND는 서로 연결
```

버스 양 끝에 CAN_H와 CAN_L 사이 120 Ω 종단 저항을 하나씩 둔다. 일부
트랜시버 모듈은 종단 저항을 내장하므로 중복 여부를 확인한다.

## ESP32-S3 빌드 및 플래시

ESP-IDF 환경을 먼저 활성화한다.

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

이 프로젝트는 ESP-IDF v6.0.2의 새 `esp_twai.h` API를 사용한다. 기존
`driver/twai.h` legacy API는 사용하지 않는다.

실제 포트가 `/dev/ttyACM0` 등일 수 있다. 프로젝트 기본 설정은 500 kbit/s,
TX GPIO 5, RX GPIO 4다.

## Teensy 4.1 부하 및 E-stop 코드

기존 단순 수신 예제는 CAN1 dump와 CAN2 E-stop 우선순위 시험 코드로
교체되었다. 배선, 설정, payload와 업로드 방법은
`teensy_can_load_estop/README.md`를 참고한다.

## 문제 해결

- `TX queue timeout`: 송신 큐가 비워지지 않는다. 배선과 수신 노드 상태를 확인한다.
- `TX_FAILED` 또는 `bus error`: 비트레이트, CAN_H/L, 공통 GND, 종단 저항을 확인한다.
- `bus-off`: 수신 노드의 ACK가 없거나 물리 계층 오류가 반복되었다. 코드는 버스 복구를 자동 시도한다.
- Teensy에서 아무것도 수신하지 못함: CAN1 핀 22/23과 트랜시버 방향, standby/enable 핀을 확인한다.

상세 구현 계획은 `CAN_COMMUNICATION_PLAN.md`를 참고한다.
