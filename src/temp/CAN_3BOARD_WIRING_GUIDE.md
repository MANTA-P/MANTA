# ESP32 A · ESP32 B · Teensy 4.1 CAN 부하 시험 연결 가이드

이 문서는 **변경 후 목표 구성**을 설명한다. 현재 Teensy 스케치는 아직 CAN1 dump와 CAN2 E-stop을 동시에 사용하고, ESP32 B는 `0x001`을 E-stop으로 해석한다. 따라서 [ESP32 수정 명세](ESP32_CAN_CHANGE_SPEC.md)와 [Teensy 수정 명세](../teensy_esp_bridge/teensy_can_load_estop/TEENSY_CAN_CHANGE_SPEC.md)를 구현하기 전에는 아래 시험을 그대로 실행하지 않는다.

## 데이터 흐름

```text
노트북/ROS ⇄ UART ⇄ ESP32 A ⇄ 공통 CAN 버스 ⇄ ESP32 B(제어 계산)
                                      ↑
                       Teensy CAN1: 스위치 ON 동안 ID 0x001 부하
```

A는 노트북에서 받은 제어/오도메트리를 CAN으로 B에 전달한다. B는 현재 구현된 출력 프레임 `0x180`/`0x181`과 상태 `0x1F0`을 A에 보낸다. A는 출력 쌍을 UART `0x80`, 상태를 UART `0x81`로 노트북에 전달한다. 향후 B의 미사일 제어 계산을 업데이트해도 이 경로를 유지한다. 스위치를 누르면 Teensy가 가장 높은 우선순위의 시험용 dump `0x001`을 보내고, 놓으면 새 dump 생성·적재를 멈춘다. 이 스위치는 **E-stop 명령 스위치가 아니라 CAN 부하 시험 트리거**다.

## 물리 연결

변경 후 필요한 것은 **보드 3개, CAN 컨트롤러 3개, 3.3 V 로직 호환 CAN 트랜시버 3개, 공통 CAN_H/CAN_L 버스 1개**다. Teensy CAN2와 그 트랜시버는 사용하지 않는다.

| 노드 | MCU TX → 트랜시버 D/TXD | MCU RX ← 트랜시버 R/RXD | 트랜시버 전원 |
|---|---|---|---|
| ESP32 A (ESP32-S3) | GPIO 17 | GPIO 18 | A의 3.3 V/GND |
| ESP32 B (ESP32-S3) | GPIO 17 | GPIO 18 | B의 3.3 V/GND |
| Teensy 4.1 CAN1 | pin 22 (`CTX1`) | pin 23 (`CRX1`) | Teensy의 3.3 V/GND |

세 트랜시버의 CANH끼리, CANL끼리, GND끼리 연결한다. 각 보드의 3.3 V 전원 레일은 무조건 합치지 않는다. 트랜시버는 각 소속 보드에서 전원을 받고 기준 GND만 공통으로 둔다. CAN_H/CAN_L는 한 줄의 버스로 배치하고 각 노드의 가지선은 짧게 둔다. **120 Ω 종단저항은 전체 버스의 물리적인 두 끝에만 총 2개** 설치한다. 전원 OFF 상태의 CAN_H–CAN_L 저항은 일반적으로 약 60 Ω이다. 모듈에 이미 종단저항이 내장되어 있는지 확인한다. ESP32/Teensy의 TX/RX 핀을 CAN_H/CAN_L에 직접 연결하지 않는다. [Espressif TWAI 배선 설명](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/twai.html)

Teensy pin 2와 GND 사이에 스위치를 연결한다. `INPUT_PULLUP`, Active LOW를 유지한다. SN65HVD230을 쓴다면 `Rs`를 정상 송신 모드로 설정한다. `Rs`가 HIGH이면 트랜시버 드라이버가 꺼질 수 있다. [TI SN65HVD230 데이터시트](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)

## CAN ID와 관측 지점

모든 노드는 Classical CAN 500 kbit/s, 11-bit 표준 ID를 사용한다.

| CAN ID | 송신자 → 수신/처리자 | 역할 |
|---|---|---|
| `0x001` | Teensy CAN1 → 버스 | 스위치 ON 동안만 보내는 8바이트 시험 부하. B는 명령으로 처리하지 않음 |
| `0x010` | A → B | 제어 명령 |
| `0x110`, `0x120` | A → B | 오도메트리 조각 |
| `0x180`, `0x181` | B → A → 노트북 | 같은 sequence의 출력 쌍; A는 UART `0x80`으로 전달 |
| `0x1F0` | B → A → 노트북 | 상태; A는 UART `0x81`로 전달 |

CAN에서는 여러 노드가 **동시에 중재에 참여할 때** 낮은 ID가 이긴다. `0x001`은 이 시스템의 다른 ID보다 우선하지만 이미 시작한 프레임을 중단시키지는 못한다. Teensy가 매 중재 시점에 준비되지 않으면 다른 프레임이 통과할 수 있다. 따라서 이 방식은 B→A 출력의 완전 차단을 보장하지 않는다. **A→B 입력도 같은 버스에서 함께 지연**되므로, 제어 출력이 노트북에 늦게 온 이유를 구분하려면 B의 입력 수신률·출력 송신률, A의 출력 쌍 완성률, 노트북 UART `0x80` 수신률을 같이 기록해야 한다. [Espressif TWAI 중재 설명](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html)

노트북/ROS는 출력 메시지가 오지 않을 때 **오래된 마지막 출력을 현재 값으로 간주하지 않도록** 수신 시각 기반 timeout을 적용해야 한다. CAN 부하 자체는 실제 구동부의 안전 정지 수단이 아니다.

## 실험 순서

1. 실제 구동부를 분리하고 ESP32 A+B만 연결한다. 노트북→A→B 제어 수신 및 B→A→노트북 `0x180`+`0x181` 왕복을 확인한다.
2. B의 `0x001` E-stop 해석 제거와 Teensy의 **CAN1 단일 채널/스위치 ON 부하** 코드 변경을 완료한다. 물리 버스의 트랜시버 3개, 공통 GND, 종단저항 2개를 확인한다.
3. 스위치 OFF에서 Teensy dump 송신 0회 및 정상 UART `0x80` 수신을 확인한다. ON 구간에 Teensy dump 송신량과 B/A/노트북의 프레임률·지연을 기록한다. B `STATE_ESTOP`이 `0x001` payload 때문에 변하지 않아야 한다.
4. 스위치 OFF 복귀 때 새 dump 생성이 중단되는지, 이미 mailbox에 적재된 프레임이 몇 개 더 나가는지, 정상 출력 수신이 얼마나 빨리 회복되는지 확인한다.

기존 `teensy_can_load_estop/README.md`와 상위 `TEENSY_CAN_TRANSCEIVER_WIRING.md`는 **수정 전 2채널 E-stop 실험**을 설명한다. 새 구현 단계에서 README를 갱신하고, 이 가이드의 CAN1 핀만 새 배선에 적용한다.
