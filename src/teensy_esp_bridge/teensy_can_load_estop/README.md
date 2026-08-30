# Teensy 4.1 CAN load and E-stop test

`TEENSY_CAN_LOAD_ESTOP_HANDOFF.md`의 시험 구성을 구현한 Arduino/Teensy
스케치다.

단일/2-mailbox 정상·역전 시험 결과는
`../TEENSY_CAN_LOAD_ESTOP_RESULTS.md`에 정리되어 있다.

## 기본 설정

- Teensy 4.1
- Classical CAN 500 kbit/s
- CAN1: dump 전용, TX 22/RX 23
- CAN2: E-stop 전용, TX 1/RX 0
- E-stop 스위치: pin 2와 GND 사이
- 스위치 입력: `INPUT_PULLUP`, Active LOW
- debounce: 50 ms
- dump 생성 주기: 200 us
- E-stop: 3회, 성공한 송신 완료 후 1 ms 간격
- 기본 정상 모드: E-stop ID `0x001`, dump ID `0x700`

값은 `config.h`에서 변경한다. `kPriorityInverted=true`로 설정하면 E-stop과
dump ID가 서로 바뀐다. 실제 구동부와 분리된 시험 환경에서만 역전 모드를
사용한다.

## 배선

CAN1과 CAN2에는 각각 별도의 3.3 V SN65HVD230 트랜시버가 필요하다.

```text
Teensy 4.1 CAN1             SN65HVD230 #1
pin 22 CTX1  ------------> D/TXD
pin 23 CRX1  <------------ R/RXD
3.3V/GND      ------------ VCC/GND

Teensy 4.1 CAN2             SN65HVD230 #2
pin 1 CTX2   -------------> D/TXD
pin 0 CRX2   <------------- R/RXD
3.3V/GND      ------------ VCC/GND

두 트랜시버 CANH ---------- 공통 CAN_H 버스
두 트랜시버 CANL ---------- 공통 CAN_L 버스
두 트랜시버 GND ----------- 공통 GND

Teensy pin 2 ---- 스위치 ---- GND
```

SN65HVD230의 `Rs` 핀 또는 모듈의 mode 핀은 Normal/High-speed 동작이 되도록
설정한다. 120 Ω 종단저항은 공통 버스의 물리적인 양 끝에만 설치한다.

## Payload

모든 multi-byte 값은 big-endian이다.

Dump, DLC 8:

| Byte | 값 |
|---:|---|
| 0..3 | 생성 sequence, uint32 |
| 4..7 | 생성 시점 `micros()`, uint32 |

E-stop, DLC 8:

| Byte | 값 |
|---:|---|
| 0 | `0x01` active, `0x00` released |
| 1 | `0x00` 정상, `0x01` 우선순위 역전 |
| 2..3 | E-stop sequence, uint16 |
| 4..7 | debounce 확정 `micros()`, uint32 |

## 구현 특성

- dump는 CAN1 mailbox 8과 9를 ping-pong 방식으로 직접 사용하며 소프트웨어 TX
  queue를 사용하지 않는다. 한 mailbox가 송신되는 동안 다른 mailbox에 다음
  프레임을 대기시켜 다음 중재 시점의 공백을 줄인다. 두 mailbox가 모두 바쁜
  동안 생성된 값은 RAM에서 최신 한 개만 유지한다.
- `STAT`의 `dump_in_flight`는 현재 CAN1 하드웨어에 적재된 dump mailbox 수다.
  포화 구간에서는 대부분 `2`가 유지되는 것이 예상된다.
- E-stop은 CAN2 mailbox 8과 64-event 전용 queue를 사용한다.
- 스위치 ISR은 raw level과 `micros()`만 저장한다.
- 송신 성공 통계는 `onTransmit` callback에서만 증가한다.
- 프레임별 로그는 출력하지 않고 1초마다 `STAT`, 완료 이벤트가 있으면 `LAT`
  통계를 출력한다.

## 업로드

Arduino IDE에서 보드를 `Teensy 4.1`로 선택하고
`teensy_can_load_estop.ino`를 연 뒤 업로드한다. Teensyduino에 포함된
`FlexCAN_T4`를 사용한다.

첫 시험에서는 `config.h`의 `kDumpEnabled=false`로 설정하고 E-stop 20회를
검증한다. 이후 dump를 활성화해 정상 모드를 시험하고 마지막으로 실제 구동부와
분리한 상태에서 역전 모드를 시험한다.
