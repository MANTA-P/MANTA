# Teensy 4.1과 CAN 트랜시버 핀 연결

이 문서는 `teensy_can_load_estop` 시험 구성에서 사용하는 **Teensy 4.1**과
**SN65HVD230 3.3 V CAN 트랜시버 2개**의 연결을 정리한다.

## 전체 구성

- CAN1: CAN 부하(dump) 송신
- CAN2: E-stop 송신
- 통신 방식: Classical CAN, 500 kbit/s
- 두 CAN 채널에는 각각 별도의 SN65HVD230이 필요하다.
- 두 트랜시버의 `CANH`, `CANL`, `GND`는 같은 CAN 버스에 연결한다.

```text
Teensy 4.1 CAN1 ── SN65HVD230 #1 ──┐
                                    ├── CANH / CANL / GND 공통 버스
Teensy 4.1 CAN2 ── SN65HVD230 #2 ──┘
```

## Teensy 4.1 ↔ SN65HVD230 연결표

| 용도 | Teensy 4.1 핀 | 방향 | SN65HVD230 신호 | 모듈에서 흔히 쓰는 표기 |
|---|---:|:---:|---|---|
| CAN1 송신 | pin 22 (`CTX1`) | Teensy → 트랜시버 | `D` | `D`, `DI`, `TXD` |
| CAN1 수신 | pin 23 (`CRX1`) | Teensy ← 트랜시버 | `R` | `R`, `RO`, `RXD` |
| CAN2 송신 | pin 1 (`CTX2`) | Teensy → 트랜시버 | `D` | `D`, `DI`, `TXD` |
| CAN2 수신 | pin 0 (`CRX2`) | Teensy ← 트랜시버 | `R` | `R`, `RO`, `RXD` |
| 전원 | `3.3V` | Teensy → 트랜시버 | `VCC` | `3V3`, `VCC` |
| 기준 전위 | `GND` | 공통 | `GND` | `GND` |

> `TX`는 트랜시버의 입력인 `D/TXD`로, `RX`는 트랜시버의 출력인
> `R/RXD`로 연결한다. TX끼리 또는 RX끼리 연결한다는 의미가 아니다.

### CAN1 트랜시버 #1

```text
Teensy 4.1                  SN65HVD230 #1
pin 22  CTX1  -----------> D / TXD
pin 23  CRX1  <----------- R / RXD
3.3V              --------> VCC
GND               --------- GND
```

### CAN2 트랜시버 #2

```text
Teensy 4.1                  SN65HVD230 #2
pin 1   CTX2  -----------> D / TXD
pin 0   CRX2  <----------- R / RXD
3.3V              --------> VCC
GND               --------- GND
```

## SN65HVD230 IC 핀 번호

아래 번호는 SN65HVD230 8핀 IC의 top view 기준이다. 완제품 모듈은 헤더의
배치와 표기가 제조사마다 다르므로 모듈 실크 또는 회로도를 우선 확인한다.

| IC 핀 | 이름 | 연결 |
|---:|---|---|
| 1 | `D` | Teensy `CTX` |
| 2 | `GND` | 공통 GND |
| 3 | `VCC` | Teensy `3.3V` |
| 4 | `R` | Teensy `CRX` |
| 5 | `Vref` | 이 구성에서는 연결하지 않음 |
| 6 | `CANL` | 공통 CAN_L 버스 |
| 7 | `CANH` | 공통 CAN_H 버스 |
| 8 | `Rs` | High-speed(normal) 동작 시 GND에 연결 |

`Rs`에 10 kΩ~100 kΩ 저항을 거쳐 GND를 연결하면 slope-control 모드가 된다.
일반적인 짧은 시험 배선에서는 `Rs`를 GND에 직접 연결하는 high-speed 모드를
사용한다. `Rs`를 HIGH로 두면 저전력 대기 모드가 되므로 송신되지 않는다.

## CAN 버스 연결

| 트랜시버 #1 | 트랜시버 #2 | 외부 CAN 노드 |
|---|---|---|
| `CANH` | `CANH` | `CAN_H` |
| `CANL` | `CANL` | `CAN_L` |
| `GND` | `GND` | `GND` |

```text
              버스 한쪽 끝                         버스 반대쪽 끝
CAN_H ────────┬────────────────────────────────────────┬────────
              │                                        │
            120 Ω                                    120 Ω
              │                                        │
CAN_L ────────┴────────────────────────────────────────┴────────
```

- 120 Ω 종단저항은 각 트랜시버마다 다는 것이 아니라 **버스의 물리적인 양
  끝에만 하나씩**, 총 2개를 설치한다.
- 전원을 끈 상태에서 CAN_H와 CAN_L 사이를 측정하면 120 Ω 두 개의 병렬값인
  약 60 Ω이 정상이다.
- CAN_H와 CAN_L는 서로 바꾸지 말고, 가능하면 트위스트 페어로 배선한다.
- Teensy, 두 트랜시버, 외부 CAN 노드는 GND를 공통으로 연결한다.
- 트랜시버 IC 가까이에 VCC-GND 간 0.1 µF 디커플링 커패시터를 배치하는 것이
  좋다.

## E-stop 스위치 연결

CAN 배선과 별도로 E-stop 입력은 다음과 같이 연결한다.

```text
Teensy pin 2 ─── 스위치 ─── GND
```

pin 2는 펌웨어에서 `INPUT_PULLUP`과 Active LOW로 설정되어 있다. 따라서
스위치가 닫혀 pin 2가 GND에 연결되면 E-stop 활성 상태로 인식한다.

## 전원 투입 전 체크리스트

- [ ] 트랜시버가 SN65HVD230 또는 동등한 **3.3 V 로직 호환** 제품이다.
- [ ] CAN1은 pin 22(TX), pin 23(RX)에 연결했다.
- [ ] CAN2는 pin 1(TX), pin 0(RX)에 연결했다.
- [ ] Teensy TX는 트랜시버 `D/TXD`, Teensy RX는 `R/RXD`에 연결했다.
- [ ] 모든 노드의 GND가 공통이다.
- [ ] CAN_H끼리, CAN_L끼리 연결했다.
- [ ] 종단저항은 버스 양 끝에만 있고, 전원 OFF 측정값이 약 60 Ω이다.
- [ ] `Rs` 또는 모듈의 mode/standby 핀이 normal/high-speed 상태다.
- [ ] 모든 CAN 노드의 bitrate가 500 kbit/s로 같다.

## 관련 파일

- Teensy 설정: `teensy_can_load_estop/config.h`
- Teensy 펌웨어: `teensy_can_load_estop/teensy_can_load_estop.ino`
- 시험 설명: `teensy_can_load_estop/README.md`

SN65HVD230 IC의 핀 기능과 동작 모드는 Texas Instruments의
[SN65HVD23x 데이터시트](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)를
기준으로 작성했다.
