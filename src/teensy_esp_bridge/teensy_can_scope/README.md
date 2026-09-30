# Teensy 고정 CAN 파형 오실로스코프 측정

이 스케치는 단일 채널 오실로스코프로 CAN_H와 CAN_L을 차례로 측정한 뒤 두
파형을 겹쳐 보기 위한 반복 신호원이다. 다음과 같은 **완전히 동일한 Classical
CAN 프레임**을 20 ms마다 전송한다.

| 항목 | 값 |
|---|---|
| CAN 속도 | 500 kbit/s (1 bit = 2 us) |
| 형식 | 11-bit Standard Data Frame |
| CAN ID | `0x123` |
| DLC | 8 |
| Payload | `55 AA 00 FF 12 34 56 78` |
| 반복 주기 | 20 ms (50 frame/s) |

ID와 payload가 변하지 않으므로 CRC와 bit stuffing 위치까지 매 프레임 동일하다.
코드는 `teensy_can_scope.ino`에 있으며 기존 E-stop 시험 코드와 독립적이다.

## CAN에는 ACK 노드가 필요하다

정상적인 CAN 송신에는 송신기 외에 프레임을 수신하고 ACK bit를 만드는 활성
노드가 하나 이상 필요하다. 송신기 하나만 연결하면 ACK error, 자동 재전송,
bus-off가 발생하여 20 ms 반복 파형이 유지되지 않는다.

이 스케치는 Teensy 4.1의 독립된 CAN 컨트롤러 두 개를 사용한다.

- CAN1: 고정 프레임 송신
- CAN2: 같은 버스에서 프레임 수신 및 ACK

따라서 Teensy 한 대와 CAN 트랜시버 두 개만으로 측정할 수 있다. CAN1과 CAN2는
같은 MCU 안에 있지만 서로 독립된 CAN 노드다.

## 준비물

- Teensy 4.1
- 3.3 V 로직 호환 CAN 트랜시버 2개
  - 예: SN65HVD230 모듈 2개
- 120 ohm 종단저항 2개
- CAN_H/CAN_L용 배선, 공통 GND 배선
- 오실로스코프와 10x 프로브
- Arduino IDE와 Teensyduino

5 V 전용 CAN 모듈은 Teensy 4.1의 3.3 V GPIO에 바로 연결하지 않는다. 모듈마다
핀 순서와 내장 종단저항 유무가 다르므로 실크와 회로도를 먼저 확인한다.

## Teensy와 트랜시버 연결

### CAN1 송신 트랜시버

| Teensy 4.1 | 기능 | SN65HVD230 모듈 |
|---:|---|---|
| pin 22 | `CTX1`, 송신 출력 | `D`, `DI` 또는 `TXD` |
| pin 23 | `CRX1`, 수신 입력 | `R`, `RO` 또는 `RXD` |
| 3.3V | 전원 | `VCC` 또는 `3V3` |
| GND | 기준 전위 | `GND` |

### CAN2 ACK 트랜시버

| Teensy 4.1 | 기능 | SN65HVD230 모듈 |
|---:|---|---|
| pin 1 | `CTX2`, 송신 출력 | `D`, `DI` 또는 `TXD` |
| pin 0 | `CRX2`, 수신 입력 | `R`, `RO` 또는 `RXD` |
| 3.3V | 전원 | `VCC` 또는 `3V3` |
| GND | 기준 전위 | `GND` |

트랜시버의 MCU 쪽 연결은 TX끼리 연결하는 방식이 아니다. Teensy의 `CTX`는
트랜시버 입력 `D/TXD`로, 트랜시버 출력 `R/RXD`는 Teensy의 `CRX`로 연결한다.

SN65HVD230의 `Rs`는 GND에 연결하여 normal/high-speed mode로 둔다. 모듈에
`STB`, `S`, `EN` 또는 mode 핀이 있다면 데이터시트에 따라 normal mode로
설정한다.

## 공통 CAN 버스와 종단저항

```text
Teensy CAN1                     Transceiver #1
pin 22 CTX1 ------------------> TXD
pin 23 CRX1 <------------------ RXD
                                 CANH ---- CAN_H BUS ---- CANH
                                 CANL ---- CAN_L BUS ---- CANL
                                                        Transceiver #2
Teensy CAN2                                            TXD <---- pin 1 CTX2
                                                       RXD ----> pin 0 CRX2

Scope tip: CAN_H 측정점 또는 CAN_L 측정점

Teensy GND ---- Transceiver #1 GND ---- Transceiver #2 GND ---- Scope GND
```

두 트랜시버의 `CANH`끼리, `CANL`끼리 연결한다. 짧은 시험 버스의 양 끝에는
CAN_H와 CAN_L 사이에 각각 120 ohm을 연결한다.

```text
버스 끝 A                                             버스 끝 B
CAN_H o-------------+-----------------------------------+-------------o
                    |                                   |
                  120 ohm                             120 ohm
                    |                                   |
CAN_L o-------------+-----------------------------------+-------------o
```

전원을 끈 상태에서 CAN_H-CAN_L 사이 저항을 측정했을 때 약 60 ohm이어야 한다.
일부 모듈에는 120 ohm 저항이 이미 있으므로 중복 설치하지 않는다. 두
트랜시버와 Teensy의 GND도 반드시 공통으로 연결한다.

## Arduino IDE로 플래시

1. Arduino IDE를 설치한다.
2. PJRC Teensyduino를 설치하고 Arduino IDE에 Teensy 보드 지원을 추가한다.
3. Arduino IDE에서 `teensy_can_scope.ino`를 연다.
4. `Tools > Board`에서 `Teensy 4.1`을 선택한다.
5. `Tools > USB Type`은 `Serial`을 선택한다.
6. Teensy를 USB로 연결하고 해당 포트를 선택한다.
7. Verify로 컴파일한 뒤 Upload를 누른다.
8. 자동 업로드가 시작되지 않으면 Teensy의 물리 Program 버튼을 한 번 누른다.
9. Serial Monitor를 115200 baud로 열어 다음 시작 메시지를 확인한다.

```text
Teensy fixed CAN oscilloscope source started
CAN1 TX pin=22 RX pin=23, CAN2 ACK TX pin=1 RX pin=0
bitrate=500000 id=0x123 period_us=20000 payload=55 AA 00 FF 12 34 56 78
```

이후 1초마다 누적 송수신 상태가 출력된다. 정상이라면 `tx_complete`와 `rx`가
같은 속도로 약 50씩 증가하고 `mismatch=0`, 마지막 표시가 `OK`여야 한다.

```text
STAT tx_complete=45 rx=45 mismatch=0 in_flight=0 OK
STAT tx_complete=95 rx=95 mismatch=0 in_flight=0 OK
```

출력 순간의 callback 순서 때문에 드물게 한 줄만 `CHECK`가 나올 수 있다. 다음
줄에서 두 카운터가 같아져 `OK`가 되면 정상이다. `tx_complete`가 증가하지 않으면
ACK 노드, 트랜시버, 비트레이트 또는 종단 배선을 점검한다. `rx`만 증가하지
않으면 CAN2 RX 배선과 mailbox 설정을 점검한다.

`FlexCAN_T4`는 일반적인 Teensyduino 설치에 포함된다. 컴파일 시
`FlexCAN_T4.h: No such file or directory`가 나오면 Teensyduino 설치 상태와 선택한
보드가 Teensy 4.1인지 확인한다.

## 오실로스코프 측정 절차

프로브는 DC coupling, 10x 감쇠를 권장한다. 500 kbit/s의 한 bit는 2 us이므로
처음에는 20~50 us/div 정도로 프레임 전체를 찾고, bit 모양은 1~2 us/div로
확대한다. 샘플레이트는 가능하면 10 MS/s 이상, 가급적 50 MS/s 이상으로 둔다.

### 1. CAN_H 저장

1. 프로브 ground clip을 **공통 GND**에 연결한다.
2. probe tip을 CAN_H에 연결한다.
3. trigger를 rising edge, 약 2.7~3.0 V로 설정한다. 실제 recessive/dominant
   전압의 중간값에 맞춰 조정한다.
4. 정상 프레임을 잡고 파형 또는 CSV를 저장한다.

### 2. CAN_L 저장

1. ground clip은 그대로 공통 GND에 둔다.
2. probe tip만 CAN_L로 옮긴다.
3. trigger를 falling edge, 약 1.7~2.1 V로 설정한다. 실제 두 전압의 중간값에
   맞춰 조정한다.
4. 첫 dominant edge가 CAN_H 측정과 같은 화면 위치에 오도록 trigger position과
   timebase를 동일하게 유지한다.
5. 파형 또는 CSV를 저장한다.

> 오실로스코프 ground clip을 CAN_L이나 CAN_H에 연결하면 안 된다. 접지형
> 오실로스코프에서는 해당 선을 보호접지에 단락시켜 통신 불량이나 장비 손상을
> 일으킬 수 있다. ground clip은 반드시 버스 공통 GND에 연결한다.

두 파형을 겹치면 dominant 상태에서 CAN_H는 약 3.5 V 방향으로 올라가고 CAN_L은
약 1.5 V 방향으로 내려가는 상보 파형이 보여야 한다. 실제 전압은 트랜시버,
종단, 프로브와 배선 조건에 따라 달라진다.

CSV 후처리로 차동 파형을 만들 때는 동일한 시간축으로 정렬한 다음 다음 값을
계산한다.

```text
Vdiff = V(CAN_H) - V(CAN_L)
```

순차 측정은 두 전압을 진짜 동시 측정한 것이 아니므로 간헐 잡음, jitter,
arbitration 충돌 분석에는 적합하지 않다. 이 스케치처럼 동일 프레임을 반복하는
정상 파형 확인에만 사용한다.

## 파형이 반복되지 않을 때

- CAN_H-CAN_L 전원 OFF 저항이 약 60 ohm인지 확인한다.
- CAN1과 CAN2가 모두 500 kbit/s인지 확인한다.
- 두 트랜시버가 normal mode인지 확인한다.
- CAN_H/CAN_L가 뒤바뀌지 않았는지 확인한다.
- CAN2 트랜시버의 RXD가 Teensy pin 0에 연결됐는지 확인한다.
- 다른 CAN 송신 장치는 분리한다. 다른 ID가 섞이면 스코프 trigger가 흔들릴 수
  있다.
- Serial 시작 문구가 보이는지 확인한다.

반복 주기, CAN ID 또는 payload를 바꾸려면 스케치 상단의 `kFramePeriodUs`,
`kCanId`, `payload`를 수정한다.
