# Teensy 4.1 스위치·CAN1 배선

이 문서는 `teensy_can_load_estop.ino`의 현재 **스위치 구동 부하 시험**용
배선이다. pin 2 스위치는 CAN 신호선이 아니라 GPIO 입력이다. `0x001`
부하 프레임은 안전 명령이 아니다.

## 스위치

| 부품/신호 | 연결 |
|---|---|
| 2단자 누름 스위치의 한쪽 | Teensy 4.1 디지털 pin **2** |
| 스위치의 다른 쪽 | Teensy **GND** |

```text
Teensy 4.1 pin 2 ───── 스위치 ───── Teensy GND
```

펌웨어는 pin 2를 `INPUT_PULLUP`으로 설정한다. 스위치가 열려 있으면
`HIGH`라서 부하 OFF, 누르거나 닫혀 pin 2가 GND와 연결되면 `LOW`라서
50 ms debounce 후 부하 ON이다. 스위치에 3.3 V나 5 V를 직접 연결하지
않는다. 이 설명은 **눌렀을 때 접점이 닫히는** 스위치 기준이다. 접점이
반대로 동작하는 스위치는 실제 접점 상태를 확인한 후 배선/입력 논리를
맞춰야 한다.

## CAN1 트랜시버

Teensy의 CAN 핀을 CAN_H/CAN_L에 직접 연결하지 않는다. 3.3 V 로직 호환
CAN 트랜시버 1개가 필요하다. SN65HVD230 기준 연결은 다음과 같다.

| Teensy 4.1 | SN65HVD230 신호 | IC 핀 | 설명 |
|---|---|---:|---|
| pin **22** (`CTX1`) | `D` / `TXD` | 1 | Teensy 송신 → 트랜시버 입력 |
| pin **23** (`CRX1`) | `R` / `RXD` | 4 | 트랜시버 출력 → Teensy 수신 |
| `3.3V` | `VCC` | 3 | 트랜시버 전원 |
| `GND` | `GND` | 2 | 공통 기준 전위 |
| — | `Rs` | 8 | 일반 high-speed 동작이면 GND |
| 버스 `CAN_L` | `CANL` | 6 | 다른 노드의 CAN_L와 연결 |
| 버스 `CAN_H` | `CANH` | 7 | 다른 노드의 CAN_H와 연결 |

SN65HVD230의 `Vref`(IC 핀 5)는 이 구성에서 사용하지 않는다. 모듈의
핀 배치와 `Rs`/mode 표기는 제품마다 다르므로 실크와 회로도를 확인한다.
CAN1만 사용하며 Teensy pin **0/1(CAN2)**과 두 번째 트랜시버는 연결하지
않는다.

```text
Teensy pin 22 CTX1 ───> D/TXD  ┌──────────────┐
Teensy pin 23 CRX1 <─── R/RXD  │ SN65HVD230   │── CAN_H → 공통 버스
Teensy 3.3V       ───> VCC     │ CAN1용 1개   │── CAN_L → 공통 버스
Teensy GND        ──── GND     └──────────────┘
```

ESP32 A, ESP32 B, Teensy CAN1을 함께 연결할 때는 세 노드의 CAN_H끼리,
CAN_L끼리, GND끼리 연결하고 모든 CAN 장치를 500 kbit/s로 맞춘다.
120 Ω 종단저항은 버스의 **물리적 양 끝에만** 하나씩 둔다. 전원을 끈 상태에서
CAN_H와 CAN_L 사이를 재면 두 종단의 병렬값인 약 60 Ω이 예상된다.

ESP32 B가 아직 ID `0x001`을 E-stop으로 해석한다면 세 보드를 같은 버스에
연결해 부하 시험을 하지 않는다. 우선 Teensy와 별도 ACK 노드만으로 스위치
ON/OFF를 확인한다. 실제 구동부 없이 시험한다.

핀과 트랜시버 모드는 [PJRC Teensy 4.1 자료](https://www.pjrc.com/store/teensy41.html)와
[TI SN65HVD230 데이터시트](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)를
참고한다.
