# ESP32 2보드 양방향 UART ↔ CAN 브리지 계획

## 1. 목표

동일한 펌웨어를 ESP32 보드 2대에 올리고, 각 보드가 UART와 CAN 사이에서 문자 1 byte를 양방향으로 전달한다.

```text
노트북 A <-> UART <-> ESP32 A <-> CAN bus <-> ESP32 B <-> UART <-> 노트북 B
```

각 ESP32는 다음 두 기능을 모두 수행한다.

1. 자기 노트북의 UART 문자 1 byte를 CAN으로 송신한다.
2. CAN에서 받은 문자 1 byte를 자기 노트북의 UART로 송신한다.

## 2. 보드별 동작

ESP32 A와 ESP32 B는 역할과 펌웨어가 같다.

| 데이터 흐름 | ESP32 A 동작 | ESP32 B 동작 |
| --- | --- | --- |
| 노트북 A → 노트북 B | UART 수신 후 CAN 송신 | CAN 수신 후 UART 송신 |
| 노트북 B → 노트북 A | CAN 수신 후 UART 송신 | UART 수신 후 CAN 송신 |

## 3. 공통 CAN 데이터 규격

| 항목 | 값 |
| --- | --- |
| CAN 형식 | 표준 11-bit data frame |
| CAN ID | `0x200` (초기값) |
| bitrate | 500 kbit/s |
| DLC | 1 |
| payload[0] | UART 문자 1 byte의 ASCII 값 |

예: 노트북 A가 `A`(ASCII `0x41`)를 보내면 ESP32 A는 CAN ID `0x200`, DLC `1`, `payload[0]=0x41`을 송신한다. ESP32 B는 이 byte를 UART로 전송하고 노트북 B는 `A`를 받는다. 반대 방향도 같은 방식으로 동작한다.

## 4. 구현 순서

### P0. CAN 하드웨어 연결 검증

- 두 ESP32에 CAN transceiver를 연결한다.
- 두 transceiver의 CANH, CANL, GND를 연결한다.
- CAN 버스 양 끝에 120 Ω 종단저항을 둔다.
- 두 보드의 CAN bitrate를 모두 500 kbit/s로 맞춘다.
- 고정 counter frame으로 양방향 CAN 송수신을 먼저 확인한다.

완료 기준: 두 ESP32가 CAN frame을 서로 송신하고 수신할 수 있다.

### P1. UART → CAN 송신 경로 유지

- UART1 RX(GPIO16)에서 문자 1 byte를 받는다.
- CAN ID `0x200`, DLC 1, `payload[0]=수신 문자`로 CAN frame을 송신한다.
- 송신 성공·실패와 전송 byte를 serial log에 남긴다.

완료 기준: 각 보드에서 UART로 보낸 문자가 CAN bus에서 동일한 byte로 관측된다.

### P2. CAN → UART 수신 경로 추가

- TWAI CAN 수신 callback 또는 queue를 설정한다.
- 표준 data frame, CAN ID `0x200`, DLC 1인 frame만 처리한다.
- `payload[0]`을 UART1 TX(GPIO15)로 전송한다.
- 수신 CAN ID, byte 값, UART write 결과를 serial log에 남긴다.

완료 기준: 상대 보드가 보낸 CAN `payload[0]`이 로컬 노트북 UART에 같은 byte로 도착한다.

### P3. 동시 양방향 통신 시험

- 노트북 A와 B에서 동시에 여러 문자를 보낸다.
- 각 노트북이 상대 노트북의 문자를 순서대로 받는지 확인한다.
- CAN 케이블 분리, CAN ID 불일치, DLC 0, UART 연결 해제를 시험한다.
- 오류 frame과 자기 보드가 보낸 frame을 UART로 되돌려 보내지 않는지 확인한다.

완료 기준: 양방향 문자 전달이 동시에 동작하고, echo loop 없이 정상 frame만 반대편으로 전달된다.

## 5. Echo loop 방지 정책

CAN 수신 데이터를 UART로 보내는 것만 수행하고, UART로 들어온 byte만 CAN으로 보낸다. CAN 수신 데이터를 다시 CAN으로 보내지 않으므로 CAN bus에서 무한 반복(echo loop)이 발생하지 않는다.

노트북 프로그램이 UART로 받은 데이터를 자동으로 다시 같은 UART에 쓰지 않도록 해야 한다.

## 6. 구현 전 확인할 정보

1. 두 ESP32의 보드 모델과 CAN TX/RX GPIO가 동일한가?
2. 두 노트북은 각각 별도 USB-UART 어댑터로 ESP32 UART1에 연결되는가?
3. UART baudrate는 두 보드 모두 115200으로 사용할 것인가?
4. `0x200` CAN ID와 500 kbit/s bitrate를 초기 시험값으로 사용해도 되는가?
5. 두 보드가 같은 CAN ID를 송신해도 되는지, 보드별 송신 ID를 분리할지 결정해야 한다.
