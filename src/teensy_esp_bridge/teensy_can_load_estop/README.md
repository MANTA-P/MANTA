# Teensy 4.1 스위치 구동 CAN1 부하 시험

이 스케치는 스위치를 누른 동안에만 CAN ID `0x001`의 **시험용 부하 프레임**을
생성한다. 이 ID와 payload는 E-stop 명령이 아니다. 과거 E-stop 우선순위 시험
구성과 결과는 `../TEENSY_CAN_LOAD_ESTOP_HANDOFF.md`,
`../TEENSY_CAN_LOAD_ESTOP_RESULTS.md`에 남아 있다.

## 현재 동작

- 스위치: Teensy pin 2와 GND 사이, `INPUT_PULLUP`, Active LOW.
- 입력은 `CHANGE` 인터럽트로 기록하고 메인 루프에서 50 ms debounce한다.
- 누른 채 부팅하면 debounce 후 송신한다. 뗀 채 부팅하면 송신하지 않는다.
- 눌린 동안 CAN1(TX 22/RX 23)에서 Classical CAN 500 kbit/s, 표준 ID
  `0x001`, DLC 8 프레임을 200 us 생성 주기로 시도한다.
- CAN1 mailbox 8과 9를 사용한다. 두 mailbox가 바쁘면 RAM에는 최신 프레임
  하나만 남기고 오래된 생성 프레임은 버린다.
- 스위치를 놓으면 새 프레임의 생성과 적재를 멈춘다. 이미 mailbox에 적재된
  최대 2개 프레임은 이후 완료되거나 timeout 복구 시 결과가 미확인될 수 있다. `LOAD off`와
  `DRAIN` 로그에서 해제 시 잔여 수와 마지막 완료 시각을 확인한다.
- CAN1 송신 완료가 100 ms 동안 없거나 bus-off가 관측되면 250 ms 뒤 CAN1을
  재초기화한다. `dump_tx_timeout`, `dump_tx_unconfirmed`, `can_recoveries`를 기록한다.
  설치된 Teensyduino 1.62.0의 `FlexCAN_T4`에 내부 TX 큐를 버리는 공개 API가
  없어 mailbox의 실제 유휴 상태를 확인한 후 송신한다. 예기치 않게
  `can_tx_queue`가 0보다 커지면 재초기화하지 않고 송신을 멈춘다.

## 프레임과 로그

| Byte | 내용 |
|---:|---|
| 0..3 | 생성 sequence, `uint32`, big-endian |
| 4..7 | 생성 시각 `micros()`, `uint32`, big-endian |

1초마다 `STAT`에 스위치 상태, 에지/전환 수, 마지막 ON/OFF 시각,
`dump_generated`, `dump_enqueued`, `dump_tx_success`, `dump_dropped`,
`dump_in_flight`, CAN 오류를 출력한다. `FIRST_TX`는 ON 확정부터 첫 송신 완료까지,
`DRAIN`은 OFF 확정 뒤 남은 프레임의 완료/미확인 수와 마지막 완료까지 걸린
시간을 출력한다. 송신 완료 수는 ESP32나 노트북의 수신 성공 수와 다르다.

## 배선과 시험

현재 필요한 Teensy 측 배선은 [SWITCH_CAN_WIRING.md](SWITCH_CAN_WIRING.md)를
따른다. **CAN2 pin 0/1과 두 번째 트랜시버는 사용하지 않는다.**

Arduino IDE에서 보드를 `Teensy 4.1`로 선택해 이 폴더의
`teensy_can_load_estop.ino`를 빌드·업로드한다. 먼저 다른 보드와 분리된
시험 버스에서 OFF 부팅, ON 부팅, ON/OFF 반복과 해제 후 잔여 프레임을
확인한다. ESP32 B가 `0x001`을 E-stop으로 처리하는 기존 코드를 제거하기
전에는 A/B와 공통 버스에 연결하지 않는다. 실제 구동부 없이 시험한다.
