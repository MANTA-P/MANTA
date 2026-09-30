# AI 구현자용: Teensy 스위치 구동 고우선순위 CAN 부하 명세

## 역할

Teensy 4.1은 이 실험에서 E-stop 명령 송신기가 아니다. **pin 2 스위치가 눌린 동안에만** CAN ID `0x001`의 부하 프레임을 가능한 연속적으로 송신해 ESP32 B→A→노트북 출력 경로의 지연·수신률을 측정한다. 스위치를 놓으면 새 부하 프레임 생성을 중단한다. `0x001`을 안전 신호로 해석하지 않는다. ESP32 쪽 필수 변경은 [`ESP32_CAN_CHANGE_SPEC.md`](../../temp/ESP32_CAN_CHANGE_SPEC.md), 새 3노드 배선은 [`CAN_3BOARD_WIRING_GUIDE.md`](../../temp/CAN_3BOARD_WIRING_GUIDE.md)를 따른다. 이 문서는 코드 변경 요구사항이며 아직 스케치를 수정한 기록이 아니다.

## 현행 구현에서 바꿀 구조

작업 대상: 이 디렉터리의 `teensy_can_load_estop.ino`, `config.h`, `README.md`. `backup/*.bak`는 비교용이며 수정·빌드하지 않는다.

현행 스케치는 CAN1의 dump `0x700`을 `kDumpEnabled=true`일 때 부팅 직후부터 송신하고, CAN2의 E-stop `0x001`을 스위치 변화 시 송신한다. `kPriorityInverted`는 둘의 ID를 바꾸는 시험 옵션이다. 새 구조에서는 **CAN1 하나만 사용**한다. CAN2 객체, `estopCan` 초기화·mailbox·이벤트 큐·3회 반복·E-stop payload·관련 통계 및 `LAT` 출력은 제거한다. CAN2용 트랜시버와 pin 0/1 배선도 새 실험에는 필요하지 않다. 스위치는 CAN2가 아니라 Teensy GPIO pin 2 입력으로 남는다.

## P0 — 스위치로 CAN1 부하 게이트

1. pin 2를 `INPUT_PULLUP`, Active LOW로 유지하고 `CHANGE` 인터럽트로 에지를 감지한다. ISR은 현재 raw level과 `micros()`만 기록한다. CAN 전송, Serial 출력, 긴 계산을 ISR 안에서 하지 않는다. 메인 루프에서 기존 50 ms debounce를 적용한 **확정된 스위치 상태**로 부하 ON/OFF를 결정한다. 스위치를 누른 채 부팅해도 debounce 후 ON으로 진입하고, 뗀 채 부팅하면 절대 부하를 송신하지 않는다.
2. OFF→ON 전환 때 `nextDumpGenerationUs`를 현재 시각으로 재설정해 곧바로 첫 프레임을 생성한다. ON 동안 기존 CAN1 MB8/MB9 ping-pong 및 최신 1개 pending 정책을 사용할 수 있다. 송신 성공 콜백을 사용해 실제 버스에 나간 프레임 수를 센다. 두 mailbox를 계속 채워 버스 사용률을 높이되, `loop()`을 무한 busy-wait로 막지 않는다.
3. ON→OFF 전환 즉시 새 dump 생성·enqueue를 멈추고 RAM의 `latestDumpPending`을 비운다. 이미 CAN 컨트롤러 mailbox에 실린 프레임은 완성되거나 라이브러리가 지원하는 안전한 취소 절차를 거쳐야 한다. **스위치 해제 순간 버스에서 0프레임이 된다고 주장하지 말고**, 마지막 완료 프레임 시각과 잔여 mailbox 수를 기록한다. 다음 ON에 지난 주기의 프레임이 재사용되지 않도록 sequence·스케줄러 상태를 정리한다.
4. 전송 완료 콜백이 오지 않거나 CAN1 bus-off이면 mailbox 점유 상태가 무기한 참으로 남을 수 있다. 전송 상태 timeout과 오류 카운터를 마련하고, 설치된 `FlexCAN_T4` 버전의 실제 API를 확인한 뒤 복구한다. 송신 중 mailbox를 임의로 덮어쓰지 않는다.

검증: 부팅 시 OFF→0프레임; 부팅 시 ON→debounce 후 송신; 20회 ON/OFF 반복에서 OFF 상태 새 생성/적재 0회; 해제 뒤 드레인된 잔여 프레임 수와 시간 기록; 재누름 때 즉시 새로운 sequence 송신.

## P0 — ID와 payload를 부하 전용으로 고정

- CAN1 dump ID를 `0x001`로 고정한다. 이것은 현재 시스템 ID `0x010`, `0x110`, `0x120`, `0x180`, `0x181`, `0x1F0`보다 낮아 중재 우선순위가 가장 높다. `kPriorityInverted`, `kNormalEstopId`, `kNormalDumpId`와 E-stop 메시지 형식은 제거한다. 실험에 구 ID `0x700` 전송이 남지 않아야 한다.
- 표준 11-bit 데이터 프레임, DLC 8을 유지한다. 기존 dump payload인 big-endian `uint32 sequence`(byte 0..3)와 `uint32 micros()` 생성 시각(byte 4..7)을 유지하면 버스 캡처로 프레임 순서와 발생 시간을 추적할 수 있다. payload는 제어 명령이 아니다.
- `config.h`의 `kDumpPeriodUs=200`은 초당 5,000개 생성 목표로 500 kbit/s의 8바이트 프레임 버스 용량보다 높다. 이는 포화 시험용 값이다. 초과 프레임은 기존 최신 1개 보존 정책으로 버리고, `dump_generated`/`dump_dropped`/`dump_tx_success`를 분리해 표시한다. 버스에서 실제로 차지한 비율과 B의 출력 지연은 측정으로 판단한다.

**선행 조건:** ESP32 B의 현재 `CAN_ID_ESTOP=0x001` 처리부가 남아 있으면 이 payload를 E-stop으로 오해할 수 있다. B 수정이 끝나기 전에는 세 보드를 한 버스에 연결해 부하 시험을 하지 않는다.

## P1 — 통계 및 실험 로그

- 1초 `STAT`에 `load_switch=on/off`, raw/debounced 에지 수, ON/OFF 전환 시각, `dump_generated`, `dump_enqueued`, `dump_tx_success`, `dump_dropped`, `dump_in_flight`, CAN 오류/bus-off를 출력한다. 기존 `estop_*` 통계와 E-stop `LAT`는 제거한다.
- ON 직후 첫 프레임 송신 완료까지의 지연과 OFF 후 마지막 프레임 완료까지의 드레인 지연을 기록한다. CAN 송신 성공은 ESP32 A나 노트북의 수신 성공과 다르므로 노트북 수신 통계와 시간축을 맞춘다.
- 이 방식은 동일 버스의 **A→B 입력도 방해**한다. 높은 우선순위는 동시에 시작하는 프레임의 중재를 이기지만 이미 전송 중인 프레임을 중단시키지 않으며, 구현·스케줄링에 따라 낮은 우선순위 프레임이 일부 통과할 수 있다. 특정 B→A 출력의 100% 차단은 보장하지 않는다. [Espressif TWAI 중재 설명](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html)

## 시험 순서

1. Teensy 단독/분리된 CAN 시험에서 OFF 부팅 0프레임, ON 부팅 및 ON/OFF 20회 반복을 확인한다.
2. B의 `0x001` E-stop 처리 제거를 확인한 뒤 ESP32 A/B와 같은 500 kbit/s 버스에 연결한다. 물리 노드는 A, B, Teensy CAN1의 3개, 트랜시버도 3개다.
3. OFF/ON/OFF 구간에 Teensy `STAT`, B의 `0x180/0x181` 송신 성공/실패, A의 페어 완성, 노트북 UART `0x80` 수신률과 지연을 비교한다. A→B 제어/오도메트리 수신률도 함께 기록한다.
4. 해제 뒤 정상 왕복 경로가 얼마나 빨리 회복되는지 확인한다. 실제 구동부 없이 시험한다.

참조: `config.h`, `teensy_can_load_estop.ino`, `README.md`; 백업은 `backup/`.
