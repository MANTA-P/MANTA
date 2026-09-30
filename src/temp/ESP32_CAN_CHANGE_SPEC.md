# AI 구현자용: ESP32 A/B의 CAN 우선순위 부하 시험 수정 명세

## 목표와 범위

노트북은 UART 바이너리 프로토콜로 ESP32 A에 명령·오도메트리를 보낸다. A는 이를 CAN `0x010`/`0x110`/`0x120`으로 ESP32 B에 전달한다. B는 현재 제어 로직의 출력을 CAN `0x180`/`0x181`과 상태 `0x1F0`으로 A에 돌려주고, A는 `0x180`+`0x181`을 묶어 UART `0x80`, 상태를 UART `0x81`로 노트북에 보낸다. 장차 B의 제어 계산이 업데이트되어도 이 왕복 경로를 유지한다.

Teensy는 **스위치가 눌린 동안에만** 표준 ID `0x001`의 고우선순위 **시험용 부하 프레임**을 연속 송신한다. 스위치를 놓으면 새 부하 프레임을 만들지 않는다. `0x001`은 E-stop 명령이 아니다. 목표는 이 부하가 B→A→노트북 경로의 출력 프레임 수신율·지연에 주는 영향을 측정하는 것이다. Teensy의 작업은 [Teensy 수정 명세](../teensy_esp_bridge/teensy_can_load_estop/TEENSY_CAN_CHANGE_SPEC.md), 배선은 [3보드 연결 가이드](CAN_3BOARD_WIRING_GUIDE.md)를 따른다. 아래에는 수정 전 문제와 구현 기준을 기록한다.

## CAN에서 가능한 범위

`0x001`은 이 시스템의 `0x010`, `0x110`, `0x120`, `0x180`, `0x181`, `0x1F0`보다 중재 우선순위가 높다. **중재는 동일 버스의 모든 송신 노드에 적용**되므로 A→B 입력도 B→A 출력과 함께 지연될 수 있다. 이미 시작한 CAN 프레임을 중간에 끊을 수 없고, 부하 프레임이 다음 중재 시점에 준비되지 않으면 낮은 우선순위 프레임이 전송될 수 있다. 따라서 `0x180/0x181`의 100% 차단을 요구사항으로 쓰지 말고, 누른/뗀 구간의 성공률과 지연을 관측한다. 다른 방향을 유지하면서 B→A만 막아야 한다면 별도 CAN 세그먼트나 게이트웨이 설계가 필요하다. [Espressif TWAI 중재 설명](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html)

## P0 — B에서 `0x001`의 E-stop 해석 제거

대상: `esp32b_fw/main/can_protocol.c`.

현재 `CAN_ID_ESTOP=0x001` 분기는 payload의 `data[2] == 1` 또는 `E5 7A`를 E-stop으로 해석하고 `controller_set_estop()`을 호출한다. 새 부하 프레임의 sequence/timestamp와 우연히 일치하면 B 출력이 0으로 바뀌어 **우선순위 실험 결과가 왜곡**된다. 이 시험 빌드에서는 `0x001`을 제어 명령으로 처리하지 말고 수신 시 무시하거나 진단용으로만 계수한다. `controller_set_estop()`으로 연결된 경로를 제거한다. 향후 실제 E-stop 기능을 넣는다면 별도 안전 설계와 별도 프로토콜로 정의한다. 기존 B 상태 머신의 다른 오류 상태까지 무조건 삭제하지 않는다.

검증: 임의 payload의 `0x001`을 반복 송신해도 B가 `STATE_ESTOP`으로 전환하거나 `0x001` 데이터로 제어 출력을 바꾸지 않는다. B의 제어 입력 `0x010` 처리와 출력 계산은 별도 정상 시험에서 확인한다.

## P0 — B→A 출력 경로의 관측 가능성 확보

대상: `esp32b_fw/main/can_protocol.c`, `app_main.c`; 필요하면 `can_protocol.h`.

- 현재 `can_send_actuator_frame()`, `can_send_actuator_b_frame()`, `can_send_status_frame()`은 내부 `can_send_frame()` 결과를 버린다. 성공/실패를 호출자 또는 별도 카운터에 전달해 ID별 시도, 송신 완료 성공, 실패, 완료 timeout을 누적한다. `on_tx_done`의 `is_tx_success`와 `twai_node_get_info()` 상태를 구분해 기록한다.
- 현재 `app_main.c`는 `0x180`, `0x181`, `0x1F0` 송신을 순차적으로 완료 대기한 뒤 50 ms 지연한다. 부하 중 송신 대기가 늘면 실제 출력 주기도 늘 수 있다. 시도 시각·완료 시각·sequence를 계측하고, 스위치를 놓은 뒤 오래된 출력 프레임이 뒤늦게 한꺼번에 전달되지 않도록 큐/재시도 정책을 검토한다. ID `0x180`/`0x181`은 같은 sequence의 한 쌍으로 유지한다.
- `watchdog_note_heartbeat(1)`은 현재 송신 성공 여부와 무관하게 호출된다. 이것을 B→A 링크가 정상이라는 증거로 사용하지 않는다. 진단용 heartbeat와 실제 송신 성공 통계를 분리한다.
- `last_error_flags`는 전역 최근 오류 값이므로 특정 `0x180` 실패의 확정 원인으로 기록하지 않는다. `on_error`의 종류별 카운터를 둔다. 로컬 ESP-IDF의 `0x01`은 `arb_lost`, `0x10`은 `ack_err`이다. `TEC`/`REC`/큐 잔량은 조회 순간의 상태다.

검증: 부하 OFF/ON/OFF 세 구간에서 B의 ID별 송신 성공률, 주기, timeout 및 버스 상태를 비교한다. 부하 ON 중 B 출력 계산 자체가 중단됐는지, 계산은 했으나 CAN 송신에 실패했는지 구분할 수 있어야 한다.

## P0 — A→노트북 전달의 관측 가능성 확보

대상: `esp32a_fw/main/uart_can_bridge.c`.

- A의 현재 `0x180`+`0x181` 동일 sequence 페어링 및 UART `0x80` 전달, `0x1F0`→UART `0x81` 전달을 유지한다. 페어 누락, sequence 불일치, 50 ms timeout, UART write 실패를 계수한다. B 송신 성공 수와 A 페어 완성 수, 노트북 수신 수를 대조할 수 있게 한다.
- A의 USB Serial/JTAG 포트는 바이너리 프로토콜 전용이다. 사람이 읽는 로그를 이 포트에 섞지 않는다. 진단은 별도 출력 경로나 호스트 측 프레임 계수로 수집한다.
- 부하 중 UART `0x80`이 오지 않거나 늦게 오면 노트북/ROS 측에서 **출력 데이터 freshness timeout**으로 감지해야 한다. 오래된 마지막 제어 출력을 새 출력으로 재사용하지 않도록 상위 시스템 요구사항에 명시한다. ROS 구현은 이 ESP32 명세의 코드 범위 밖이다.

## P1 — 입력 경로 영향 및 시험 해석

- 부하 `0x001`은 A의 제어 `0x010`과 오도메트리 `0x110`/`0x120`도 지연시킨다. B의 `RX control count`, 오도메트리 완성 횟수와 freshness를 함께 기록해 B→A 차단 효과와 입력 상실 효과를 구분한다.
- `watchdog.c`는 현재 제어 timeout을 로그만 남긴다. 실제 구동부를 연결할 미래 펌웨어에서는 입력 상실 시 안전 출력 및 재개 정책을 별도로 구현한다. 이 부하 시험만으로 안전 차단 기능이 구현됐다고 간주하지 않는다.
- CAN 부하가 사라져도 버스 회복에는 송신 대기·재시도 시간이 들 수 있다. OFF 복귀 지연을 별도 지표로 측정한다.

## 수용 시험

1. **부하 OFF:** 노트북 명령→A→B RX, B `0x180/0x181`→A 페어→UART `0x80`→노트북을 sequence로 확인한다.
2. **스위치 ON:** Teensy `0x001` 수신과 함께 B 시도/성공, A 페어/노트북 수신율, A→B 입력률을 동시에 측정한다. `0x001` 수신 자체가 B의 E-stop 상태를 만들지 않아야 한다.
3. **스위치 OFF 복귀:** Teensy의 신규 dump 송신 중단과 정상 왕복 프레임 복귀 시간을 측정한다. 이미 전송 중인 최대 한 프레임 및 드라이버 내부에 남은 프레임은 별도 계측한다.
4. **실패 구분:** CAN TX 실패, A 페어 timeout, UART 전송 실패, ROS 수신 timeout을 구분한다. 실제 구동부 없이 시험한다.

근거 코드: `esp32b_fw/main/{can_protocol.c,app_main.c,controller_state.c,watchdog.c}`, `esp32a_fw/main/uart_can_bridge.c`.

## 구현 상태

- ESP32 B: `0x001` 부하 프레임을 ISR에서 계수하고 제어 큐에 넣지 않는다. 기존 E-stop 판정은 제거했다. ID별 송신 시도·성공·실패·timeout, 실제 TX 완료, 오류 종류와 출력 주기를 UART0/USB 보조 콘솔에 주기적으로 기록한다. CAN RX 큐 처리량을 한 주기당 32개로 제한해 메인 루프가 정체되지 않도록 했다.
- ESP32 A: `0x180`/`0x181` 페어 완성·불일치·timeout, 바이너리 UART 전달 성공/실패, `0x001` 수신 수를 UART0 콘솔에 주기적으로 기록한다. 노트북용 USB Serial/JTAG 바이너리 프로토콜은 바꾸지 않았다.
- 아직 필요한 검증: 실제 세 보드의 OFF/ON/OFF HIL 측정, Teensy 스위치 구동 펌웨어 변경, 노트북/ROS의 출력 freshness timeout. B의 제어 입력 timeout을 안전 출력으로 연결하는 것은 향후 실제 구동부용 상태 정책에 포함해야 한다.
