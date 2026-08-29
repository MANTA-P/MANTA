# Teensy CAN 부하·E-stop 작업 인수인계

## 에이전트 작업 원칙

이 작업은 신규 구현이 아니라 기존 Teensy 담당자가 진행한 코드를 이어서
완성하는 작업이다.

1. 작업 시작 즉시 Teensy 저장소의 README, 현재 코드, 설정과 Git 변경사항을
   먼저 확인한다.
2. 기존 스위치 인식 및 CAN 시험 코드를 지우거나 처음부터 다시 만들지 않는다.
3. 이미 구현된 구조와 명명 방식을 최대한 유지하고 필요한 기능만 추가한다.
4. 이 문서는 인수인계 기준이지만 실제 하드웨어와 최신 코드가 더 구체적이면
   실제 구현을 근거로 판단한다.
5. 코드와 이 문서가 다를 때 사소한 이름·구조 차이는 합리적으로 맞춰 진행한다.
6. 다음처럼 결과를 바꾸는 중요한 불일치는 임의로 결정하지 말고 사용자에게
   현재 코드의 근거와 선택지를 설명한 뒤 확인한다.

   - CAN bitrate, CAN 컨트롤러 또는 배선 구성
   - E-stop/dump CAN ID와 우선순위
   - E-stop payload와 수신 ESP32 규약
   - 스위치 active level, 안전 동작 또는 재전송 정책
   - 기존 작업을 크게 삭제·재작성해야 하는 구조적 충돌

핀 번호나 라이브러리 API를 추측하지 않는다. 질문하기 전에는 저장소와 설정에서
확인 가능한 정보를 먼저 찾는다.

## 목표와 현재 상태

Teensy 4.1은 공통 CAN 버스에서 다음 두 역할을 담당한다.

- CAN1: 가상 소나 덤프를 발생시켜 버스를 거의 포화
- CAN2: 물리 스위치를 감지해 E-stop 프레임 송신

현재 담당자가 완료했다고 전달한 내용:

- 물리 스위치의 소프트웨어 인식
- Teensy 기본 CAN 동작 시험

따라서 남은 핵심은 덤프 생성, 우선순위 전환, latest-only 폐기 정책, timestamp,
통계 및 지연 비교 시험이다.

~~~text
Teensy CAN1 - SN65HVD230 -+  소나 덤프
Teensy CAN2 - SN65HVD230 -+-- CANH / CANL / GND 공통 버스
ESP32 게이트웨이 ---------+
ESP32 제어기 --------------+
~~~

덤프와 E-stop은 서로 다른 CAN 컨트롤러를 사용해 실제 CAN ID 중재 결과를
관찰한다. 각 컨트롤러에는 별도 SN65HVD230이 필요하다.

## 확정 기준

| 항목 | 값 |
| --- | --- |
| 보드 | Teensy 4.1 |
| CAN | Classical CAN, 500 kbit/s |
| ID | 표준 11-bit, 숫자가 작을수록 높은 우선순위 |
| payload | 최대 8 byte |
| byte order | big-endian, DBC에서는 Motorola |
| 배선 | CANH, CANL, GND 공통 |
| 종단저항 | 버스 물리 양 끝에만 120 Ω |

## 우선순위 비교 모드

| 모드 | E-stop ID | Dump ID | 목적 |
| --- | ---: | ---: | --- |
| 정상 | 0x001 | 0x700 | 포화 상태에서 고우선순위 E-stop 지연 측정 |
| 역전 | 0x700 | 0x001 | 저우선순위 E-stop 지연 또는 starvation 관찰 |

E-stop은 이미 전송 중인 프레임을 중단시키지 못한다. 정상 모드에서는 현재
프레임이 끝난 뒤 다음 중재에서 이기는 것이 예상된다. 역전 모드는 덤프가 항상
대기할 때 E-stop이 계속 중재에서 패배하는 의도적인 비정상 시험이다.

통합 전 다른 노드가 시험 ID를 사용하지 않는지 확인한다.

## 구현 요구사항

### CAN1 덤프

- DLC 8 프레임을 연속 생성한다.
- 500 kbit/s에서 표준 ID, DLC 8 프레임의 예상 점유시간은 bit stuffing을
  포함해 약 222~270 us다.
- 초기 포화 시험은 생성 주기 200 us 이하로 시작한다.
- 긴 FIFO를 만들지 않고 최신 프레임 하나만 유지한다.
- mailbox가 비면 최신 프레임을 넣고, 적재하지 못한 샘플은 폐기한다.
- 프레임별 Serial 출력은 금지하고 통계만 약 1초 주기로 출력한다.

권장 payload:

| Byte | 크기 | 값 |
| ---: | ---: | --- |
| 0 | 4 | 생성 sequence, uint32 big-endian |
| 4 | 4 | micros() 생성 시각, uint32 big-endian |

### CAN2 E-stop

- raw edge, 디바운싱 확정, 송신 요청 시각을 각각 기록한다.
- active 확정 즉시 송신한다.
- 초기 권장값은 3회, 1 ms 간격 반복이며 기존 구현과 다르면 확인 후 결정한다.
- E-stop 실패는 덤프처럼 폐기하지 않고 실패·재시도 횟수를 기록한다.
- active와 release를 구분하고 active 안전 동작을 우선한다.

권장 payload:

| Byte | 크기 | 값 |
| ---: | ---: | --- |
| 0 | 1 | 0x01=active, 0x00=released |
| 1 | 1 | 0x00=정상, 0x01=우선순위 역전 |
| 2 | 2 | E-stop sequence, uint16 big-endian |
| 4 | 4 | 디바운싱 확정 micros(), uint32 big-endian |

수신 ESP32가 이미 다른 payload를 구현했다면 임의로 바꾸지 말고 사용자와
규약을 먼저 확정한다.

### 구조

- 부팅 시 모드와 실제 E-stop/dump ID를 한 번 출력한다.
- 덤프는 latest-only, E-stop은 별도 상태/큐로 처리한다.
- ISR에서는 입력 상태와 timestamp만 저장한다.
- CAN 및 Serial 처리는 메인 실행 문맥에서 수행한다.
- 공유 변수는 원자성 또는 임계구역으로 보호한다.
- micros() wrap은 uint32 unsigned subtraction으로 처리한다.

## 필수 통계

~~~text
mode, uptime_us
dump_generated, dump_enqueued, dump_tx_success
dump_dropped, dump_mailbox_busy
estop_raw_edges, estop_debounced_events
estop_tx_requests, estop_tx_success, estop_tx_fail_or_retry
can_error_count, bus_off_count
~~~

송신 완료 callback이 없다면 enqueue 성공을 실제 버스 전송 완료로 표기하지
않는다.

## 지연 측정

~~~text
t_raw       스위치 최초 감지
t_debounced E-stop 활성 확정
t_request   CAN 송신 요청
t_tx        실제 송신 완료 또는 가장 가까운 관측 시각
t_rx        ESP32/측정 노드 수신
t_safe      ESP32 안전 출력 적용
~~~

- 스위치 처리: t_debounced - t_raw
- Teensy 송신: t_tx - t_request
- CAN 전달: t_rx - t_request
- 전체 안전 적용: t_safe - t_raw

보드별 micros()는 동기화되지 않았으므로 서로 다른 보드 timestamp를 직접 빼지
않는다. 가능하면 측정 GPIO와 CANH/CANL을 로직 분석기나 오실로스코프로 함께
관찰한다.

## 시험 순서

1. 덤프 없이 E-stop 20회: 디바운싱, sequence, payload 검증
2. 덤프 단독: 500, 300, 250, 200, 150 us를 각 30초 시험
3. 정상 모드 포화: E-stop 30회 이상 측정
4. 역전 모드 포화: 같은 조건에서 E-stop 30회 이상 측정

두 우선순위 시험은 생성 주기, payload, 배선, bitrate와 측정 시간을 동일하게
유지한다. 결과에는 최소·평균·최대·p95 지연, 누락 수, timeout을 넘긴
starvation 수, CAN error와 bus-off 수를 포함한다.

## 완료 조건

- CAN1 덤프와 CAN2 E-stop이 같은 버스에서 독립 동작
- 모든 노드가 500 kbit/s로 통신
- 덤프 latest-only 동작과 폐기 카운터 확인
- 정상/역전 모드를 쉽게 전환
- 두 모드에서 동일 조건으로 E-stop 지연 30회 이상 측정
- 정상 모드 최대 지연과 역전 모드 지연·starvation 비교
- CAN error와 bus-off 포함 결과 작성
- 실제 위험한 구동부와 분리해 역전 시험 수행

## 시작 시 확인할 정보

1. Teensy 코드 경로와 Arduino IDE/PlatformIO 등 빌드 환경
2. CAN 라이브러리와 CAN1/CAN2 객체
3. CAN TX/RX 및 스위치 핀
4. 스위치 active level과 debounce 방식
5. 송신 완료 callback 및 CAN 오류 통계 지원 여부
6. ESP32가 현재 사용하는 E-stop ID와 payload

## 관련 문서

- 전체 구성: src/esp32_bridge/docs/HIL_CAN_BRIDGE_PROJECT.md
- CAN 규약: src/esp32_bridge/docs/CAN_PROTOCOL_GUIDE.md
- ROS 토픽: docs/ROS_TOPIC_INVENTORY.md
