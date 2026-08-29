# ESP32 UART-CAN 게이트웨이 작업 인수인계

## 에이전트 작업 원칙

이 작업은 새 프로젝트를 만드는 것이 아니라 ESP32 담당자가 이미 진행한
게이트웨이 코드를 이어서 완성하는 작업이다.

1. 시작할 때 README, 현재 소스, 설정, 빌드 방법과 Git 변경사항부터 확인한다.
2. 기존 parser와 동작 확인 코드를 지우거나 처음부터 다시 만들지 않는다.
3. 현재 구조와 명명 방식을 유지하면서 다음 미완성 단계부터 구현한다.
4. 문서는 기준안이며 실제 최신 코드와 하드웨어가 더 구체적이면 그 근거를 먼저
   확인한다.
5. 사소한 함수명이나 폴더 구조 차이는 합리적으로 맞춰 진행한다.
6. 다음처럼 통합 결과를 바꾸는 중요한 차이는 임의로 정하지 말고, 발견한 코드
   근거와 가능한 선택지를 사용자에게 설명해 확인받는다.

   - USB CDC인지 GPIO UART인지와 실제 포트 설정
   - UART packet header, CRC 방식, message type 또는 byte order
   - CAN bitrate, TX/RX 핀, CAN ID와 payload
   - ESP32 게이트웨이와 제어 ESP32 사이의 데이터 분할 방식
   - timeout, 재전송, E-stop 또는 안전 동작
   - 기존 구현을 크게 삭제하거나 다시 설계해야 하는 충돌

질문하기 전에는 저장소와 설정에서 확인 가능한 정보를 먼저 찾는다. 핀, ESP-IDF
버전이나 라이브러리 API를 추측하지 않는다.

## 역할과 현재 진행 상태

이 ESP32는 PC와 제어 ESP32 사이의 양방향 게이트웨이다.

~~~text
PC ROS 2
  -> USB/UART byte stream
ESP32 게이트웨이
  -> Classical CAN
ESP32 제어기

ESP32 제어기
  -> CAN 추진기 출력
ESP32 게이트웨이
  -> USB/UART packet
PC ROS 2
~~~

현재 완료된 범위:

- PC에서 들어오는 byte stream 수신
- 패킷 경계를 찾는 기본 parser 구현

현재 완료되지 않은 범위:

- CRC 검증
- sequence와 packet 유효성 검사
- message type별 payload 해석
- UART/USB packet에서 CAN frame으로 변환
- CAN 송수신과 역방향 packet 생성
- timeout, 오류 통계와 bus-off 복구

따라서 기존 parser를 확인한 뒤 CRC 검증 단계부터 이어서 진행한다.

## 현재 기준 통신 설정

| 항목 | 기준 |
| --- | --- |
| PC 장치 | /dev/ttyACM0 |
| PC 연결 | ESP32-S3 USB CDC/Serial을 기본 가정 |
| 터미널 설정 | 현재 PC 코드 기본값 115200, 8N1, flow control 없음 |
| CAN | Classical CAN, 500 kbit/s |
| CAN ID | 표준 11-bit |
| CAN payload | 최대 8 byte |
| byte order | big-endian, DBC에서는 Motorola |
| 트랜시버 | SN65HVD230 |

/dev/ttyACM0가 USB CDC인지 실제 GPIO UART인지에 따라 처리량 의미가 달라진다.
현재 펌웨어 구현이 기준안과 다르면 임의 변경하지 말고 사용자에게 확인한다.

## PC-ESP32 packet 기준안

| Offset | 크기 | 필드 |
| ---: | ---: | --- |
| 0 | 2 | Magic: AA 55 |
| 2 | 1 | Version: 초기값 01 |
| 3 | 1 | Message type |
| 4 | 1 | Flags |
| 5 | 2 | Sequence, big-endian |
| 7 | 2 | Payload length, big-endian |
| 9 | 4 | Timestamp us, big-endian |
| 13 | N | Payload |
| 13+N | 2 | CRC16 |

- 고정 overhead는 CRC 포함 15 byte다.
- payload length 기준안의 최대값은 512 byte다.
- CRC 기준안은 CRC-16/CCITT-FALSE다.
- 다중 byte 정수와 IEEE 754 float32 bit pattern은 모두 big-endian이다.
- C/C++ 구조체 메모리를 그대로 전송하지 않는다.

현재 parser의 packet 구조가 위 표와 다르면 먼저 실제 코드를 파악한다. 이미 다른
규약으로 양쪽 코드가 진행됐다면 한쪽만 독단적으로 변경하지 않는다.

## 다음 구현 순서

### 1. Parser 검증 완성

- Magic, Version, payload length를 검사한다.
- 전체 packet이 모인 뒤 CRC-16/CCITT-FALSE를 검증한다.
- CRC, version, length 오류를 각각 집계하고 packet을 폐기한다.
- 손상된 packet 뒤에서 한 byte씩 전진해 다음 Magic으로 재동기화한다.
- 한 read에 여러 packet이 오거나 한 packet이 여러 read로 나뉘는 경우를
  모두 처리한다.
- message type별 sequence 누락·중복을 감지한다.

### 2. Payload 해석

- message type과 예상 payload 길이를 함께 검사한다.
- big-endian 정수와 float32를 명시적 함수로 복원한다.
- NaN, infinity와 물리 범위 밖 값을 거부한다.
- 정상 packet만 message type별 최신 데이터 저장소로 전달한다.
- 주기 센서 데이터는 backlog를 쌓지 않고 최신값으로 교체한다.

### 3. CAN 송신

- TWAI/CAN을 500 kbit/s, 표준 11-bit ID로 설정한다.
- message type을 확정된 CAN ID에 매핑한다.
- 8 byte를 넘는 payload는 규약에 따라 여러 CAN frame으로 분할한다.
- frame sequence 또는 fragment 번호로 누락을 검출한다.
- enqueue 성공, 실제 송신 성공, queue full과 실패를 구분해 기록한다.
- CAN error와 bus-off를 감지하고 복구 정책을 구현한다.

### 4. CAN 수신과 PC 반환

- 제어 ESP32에서 오는 추진기 출력을 ID로 필터링한다.
- 분할 frame이면 sequence, fragment와 timeout으로 재조립한다.
- 6개 추진기 값이 유효할 때 PC용 message type으로 직렬화한다.
- header, sequence, timestamp, length와 CRC16을 생성해 PC로 보낸다.
- binary 통신 채널에 일반 디버그 문자열을 섞지 않는다.

## 필수 통계

약 1초 주기로 요약 출력하거나 진단 message로 제공한다.

~~~text
rx_bytes, rx_packets
rx_crc_error, rx_length_error, rx_version_error
rx_sequence_gap, rx_resync_count
payload_decode_error, latest_value_overwrite
can_tx_enqueued, can_tx_success, can_tx_fail
can_rx_frames, can_rx_reassembly_error
can_error_count, bus_off_count, bus_recovery_count
uart_tx_packets, uart_tx_fail
~~~

라이브러리가 실제 전송 완료를 알려주지 않으면 enqueue와 실제 bus 전송을 같은
통계로 표현하지 않는다.

## 시험 순서

1. 기존 parser 회귀시험: 분할 read, 연속 packet, Magic이 payload에 포함된 경우
2. CRC 시험: 정상, payload 1-bit 손상, CRC 손상, 잘린 packet
3. 재동기화 시험: 쓰레기 byte 뒤 정상 packet 수신
4. payload 시험: 각 message type, endian, 잘못된 길이, NaN과 범위 초과
5. PC -> ESP32 -> CAN 단방향 실기기 시험
6. CAN -> ESP32 -> PC 추진기 packet 역방향 시험
7. 양방향 동시 통신과 1분 이상 연속 부하 시험
8. Teensy 덤프 포화 상태에서 지연, 누락, queue overflow와 bus-off 측정

## 완료 조건

- 기존 parser 기능을 유지하면서 CRC와 유효성 검사가 동작
- 손상·분할·연속 packet 뒤에도 stream 재동기화
- 모든 CAN 노드와 500 kbit/s 통신
- PC packet을 CAN frame으로 변환해 제어 ESP32가 수신
- 추진기 CAN 값을 PC packet으로 반환
- big-endian과 CRC test vector가 PC 구현과 일치
- 주기 데이터는 latest-only이며 무제한 backlog가 없음
- 오류·폐기·sequence·bus-off 통계 제공
- CAN 포화 상태의 실제 통합시험 결과 기록

## 시작 시 확인할 정보

1. ESP32 펌웨어 코드 경로와 현재 브랜치
2. ESP-IDF 또는 Arduino 등 빌드 환경과 버전
3. parser가 현재 인식하는 실제 packet 구조
4. USB CDC/GPIO UART 방식과 사용 API
5. CAN TX/RX 핀과 TWAI 설정
6. 현재 정해진 CAN ID 및 분할 규약
7. 제어 ESP32가 기대하는 payload

## 관련 문서

- 전체 프로젝트: src/esp32_bridge/docs/HIL_CAN_BRIDGE_PROJECT.md
- CAN 및 UART 규약: src/esp32_bridge/docs/CAN_PROTOCOL_GUIDE.md
- PC ROS 토픽: docs/ROS_TOPIC_INVENTORY.md
- Teensy 시험: docs/TEENSY_CAN_LOAD_ESTOP_HANDOFF.md
