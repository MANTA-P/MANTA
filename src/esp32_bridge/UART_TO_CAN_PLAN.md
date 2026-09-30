# ROS 토픽 → UART → CAN 재구성 계획

## 1. 변경 목표

기존 문자 1 byte 시험 코드와 복잡한 양방향 HIL 패킷 초안을 걷어내고 다음
경로를 단계별로 완성한다.

```text
ROS 2 상태 토픽
  → PC 송신 노드(고정소수점 직렬화)
  → USB Serial/JTAG 또는 UART
  → ESP32 A(프레임 파싱 및 CAN 분할)
  → CAN
  → ESP32 B 제어기
```

이번 변경의 PC 측 범위는 ROS 토픽을 구독해 UART로 보내는 단방향 송신까지다.
ESP32 펌웨어의 새 프레임 파서와 UART→CAN 변환은 다음 단계에서 구현한다.

상세 wire format의 기준 문서는
[`docs/UART_TOPIC_PROTOCOL.md`](docs/UART_TOPIC_PROTOCOL.md)다.

## 2. 이번 단계: PC ROS 송신 노드

### 구현 내용

- 키보드 문자 송신 코드 삭제
- UART 역수신, packet parser와 추진기 토픽 publish 로직 삭제
- ROS 콜백에서 필요한 필드만 선택
- float/double을 명세된 고정소수점 정수로 변환
- `AA 55 | TOPIC_ID | DATA_LENGTH | DATA` 프레임 생성
- UART에 전체 프레임 write
- NaN, Inf, 범위 초과 메시지는 프레임 단위로 폐기
- 토픽 이름, serial device와 baud rate는 ROS parameter로 유지

### 완료 조건

- 패키지가 warning 없이 빌드된다.
- 각 ROS 토픽 입력에 대해 명세된 ID와 고정 길이 프레임이 나온다.
- 음수, endian, 반올림과 범위 오류가 명세대로 처리된다.
- 노드는 UART를 읽거나 ROS 토픽을 publish하지 않는다.

## 3. 다음 단계: ESP32 UART 수신기

- USB Serial/JTAG byte를 ring buffer에 누적한다.
- `AA 55`를 찾아 4-byte 헤더를 읽는다.
- Topic ID별 고정 DATA 길이와 헤더 길이가 같은지 검사한다.
- 완전한 프레임만 big-endian 고정소수점 값으로 복원한다.
- Topic ID별 최신값과 로컬 수신시각을 저장한다.
- 잘못된 ID/길이 또는 중간에 끊긴 프레임에서 다시 동기화한다.
- 프레임마다 로그를 출력하지 않고 수신·오류 통계만 저속 출력한다.

완료 조건은 PC에서 보낸 모든 Topic ID를 ESP32가 연속 수신하고, 원래 물리값과
복원값의 차이가 각 신호의 0.5 LSB 이하인 것이다.

## 4. 그다음 단계: UART → CAN 매핑

UART DATA는 최대 32 byte지만 Classical CAN payload는 8 byte다. UART 프레임을
그대로 CAN에 넣지 않고, 제어기에 실제로 필요한 신호를 8-byte 이하의 CAN
메시지 여러 개로 나눈다.

- CAN 통신 매트릭스에서 ID, 송신주기, timeout과 우선순위를 확정한다.
- 위치, 속도, 자세 등 최신값 중심의 독립 CAN 프레임으로 나눈다.
- E-stop은 가장 낮은 수치의 CAN ID와 별도 프레임을 사용한다.
- 각 CAN 메시지는 가능한 한 송신 노드를 하나로 제한한다.
- ESP32 B는 필수 데이터 timeout 시 안전 출력을 적용한다.
- 규약 확정 후 DBC를 만든다.

## 5. 통합 시험 순서

1. pseudo-terminal로 PC 송신 byte를 캡처해 UART 프레임을 검증한다.
2. 실제 `/dev/ttyACM*`에서 ESP32 수신·재동기화를 검증한다.
3. ESP32 A에서 복원한 값과 ROS 원본을 비교한다.
4. CAN 분석기로 CAN ID, DLC, endian과 주기를 확인한다.
5. ESP32 B의 timeout 및 범위 오류 안전 동작을 확인한다.
6. 정상 부하와 소나 포화 부하에서 손실률 및 E-stop 지연을 측정한다.

## 6. 전송률 기준

현재 실측 주기(odometry/IMU/torpedo 약 100 Hz, pressure/depth 약 50 Hz)를 모두
그대로 전송하면 헤더 포함 약 10.4 kB/s이며 DVL과 USB/serial overhead가 더해진다.
일반 115200 8N1 UART의 이론상 payload 한계 11.52 kB/s에 너무 가깝다.

- ESP32-S3 USB Serial/JTAG(`/dev/ttyACM*`)를 기본 경로로 사용한다.
- 실제 GPIO UART를 쓸 경우 921600 baud를 기준으로 한다.
- 토픽 주기가 지나치게 높아지면 콜백마다 전송하지 않고 Topic ID별 최신값을
  정해진 송신주기로 내보내는 rate limiter를 추가한다.

## 7. 현재 남은 주의점

- 루트 `main/uart_can_bridge.c`는 아직 문자 1 byte 시험용 ESP-IDF 펌웨어이므로
  새 ROS UART 프레임과 호환되지 않는다.
- UART 프로토콜에는 CRC, sequence와 timestamp가 없다. USB가 아닌 잡음 환경을
  사용하거나 누락 측정이 필요해지면 프로토콜 확장이 필요하다.
- DVL 토픽은 기존 실측에서 publisher가 없었다. 센서가 활성화된 환경에서 실제
  필드와 주기를 다시 확인한다.
