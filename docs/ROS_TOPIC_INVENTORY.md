# MANTA ROS 2 주요 토픽 실측 기록

## 목적

MANTA 프로젝트 실행 중 생성되는 주요 ROS 2 토픽을 실측하여 PC-ESP32 HIL
통신 규약의 입력 데이터, 송신 주기 및 예상 대역폭을 결정한다.

## 측정 환경

| 항목 | 값 |
| --- | --- |
| 측정 날짜 | TBD |
| ROS 배포판 | Jazzy |
| 실행 구성 | TBD |
| 시뮬레이션 시간 사용 | TBD |
| 측정 시간 | 토픽별 약 5~10초 |

## 주요 토픽 측정 결과

프로젝트 실행 후 실제 ROS 그래프를 기준으로 작성한다.

| 분류 | 토픽 | 메시지 타입 | Publisher | 실측 Hz | QoS | HIL 방향 | 비고 |
| --- | --- | --- | --- | ---: | --- | --- | --- |
| BlueROV 위치/속도 | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| BlueROV IMU | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| 압력 | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| 깊이 | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| DVL | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| 어뢰 상태 | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| 임무 목표 | TBD | TBD | TBD | TBD | TBD | PC→ESP32 | |
| 추진기 출력 1~6 | TBD | TBD | TBD | TBD | TBD | ESP32→PC | |

## 전체 토픽 목록

측정 시점의 `ros2 topic list -t` 결과를 기록한다.

```text
TBD
```

## 측정 방법

- 토픽 및 타입: `ros2 topic list -t`
- 연결 정보와 QoS: `ros2 topic info --verbose <topic>`
- 실제 발행 주기: `ros2 topic hz <topic>`
- 메시지 구조: `ros2 interface show <type>`
- 대표 메시지 값: `ros2 topic echo --once <topic>`

## HIL 패킷 설계 반영 항목

실측 후 각 주요 토픽에 대해 다음을 결정한다.

- 보드로 보낼 필드와 제외할 필드
- 원본 ROS 자료형과 wire 자료형
- big-endian payload 배치
- 목표 송신 주기와 timeout
- 토픽별 USB 데이터율
- CAN ID, CAN 프레임 수와 CAN 점유율
- 최신값 덮어쓰기 또는 신뢰성 있는 전달 여부
