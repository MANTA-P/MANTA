# ESP32-S3 → Teensy CAN 통신 구현 계획

## 1. 목표

ESP-IDF를 사용해 **ESP32-S3-N16R8** 보드에 펌웨어를 작성하고, ESP32-S3가 CAN 버스를 통해 **Teensy 보드로 주기적인 메시지를 송신**하도록 구현한다.

초기 범위는 다음과 같다.

- ESP32-S3: CAN 메시지 송신 노드
- Teensy 4.1: CAN1 메시지 수신 노드
- 통신 방식: Classical CAN 2.0
- ESP32 개발 환경: ESP-IDF
- 1차 검증: 고정 CAN ID와 테스트 데이터를 주기적으로 송신하고 Teensy에서 수신 여부 확인

> ESP32-S3의 주변장치 명칭은 TWAI(Two-Wire Automotive Interface)이며 Classical CAN과 호환된다. ESP32-S3의 TWAI 컨트롤러만으로 CAN 버스에 직접 연결할 수 없으므로 외부 CAN 트랜시버가 필요하다. CAN FD 프레임은 이 초기 구현 범위에서 제외한다.

## 2. 구현 전 확정할 항목

다음 값은 실제 보드와 시스템 요구사항에 맞춰 확정해야 한다.

| 항목 | 초기 제안 | 비고 |
|---|---:|---|
| ESP32-S3 정확한 보드 모델 | 확인 필요 | N16R8은 일반적으로 Flash 16 MB, PSRAM 8 MB 구성을 의미하며 핀 배치는 보드별로 다를 수 있음 |
| Teensy 모델 | Teensy 4.1 | CAN1 기본 핀 TX 22/RX 23, `FlexCAN_T4` 수신 예제 사용 |
| CAN 비트레이트 | 500 kbit/s | 송신·수신 노드에서 반드시 동일하게 설정 |
| ESP32 TWAI TX GPIO | 보드 핀맵 확인 후 결정 | 부트 스트랩, USB, Flash/PSRAM 사용 핀과 충돌하지 않아야 함 |
| ESP32 TWAI RX GPIO | 보드 핀맵 확인 후 결정 | 입력 가능 GPIO 사용 |
| CAN 트랜시버 | 3.3 V 로직 호환 제품 | 예: SN65HVD230 계열. 실제 모듈 전압과 핀 구성을 반드시 확인 |
| 송신 CAN ID | `0x123` | 11-bit Standard ID 기준 |
| 송신 주기 | 100 ms | 통합 테스트 후 조정 |
| 데이터 형식 | 8 byte 테스트 payload | 이후 실제 데이터 프로토콜로 교체 |

## 3. 하드웨어 구성

### 3.1 필요한 장비

- ESP32-S3-N16R8 개발 보드
- Teensy 개발 보드
- 각 노드에 적합한 CAN 트랜시버
- CAN_H/CAN_L 배선
- 버스 양 끝의 120 Ω 종단 저항 2개
- 공통 GND 배선
- USB 케이블 및 전원
- 선택 사항: USB-CAN 분석기 또는 오실로스코프/로직 분석기

### 3.2 기본 연결

```text
ESP32-S3              CAN Transceiver A
---------             -----------------
TWAI TX GPIO  ------> TXD
TWAI RX GPIO  <------ RXD
3.3 V/GND      ------ VCC/GND
                           |
                     CAN_H |================| CAN_H
                     CAN_L |================| CAN_L
                           |                 |
                           |       CAN Transceiver B
                           |       -----------------
                           |       Teensy CAN TX/RX

버스 양 끝: CAN_H와 CAN_L 사이에 각각 120 Ω 종단 저항
ESP32-S3, Teensy, 두 트랜시버의 GND는 서로 연결
```

### 3.3 하드웨어 주의 사항

- ESP32-S3 GPIO를 CAN_H/CAN_L에 직접 연결하지 않는다.
- 트랜시버의 MCU 측 로직 전압이 ESP32-S3 및 Teensy의 3.3 V I/O와 호환되는지 확인한다.
- CAN_H와 CAN_L가 뒤바뀌지 않도록 한다.
- 종단 전원이 꺼진 상태에서 CAN_H-CAN_L 사이 저항을 측정했을 때 정상적인 120 Ω 종단 두 개가 병렬 연결된 버스는 약 60 Ω이 측정된다.
- 트랜시버에 `STB`, `S`, `EN` 같은 대기/활성 제어 핀이 있으면 정상 동작 상태로 설정한다.
- 긴 배선에서는 CAN_H/CAN_L를 트위스트 페어로 구성한다.

## 4. ESP-IDF 프로젝트 구조 계획

```text
teensy_esp_bridge/
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── can_sender.c
    └── can_sender.h       # 필요 시 분리
```

역할은 다음과 같이 구성한다.

- 최상위 `CMakeLists.txt`: ESP-IDF 프로젝트 선언
- `sdkconfig.defaults`: 기본 로그 레벨 및 보드 공통 설정
- `main/can_sender.c`: TWAI 드라이버 설치, 시작, 프레임 송신, 오류 처리
- `README.md`: 빌드·플래시·모니터링 방법과 핀 연결표

## 5. ESP32 송신 펌웨어 설계

### 5.1 초기화 순서

1. TWAI TX/RX GPIO를 설정한다.
2. TWAI 일반 설정을 생성하고 Normal Mode를 선택한다.
3. 양쪽 노드와 동일한 비트레이트의 타이밍 설정을 적용한다.
4. 초기 송신 전용 테스트에서는 모든 프레임을 허용하는 필터 설정을 사용한다.
5. `twai_new_node_onchip()`으로 TWAI 노드를 생성한다.
6. `twai_node_enable()`로 TWAI 컨트롤러를 시작한다.
7. 주기적인 송신 루프 또는 FreeRTOS task를 시작한다.

### 5.2 테스트 프레임

초기 통신 확인용 프레임 예시는 다음과 같다.

| 필드 | 값 |
|---|---|
| Frame type | Standard Data Frame |
| Identifier | `0x123` |
| DLC | 8 |
| Byte 0~3 | 증가하는 송신 카운터 |
| Byte 4~7 | 고정 패턴 또는 상태 값 |
| Period | 100 ms |

카운터는 Teensy 측에서 프레임 누락과 순서를 확인할 수 있도록 little-endian 또는 big-endian 중 하나를 정해 명시적으로 직렬화한다.

### 5.3 송신 및 오류 처리

- `twai_node_transmit()`의 반환값을 항상 확인한다.
- 성공 시 CAN ID, 카운터, payload를 디버그 로그에 출력한다.
- 송신 타임아웃, ACK 부재, bus-off 상태를 구분해 기록한다.
- TWAI 상태 및 alert를 감시해 오류 카운터와 bus-off 여부를 확인한다.
- bus-off 발생 시 원인을 기록하고, 정책에 따라 컨트롤러 복구 절차를 수행한다.
- 로그가 통신 타이밍에 영향을 주지 않도록 최종 단계에서는 출력 빈도를 낮춘다.

> CAN은 다른 활성 노드의 ACK가 필요하다. Teensy가 정상 설정되어 있지 않거나 트랜시버/배선에 문제가 있으면 ESP32의 송신 오류 카운터가 증가하고 bus-off로 진입할 수 있다.

## 6. Teensy 수신 측 준비

Teensy 측은 정확한 모델에 맞는 CAN 컨트롤러와 라이브러리를 사용해 다음 조건을 맞춘다.

- ESP32와 동일한 CAN 비트레이트
- CAN 트랜시버를 통한 물리 버스 연결
- Standard ID `0x123` 수신 허용
- 수신한 ID, DLC, 8-byte payload, 카운터를 Serial Monitor에 출력
- 카운터 연속성을 검사해 누락 프레임 수 기록

초기에는 필터를 넓게 열어 물리 통신을 먼저 확인하고, 통신 성공 후 필요한 ID만 수신하도록 필터를 좁힌다.

## 7. 개발 단계

### 단계 1: 환경 및 보드 확인

- 설치된 ESP-IDF 버전 확인
- `idf.py --version`과 toolchain 동작 확인
- 정확한 ESP32-S3 보드 모델과 핀맵 확인
- Teensy 4.1 CAN1 핀(TX 22/RX 23)과 `FlexCAN_T4` 수신 환경 확인
- 트랜시버 모델 및 전압 사양 확인

완료 조건: 사용할 GPIO, 비트레이트, 트랜시버, Teensy CAN 인터페이스가 확정됨.

### 단계 2: ESP-IDF 프로젝트 생성

- 최소 ESP-IDF 프로젝트 구조 생성
- 타깃을 `esp32s3`로 설정
- GPIO와 CAN 설정값을 코드 상단 또는 Kconfig로 관리
- TWAI 송신 코드 구현

완료 조건: 프로젝트가 경고 없이 빌드되고 ESP32-S3에 플래시됨.

### 단계 3: 단일 프레임 송신 확인

- ESP32에서 `0x123` 테스트 프레임 송신
- Teensy에서 ID, DLC, payload 출력
- 양쪽 비트레이트와 배선 확인

완료 조건: Teensy가 예상한 프레임을 반복적으로 수신함.

### 단계 4: 안정성 시험

- 송신 카운터를 이용해 누락 및 순서 검사
- 최소 10분 이상 연속 송수신
- ESP32 TWAI 오류 상태와 Teensy 수신 오류 확인
- 전원 재인가 후 자동 복구 확인
- CAN_H/CAN_L 단선 또는 Teensy 재부팅 후 동작 확인

완료 조건: 정상 조건에서 누락 없이 동작하고, 장애 후 정의된 방식으로 복구됨.

### 단계 5: 실제 메시지 프로토콜 적용

- 실제 전달 데이터와 단위 정의
- CAN ID 할당표 작성
- byte order, signed/unsigned, scale, offset 명시
- sequence counter, 상태 bit, checksum 필요 여부 결정
- 송신 주기와 버스 부하 검토

완료 조건: ESP32와 Teensy가 동일한 메시지 명세를 사용하고 실제 데이터를 정확히 해석함.

## 8. 빌드 및 실행 명령 계획

ESP-IDF 환경이 활성화된 터미널에서 다음 흐름을 사용한다.

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

실제 시리얼 포트가 `/dev/ttyACM0` 등으로 잡힐 수 있으므로 플래시 전에 포트를 확인한다. Monitor 종료는 일반적으로 `Ctrl+]`를 사용한다.

## 9. 검증 체크리스트

- [ ] ESP32-S3와 Teensy의 CAN 비트레이트가 동일하다.
- [ ] 두 보드 모두 외부 CAN 트랜시버를 사용한다.
- [ ] 트랜시버 로직 레벨과 전원 사양이 보드에 적합하다.
- [ ] CAN_H, CAN_L, GND가 올바르게 연결되어 있다.
- [ ] 버스 양 끝에만 120 Ω 종단 저항이 있다.
- [ ] ESP32-S3 TWAI TX/RX GPIO가 다른 기능과 충돌하지 않는다.
- [ ] Teensy에서 CAN ID `0x123`, DLC 8의 프레임을 확인한다.
- [ ] payload 카운터가 순서대로 증가한다.
- [ ] ESP32 로그에 지속적인 timeout 또는 bus-off가 없다.
- [ ] 재부팅 및 일시적인 수신 노드 분리 후 동작을 확인했다.

## 10. 후속 결정 사항

코드 구현을 시작하기 전에 아래 정보를 확정하면 하드웨어에 맞는 설정을 바로 적용할 수 있다.

1. ESP32-S3-N16R8 보드의 제조사와 정확한 모델명
2. 사용할 CAN 트랜시버 또는 트랜시버 모듈명
3. 원하는 통신 속도
4. ESP32-S3에서 사용할 TWAI TX/RX 핀
5. Teensy로 보낼 실제 데이터와 원하는 송신 주기
