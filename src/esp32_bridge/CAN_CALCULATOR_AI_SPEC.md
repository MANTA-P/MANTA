# CAN 계산기 AI 구현 명세

이 문서는 후속 AI 또는 개발자가 현재 저장소에 CAN 계산기를 구현할 때 사용할 작업 지시서다. 와이어 프로토콜의 최종 기준은 [`CAN_CALCULATOR_PROTOCOL.md`](./CAN_CALCULATOR_PROTOCOL.md)이며, 두 문서가 충돌하면 프로토콜 문서를 따른다.

## 1. 구현 목표

현재 ESP-IDF 6.1 기반 ESP32-S3 기본 예제를 두 노드에서 사용할 수 있는 CAN 계산기 펌웨어로 변경한다.

- 요청 노드: ADD/SUB 요청 송신, 결과 수신 및 검증, 출력
- 계산 노드: 요청 수신 및 검증, 계산, 결과 송신
- 동일 저장소에서 빌드 설정으로 노드 역할과 TWAI GPIO를 선택 가능하게 한다.
- Classical CAN, 500 kbit/s, 11-bit Standard ID를 사용한다.

## 2. 현재 저장소 기준

- 타깃: `ESP32-S3`
- ESP-IDF: `6.1.0`
- 현재 진입점: `main/esp32s3_demo.c`
- 현재 동작: 1초마다 Hello/count 로그만 출력
- `sdkconfig`상 TWAI 컨트롤러 1개 지원
- CAN/TWAI 코드와 테스트는 아직 없음

구현 전에 설치된 ESP-IDF 6.1의 로컬 헤더와 공식 예제를 확인하고, 해당 버전에 맞는 TWAI API를 사용한다. 구형 API와 신형 API를 한 구현에서 혼용해서는 안 된다.

## 3. 규범 용어

- **MUST**: 반드시 지켜야 한다.
- **MUST NOT**: 절대 해서는 안 된다.
- **SHOULD**: 특별한 이유가 없다면 지켜야 한다.
- **MAY**: 선택 사항이다.

## 4. 프로토콜 상수

```text
CAN_BITRATE             = 500000
CAN_ID_RESULT           = 0x001
CAN_ID_ADD_REQUEST      = 0x002
CAN_ID_SUB_REQUEST      = 0x003
CAN_DLC                 = 8
PROTOCOL_VERSION        = 0x01

OPERATION_ADD           = 0x02
OPERATION_SUB           = 0x03

STATUS_OK               = 0x00
STATUS_INVALID_FORMAT   = 0x01
STATUS_UNSUPPORTED_VER  = 0x02
STATUS_INTERNAL_ERROR   = 0x03

RESPONSE_TIMEOUT_MS     = 100
MAX_RETRIES             = 1
```

CAN ID는 표준 11-bit ID다. `0x001` 같은 표기와 정수 상수 이름을 일관되게 사용하고, 문맥 없는 `1`, `2`, `3` 매직 넘버를 코드에 두지 않는다.

## 5. 정확한 wire format

### ADD/SUB 요청: ID `0x002` / `0x003`, DLC 8

```text
Byte 0      version       uint8, must be 0x01
Byte 1      sequence      uint8
Byte 2..3   operand_a     int16, little endian
Byte 4..5   operand_b     int16, little endian
Byte 6..7   reserved      must be 0x00 0x00
```

### 결과: ID `0x001`, DLC 8

```text
Byte 0      version       uint8, 0x01
Byte 1      sequence      uint8, copied from request
Byte 2      operation     uint8, ADD=0x02 or SUB=0x03
Byte 3      status        uint8
Byte 4..7   result        int32, little endian; zero on error
```

구현은 다음 규칙을 MUST 준수한다.

- 멀티바이트 값은 명시적인 encode/decode 함수로 바이트별 변환한다.
- C 구조체를 CAN payload에 `memcpy`하거나 구조체 포인터로 캐스팅하지 않는다.
- signed 값은 2의 보수 비트 패턴으로 encode/decode한다.
- 수신 DLC를 확인하기 전에는 payload 바이트를 읽지 않는다.
- 뺄셈은 반드시 `operand_a - operand_b` 순서다.
- 계산 전에 두 입력을 `int32_t`로 승격한다.

## 6. 권장 파일 구조

```text
main/
├── esp32s3_demo.c          # 초기화, 선택된 역할 실행
├── can_protocol.h          # ID, enum, DTO, 공개 codec API
├── can_protocol.c          # encode/decode 및 유효성 검사
├── can_transport.h         # TWAI 초기화/송신/수신 API
├── can_transport.c         # ESP-IDF TWAI 어댑터
├── requester_node.h
├── requester_node.c        # 요청 상태 머신
├── calculator_node.h
├── calculator_node.c       # 계산 서버
└── Kconfig.projbuild       # 역할 및 GPIO 설정
```

`main/CMakeLists.txt`에는 추가된 `.c` 파일을 모두 등록한다. 역할별 코드와 프로토콜 codec을 분리해 codec이 TWAI 드라이버 없이도 단위 시험 가능하도록 한다.

## 7. 빌드 설정 요구사항

Kconfig에서 다음을 선택 가능하게 한다.

- 노드 역할: `REQUESTER` 또는 `CALCULATOR` 중 정확히 하나
- `CAN_TX_GPIO`
- `CAN_RX_GPIO`

GPIO 기본값이 필요하면 개발 편의를 위해 TX=`4`, RX=`5`를 사용할 수 있지만, 시작 로그에 실제 설정값을 출력하고 실제 배선과 일치하는지 사용자에게 명확히 알린다. GPIO 값은 프로토콜 값이 아니며 보드별로 달라도 된다.

## 8. 공통 transport 요구사항

- TWAI를 500 kbit/s normal mode로 시작한다.
- Standard Data Frame만 송신한다.
- 수신 후 extended/RTR 여부, ID, DLC를 검사한다.
- 하드웨어 필터만으로 완전한 ID 필터링이 어렵다면 software에서 `0x001`~`0x003`을 정확히 검사한다.
- 각 TWAI API 반환값을 검사하고 실패를 성공으로 기록하지 않는다.
- 오류 상태와 bus-off를 로그로 남기고 ESP-IDF 권장 절차에 따라 복구를 시도한다.
- 무한 대기는 피하고 FreeRTOS task가 watchdog을 방해하지 않게 block timeout을 사용한다.

## 9. 요청 노드 상태 머신

```text
IDLE → SEND_REQUEST → WAIT_RESULT
                     ├─ valid result → IDLE
                     ├─ 100 ms timeout and retry=0 → SEND_SAME_REQUEST
                     └─ second timeout → LOG_FAILURE → IDLE
```

요청 노드는 다음을 MUST 수행한다.

1. 한 번에 미완료 요청 하나만 유지한다.
2. 새 요청의 sequence를 증가시키고 `255` 다음에 `0`을 사용한다.
3. 재전송할 때 payload와 sequence를 바꾸지 않는다.
4. 결과의 version, sequence, operation, status를 모두 검사한다.
5. 현재 요청과 일치하지 않는 응답과 완료 후 도착한 중복 응답을 무시하고 진단 로그를 남긴다.
6. 정상 응답이면 식과 결과를 사람이 읽을 수 있게 출력한다.

요청 입력 UI가 별도로 정해지지 않았다면 최초 통합 시험에서는 문서의 고정 테스트 벡터를 순서대로 보내는 데모 task를 구현한다. 콘솔 명령, 버튼, 디스플레이 등 문서에 없는 UI를 임의로 추가하지 않는다.

## 10. 계산 노드 처리 순서

1. 프레임 수신
2. Standard Data Frame 여부 검사
3. CAN ID가 `0x002` 또는 `0x003`인지 검사
4. DLC가 8인지 검사
5. version 검사
6. sequence 읽기
7. reserved 두 바이트 검사
8. 피연산자 decode
9. `int32_t`로 ADD 또는 `A - B` 계산
10. 원 요청과 같은 sequence와 해당 operation을 넣어 `0x001` 결과 송신

오류 처리:

- DLC 오류, Extended/RTR, 알 수 없는 ID: 폐기하며 응답하지 않는다.
- 잘못된 version: `UNSUPPORTED_VERSION`, result=`0`으로 응답한다.
- reserved가 0이 아님: `INVALID_FORMAT`, result=`0`으로 응답한다.
- 내부 처리 실패: 가능한 경우 `INTERNAL_ERROR`, result=`0`으로 응답한다.
- 유효한 요청 하나마다 결과 한 개를 송신한다. 동일 요청 재수신도 다시 응답한다.

## 11. 로그 요구사항

로그에는 최소한 다음 정보가 있어야 한다.

- 시작 시 역할, bitrate, TX GPIO, RX GPIO
- 송수신 시 CAN ID, sequence, operation
- 정상 결과: `A`, `B`, 연산자, 결과
- 폐기한 프레임의 이유
- timeout과 재전송 횟수
- TWAI 오류 상태와 bus-off 복구 결과

로그에 성공하지 않은 송신을 `sent` 또는 성공으로 표시하면 안 된다.

## 12. 필수 테스트 벡터

| 목적 | 요청 ID | 요청 DATA | 기대 결과 DATA |
|---|---:|---|---|
| 양수 덧셈 `1+2=3` | `0x002` | `01 01 01 00 02 00 00 00` | `01 01 02 00 03 00 00 00` |
| 음수 포함 덧셈 `1000+(-250)=750` | `0x002` | `01 2A E8 03 06 FF 00 00` | `01 2A 02 00 EE 02 00 00` |
| 음수 결과 `10-25=-15` | `0x003` | `01 2B 0A 00 19 00 00 00` | `01 2B 03 00 F1 FF FF FF` |
| 최대 덧셈 `32767+32767=65534` | `0x002` | `01 FE FF 7F FF 7F 00 00` | `01 FE 02 00 FE FF 00 00` |
| 최대 차 `32767-(-32768)=65535` | `0x003` | `01 FF FF 7F 00 80 00 00` | `01 FF 03 00 FF FF 00 00` |
| sequence 순환 | 다음 ADD | `01 00 01 00 01 00 00 00` | `01 00 02 00 02 00 00 00` |

추가 오류 시험:

- DLC 7 요청은 응답하지 않는다.
- version `0x02` 요청은 status `0x02`, result `0`을 응답한다.
- reserved가 0이 아닌 요청은 status `0x01`, result `0`을 응답한다.
- Extended ID, RTR, 미정의 ID는 무시한다.

## 13. 검증 절차

1. protocol codec 단위 테스트를 host 또는 ESP-IDF test 환경에서 실행한다.
2. 각 역할이 warning 없이 빌드되는지 확인한다.
3. 계산 노드 단독으로 CAN 분석기에서 요청을 주입해 결과 바이트를 확인한다.
4. 요청 노드 단독으로 결과를 주입해 matching과 timeout을 확인한다.
5. 두 보드, 트랜시버, 120 Ω 종단 두 개를 연결한다.
6. 필수 테스트 벡터를 순서대로 실행한다.
7. 케이블 분리 등으로 timeout, bus-off, 복구 동작을 확인한다.

## 14. 구현 금지 사항

- 프로토콜 문서 없이 CAN ID, payload 필드, 상태 코드를 추가하거나 변경하지 않는다.
- C bit-field 또는 compiler packing에 wire format을 의존하지 않는다.
- GPIO를 여러 소스 파일에 중복 하드코딩하지 않는다.
- 수신 프레임 검증 전에 계산하지 않는다.
- 요청 노드와 계산 노드가 모두 `0x001`을 송신하게 만들지 않는다.
- CAN 트랜시버 없이 ESP32-S3 핀을 CAN_H/CAN_L에 직접 연결하지 않는다.
- 테스트를 통과시키기 위해 수신 검사를 생략하지 않는다.

## 15. 완료 조건(Definition of Done)

- [ ] `CAN_CALCULATOR_PROTOCOL.md`와 실제 wire format이 일치한다.
- [ ] 요청/계산 역할을 빌드 설정으로 각각 선택할 수 있다.
- [ ] 모든 프로토콜 상수가 공통 헤더 한 곳에 정의되어 있다.
- [ ] codec encode/decode 단위 테스트가 필수 벡터를 모두 통과한다.
- [ ] 두 역할 모두 ESP32-S3 대상으로 clean build된다.
- [ ] 두 실제 노드에서 ADD와 SUB 정상 결과가 확인된다.
- [ ] 경계값, 음수, sequence 순환이 확인된다.
- [ ] 잘못된 프레임과 timeout 동작이 명세와 일치한다.
- [ ] bus-off가 로그에 남고 복구 가능하다.
- [ ] 핀, 트랜시버, 실제 배선 정보가 사람용 문서의 표에 최종 반영된다.

## 16. 범위 밖 항목

v1에서는 다음을 구현하지 않는다.

- 곱셈, 나눗셈 또는 부동소수점 계산
- CAN FD 및 29-bit Extended ID
- 인증, 암호화, 애플리케이션 CRC
- 여러 요청 노드 또는 여러 계산 노드의 주소 지정
- 디스플레이, 모바일 앱, 클라우드 연동
- DBC 자동 생성

