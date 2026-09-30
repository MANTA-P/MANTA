# ESP32 A/B 펌웨어 플래시 및 검증 가이드

이 문서는 다음 두 프로젝트를 실제 보드에 올리고, UART/CAN 통신이 정상인지 검증하는 절차를 정리한 문서다.

- ESP32 A: [src/temp/esp32a_fw](src/temp/esp32a_fw)
- ESP32 B: [src/temp/esp32b_fw](src/temp/esp32b_fw)

현재 워크스페이스 기준으로, 둘 다 ESP-IDF 6.0 환경에서 빌드 검증이 완료된 상태다.

---

## 1. 준비물

필수:
- ESP32 보드 2개
- USB 데이터 케이블 2개
- PC with ESP-IDF 6.0 installed
- USB-UART TTL adapter 1개 (UART 검증용)
- CAN transceiver (SN65HVD230 계열 등) 또는 CAN 인터페이스 보드
- 공통 GND 케이블
- 점퍼선

권장:
- oscilloscope or logic analyzer
- CAN USB dongle (candump 가능)
- 브레드보드/테스트 점퍼

---

## 2. 프로젝트 경로

다음 두 경로가 최종 대상 프로젝트다.

```bash
/home/user/manta_ws/src/temp/esp32a_fw
/home/user/manta_ws/src/temp/esp32b_fw
```

각 프로젝트는 다음 방식으로 빌드한다.

```bash
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
cd /home/user/manta_ws/src/temp/esp32a_fw
idf.py build

cd /home/user/manta_ws/src/temp/esp32b_fw
idf.py build
```

빌드가 성공하면 각각 다음 파일이 생성된다.

```bash
/home/user/manta_ws/src/temp/esp32a_fw/build/esp32a_fw.bin
/home/user/manta_ws/src/temp/esp32b_fw/build/esp32b_fw.bin
```

---

## 3. 보드별 역할

### ESP32 A

- UART-to-CAN bridge 역할
- PC 또는 USB-UART 장치의 UART 패킷을 CAN 트래픽으로 변환
- CAN 수신을 다시 UART 패킷으로 변환
- 보드 시작 로그: `ESP32 A gateway starting`

### ESP32 B

- CAN 수신 기반 제어기
- CAN control/status 메시지 처리
- actuator status periodic transmit
- 보드 시작 로그: `ESP32 B starting`

---

## 4. 보드 연결 구성

### 4.1 CAN 연결

ESP32 A와 ESP32 B는 모두 다음 핀을 사용한다.

```text
ESP32 A/B CAN TX = GPIO17
ESP32 A/B CAN RX = GPIO18
```

권장 연결:

```text
ESP32 A CAN_H  <----> ESP32 B CAN_H
ESP32 A CAN_L  <----> ESP32 B CAN_L
ESP32 A GND    <----> ESP32 B GND
```

추가 권장:
- CAN bus 양 끝이면 120Ω termination resistor를 붙인다.
- CAN transceiver가 각 보드에 연결되어 있어야 한다.
- 전원 GND를 공통으로 맞춘다.

### 4.2 UART 연결 (검증용)

A 보드를 PC와 연결해서 UART 검증을 하려면, USB-UART TTL adapter를 사용한다.

```text
USB-UART TX  -> ESP32 A RX = GPIO44
USB-UART RX  <- ESP32 A TX = GPIO43
USB-UART GND -> ESP32 A GND
```

주의:
- A 보드의 GPIO43/44는 UART1로 사용된다.
- PC에서 시리얼 터미널을 열면 UART 패킷을 보내고 받을 수 있다.

---

## 5. 보드 플래시 절차

### 5.1 포트 확인

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

예시:

```bash
/dev/ttyUSB0
/dev/ttyUSB1
```

보드가 2개라면 보통 각각 다른 포트가 잡히며, USB 포트마다 결정된다.

### 5.2 A 보드 플래시

```bash
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
cd /home/user/manta_ws/src/temp/esp32a_fw
idf.py -p /dev/ttyUSB0 flash
```

A 보드가 다른 포트에 연결되어 있으면 해당 포트로 바꾼다.

### 5.3 B 보드 플래시

```bash
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
cd /home/user/manta_ws/src/temp/esp32b_fw
idf.py -p /dev/ttyUSB1 flash
```

보드 포트가 다르면 맞춰서 변경한다.

### 5.4 모니터 확인

플래시 후 모니터를 열어 boot 로그를 확인한다.

```bash
cd /home/user/manta_ws/src/temp/esp32a_fw
idf.py -p /dev/ttyUSB0 monitor
```

```bash
cd /home/user/manta_ws/src/temp/esp32b_fw
idf.py -p /dev/ttyUSB1 monitor
```

예상 로그:

```text
ESP32 A gateway starting
ESP32 B starting
```

---

## 6. UART 통신 검증

A firmware는 UART input을 받아 CAN으로 전달하는 구조다. UART 검증은 A 보드에 USB-UART 변환기를 연결해서 수행한다.

### 6.1 UART 검증 전 준비

1. A 보드를 USB로 PC에 연결
2. USB-UART adapter를 A 보드에 연결
3. `idf.py monitor`로 A 보드 로그를 확인
4. 시리얼 터미널을 별도 창에서 열기

### 6.2 테스트 패킷 구조

A firmware가 기대하는 패킷 형식은 다음과 같은 단순 구조를 기준으로 한다.

```text
[magic][dlc][id_hi][id_lo][payload...][checksum]
```

- magic: 0xA5
- dlc: payload 길이 (최대 8)
- id: CAN ID (2 bytes big-endian)
- payload: dlc 바이트
- checksum: payload/헤더의 simple checksum

### 6.3 UART 전송 예시

예를 들어 CAN ID 0x010, payload `01 02 03 04`를 보낸다고 가정하면, PC에서 아래처럼 전송한다.

```python
import serial, time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def checksum(data: bytes) -> int:
    return sum(data) & 0xFF

payload = bytes([0x01, 0x02, 0x03, 0x04])
can_id = 0x010
packet = bytes([0xA5, len(payload), (can_id >> 8) & 0xFF, can_id & 0xFF]) + payload
packet += bytes([checksum(packet[1:])])
ser.write(packet)
ser.flush()
print('sent:', packet.hex())
```

### 6.4 UART 검증 기대 결과

- A 보드가 UART 패킷을 받아 CAN으로 전송해야 함
- A 보드 로그에서 다음과 같은 흐름을 관찰할 수 있어야 함

```text
ESP32 A gateway starting
```

- 보드의 CAN RX 쪽이 반응하면 연결이 정상
- PC에서 `idf.py monitor`로 A 쪽 로그를 보면서 동작 확인

### 6.5 UART 검증 체크리스트

- 보드 전원이 정상인지
- GND 연결이 되었는지
- TX/RX가 반대로 연결되지 않았는지
- baud rate가 115200인지
- 패킷 길이와 체크섬이 맞는지

---

## 7. CAN 통신 검증

CAN 검증은 A 보드와 B 보드를 같은 버스에 연결하고, A 또는 B 쪽에서 프레임이 전송되는지 확인하는 방식이다.

### 7.1 CAN 검증 준비

1. A와 B 보드를 같은 CAN 버스에 연결
2. 각각 CAN H/L, GND 연결
3. 두 보드 모두 전원 ON
4. 두 보드의 모니터를 각각 실행

### 7.2 기본 CAN 확인

A 보드와 B 보드 모두 초기화 후 CAN node가 활성화되어야 한다.

B 보드 로그에서 다음을 기대한다.

```text
CAN initialized: TX=17 RX=18 bitrate=500000
ESP32 B starting
```

A 보드 로그에서 다음을 기대한다.

```text
UART-CAN bridge initialized: UART1 ... TWAI ...
ESP32 A gateway starting
```

### 7.3 CAN 프레임 전송 테스트

A 보드가 UART 패킷을 받아 CAN으로 전송하는 동작을 확인하려면,
다음 흐름을 검증한다.

```text
PC UART -> ESP32 A -> CAN bus -> ESP32 B
```

반대로 B 보드가 상태/액추에이터 프레임을 보내면,
A 보드가 그것을 UART로 다시 내보내는 흐름도 검증한다.

```text
CAN bus -> ESP32 A -> UART -> PC
```

### 7.4 CAN 프레임 확인 도구

`can-utils`가 설치되어 있으면 다음을 사용할 수 있다.

```bash
candump any
```

또는 특정 인터페이스를 지정해 확인한다.

```bash
candump can0
```

USB-CAN dongle 또는 CAN 인터페이스가 있다면 해당 인터페이스에서 프레임을 확인한다.

검증 포인트:
- CAN ID가 보여지는지
- DLC 길이가 맞는지
- payload가 예상한 값과 일치하는지
- 전송 주기가 20 Hz 또는 50 ms 수준인지

---

## 8. 검증 시나리오

### 시나리오 A: UART -> CAN

1. A 보드 USB/UART 연결
2. PC 시리얼 터미널에서 테스트 패킷 전송
3. A 보드의 로그 확인
4. B 보드의 로그 또는 CAN 도구로 프레임 확인
5. CAN ID, payload, checksum 확인

### 시나리오 B: CAN -> UART

1. A와 B 보드를 CAN으로 연결
2. B 보드에서 상태/액추에이터 메시지 전송
3. A 보드가 UART로 다시 내보내는지 확인
4. PC 시리얼 터미널에서 패킷 수신 확인

### 시나리오 C: A/B 동시 검증

1. A 보드와 B 보드 각각 모니터 실행
2. A 보드 UART 입력으로 테스트
3. B 보드에서 CAN 수신 로그 확인
4. A 보드에서 CAN 수신 재전송 로그 확인
5. UART와 CAN 양쪽이 모두 정상인지 비교

---

## 9. 자주 발생하는 문제

### 9.1 빌드 실패

```bash
source /home/user/.espressif/v6.0.2/esp-idf/export.sh
```

그리고 프로젝트 경로에서 다시 빌드한다.

### 9.2 포트 못 찾음

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
dmesg | tail -n 50
```

### 9.3 UART 수신 없음

- TX/RX가 반대로 연결되지 않았는지
- GND 연결이 되었는지
- baud rate가 115200인지
- 패킷 구조가 맞는지

### 9.4 CAN 수신 없음

- CAN_H/CAN_L 연결 상태 확인
- 공통 GND 연결 여부 확인
- termination resistor 확인
- 각 보드의 CAN transceiver 전원 확인
- GPIO17/18 사용 여부 확인

### 9.5 보드가 재시작 반복

- 전원 전압 부족 여부
- GND 흔들림 여부
- CAN transceiver 및 UART 변환기 GND 연결 확인

---

## 10. 최종 체크리스트

플래시 전:
- [ ] `idf.py build` 통과
- [ ] 포트 확인 완료
- [ ] A/B 전원, GND 연결 확인
- [ ] CAN transceiver 연결 확인
- [ ] UART adapter 연결 확인

플래시 후:
- [ ] A 보드 로그 확인
- [ ] B 보드 로그 확인
- [ ] UART 패킷 송수신 확인
- [ ] CAN 패킷 송수신 확인
- [ ] 각 보드의 모니터 로그가 정상 동작을 보여주는지 확인

---

## 11. 권장 실험 순서

1. A 보드 단독 flash 및 monitor 확인
2. B 보드 단독 flash 및 monitor 확인
3. CAN 버스 연결 후 A/B 시작 로그 확인
4. UART 테스트 패킷 전송 및 A 쪽 수신 확인
5. B 쪽 CAN 수신 및 응답 확인
6. 최종적으로 UART/CAN 양방향 생존 여부 확인

이 절차를 거치면, A와 B 펌웨어가 실제 하드웨어에서 서로 통신하는지 체계적으로 검증할 수 있다.
