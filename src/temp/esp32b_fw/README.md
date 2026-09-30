# ESP32 B 제어 이식

`main/control/`과 `main/torpedo_control_v2/`는 `torpedo_control_v2`의
Simple Tracking, PNG, 롤 제어, 핀 믹서, 설정을 가져온 코드다. 두 모드의 추력은
키보드 증감 대신 CAN 제어 명령의 `target_thrust`로 설정한다.

입력 경로는 다음과 같다.

1. CAN `0x110`(목표 BlueROV), `0x120`(어뢰)의 6개 조각을 각각 32바이트로 조립한다.
2. `control_algorithm.cpp`가 BE 정수를 m, m/s, rad/s 및 Q14 quaternion으로 변환한다.
3. 각 완성 odometry가 50 ms 이내이고 CAN `0x010` 명령이 200 ms 이내일 때
   20 Hz로 제어 계산을 수행한다. armed=0 또는 thrust=0이면 출력은 모두 0이다.
4. 핀 각도(rad)는 0.001 rad/LSB로 변환해 CAN `0x180`과 `0x181`로 보낸다.

빌드: `idf.py build` (ESP32-S3용 기존 `sdkconfig` 사용).

호스트 계산 테스트:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I. -Imain \
  -o /tmp/test_control_algorithm tests/test_control_algorithm.cpp \
  main/control_algorithm.cpp main/control/*.cpp
/tmp/test_control_algorithm
```

이 빌드와 호스트 테스트는 CAN 버스·실제 구동기를 검증하지 않는다. 실기 연결 전
핀 부호와 작동 방향, 50 ms odometry 수신 주기, 명령 timeout 및 출력 0 전환을 확인한다.
