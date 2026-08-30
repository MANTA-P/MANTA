# Teensy CAN 부하·E-stop 우선순위 시험 결과

## 1. 시험 목적

Teensy 4.1의 독립된 CAN1/CAN2 컨트롤러를 같은 Classical CAN 버스에 연결하고,
버스 부하가 높은 상태에서 E-stop CAN ID의 우선순위에 따른 송신 지연과
starvation 발생 여부를 확인한다.

## 2. 시험 구성

| 항목 | 설정 |
| --- | --- |
| 보드 | Teensy 4.1 |
| CAN | Classical CAN, 500 kbit/s, 표준 11-bit ID |
| CAN1 | dump 송신, TX pin 22 / RX pin 23 |
| CAN2 | E-stop 송신, TX pin 1 / RX pin 0 |
| 트랜시버 | CAN1/CAN2에 각각 1개, CANH/CANL/GND 공통 연결 |
| E-stop 입력 | pin 2, `INPUT_PULLUP`, Active LOW |
| 입력 방법 | 물리 스위치 대신 pin 2를 부팅 전부터 GND에 연결 |
| debounce | 50,000 us |
| E-stop 반복 | 3회, 성공한 송신 완료 후 1 ms 간격 |
| dump | DLC 8, 생성 주기 200 us |
| 통계 출력 | 1초 주기 |

pin 2를 부팅 전부터 GND에 연결했으므로 물리 edge는 발생하지 않았고,
`estop_raw_edges=0`, `estop_debounced_events=1`이 정상적으로 관찰되었다.

## 3. CAN ID 설정

| 모드 | E-stop ID | Dump ID | 예상 결과 |
| --- | ---: | ---: | --- |
| 정상 | `0x001` | `0x700` | 현재 프레임 종료 후 E-stop이 다음 중재에서 승리 |
| 역전 | `0x700` | `0x001` | dump가 항상 대기하면 E-stop starvation 가능 |

CAN ID 숫자가 작을수록 중재 우선순위가 높다. 높은 우선순위 프레임도 이미
전송을 시작한 프레임을 중단할 수는 없다.

## 4. 단일 dump mailbox 시험

초기 코드는 CAN1의 MB8 하나만 dump 송신에 사용했다. 송신 완료 후 메인
루프가 MB8을 다시 채우기 때문에 다음 dump가 CAN 하드웨어에 등록되기 전까지
짧은 중재 공백이 존재했다.

| 모드 | 최초 E-stop 송신 완료 지연 | 송신 결과 | Starvation |
| --- | ---: | --- | --- |
| 정상 | 391 us | 3/3 성공 | 없음 |
| 역전 | 466 us | 3/3 성공 | 없음 |

역전 모드는 정상 모드보다 75 us 느렸지만 E-stop은 모두 전송되었다. dump
샘플이 RAM에 최신 값으로 존재하더라도 `dumpCan.write()`로 CAN mailbox에
적재되기 전에는 버스 중재에 참여하지 않는다. E-stop이 이 재적재 공백에서
송신을 시작한 것으로 판단한다.

두 시험에서 관찰한 1초당 dump 변화량은 거의 같았다.

```text
dump_generated  +5000
dump_enqueued   +4177
dump_tx_success +4176
dump_dropped    +823
```

따라서 단일-mailbox 정상/역전 비교의 dump 생성 조건은 동일하게 유지되었다.
다만 각 모드의 유효한 active 표본은 `seq=0` 한 건뿐이므로, 391 us와 466 us의
차이를 통계적인 우선순위 효과로 확정하지는 않는다.

## 5. 2개 dump mailbox 시험

중재 공백을 줄이기 위해 CAN1의 MB8과 MB9를 ping-pong으로 사용하도록
변경했다.

```text
MB8: 현재 dump 송신
MB9: 다음 중재에 참여할 dump 대기
```

한 mailbox가 송신되는 동안 다른 mailbox에 다음 프레임을 대기시키고, 송신
완료된 mailbox를 다시 채운다. 두 mailbox가 모두 바쁜 동안 새로 생성되는
샘플은 RAM에서 최신 한 개만 유지한다.

### 5.1 역전 모드

대표 관찰 범위는 다음과 같다.

| uptime | dump TX 성공 | dump in flight | E-stop 요청 | E-stop 성공 | E-stop queue |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 11.345 s | 41,636 | 2 | 1 | 0 | 1 |
| 12.345 s | 45,843 | 2 | 1 | 0 | 1 |
| 13.345 s | 50,040 | 2 | 1 | 0 | 1 |
| 14.345 s | 54,235 | 2 | 1 | 0 | 1 |
| 15.345 s | 58,420 | 2 | 1 | 0 | 1 |
| 16.345 s | 62,618 | 2 | 1 | 0 | 1 |

```text
mode=inverted
dump_in_flight=2
estop_debounced_events=1
estop_tx_requests=1
estop_tx_success=0
estop_queue=1
estop_tx_fail_or_retry=0
can_error_count=0
bus_off_count=0
```

제공된 로그 구간에서 E-stop은 최소 5초 연속 송신 완료되지 않았으며,
마지막 관측 시점인 uptime 16.345초에도 CAN2에서 대기 중이었다. E-stop 송신
완료 callback이 발생하지 않았으므로 `LAT` 출력도 없었다.

E-stop 요청 자체는 CAN2 mailbox에 정상 적재되었고, CAN 컨트롤러가 중재에서
패배한 프레임을 자동으로 재시도한다. 중재 패배는 CAN 오류가 아니므로
`estop_tx_fail_or_retry`, `can_error_count`, `bus_off_count`는 모두 0으로
유지되었다.

### 5.2 정상 모드

```text
mode=normal
dump_in_flight=2
estop_tx_requests=3
estop_tx_success=3
estop_queue=0
request_to_first_tx_us=475
estop_tx_fail_or_retry=0
can_error_count=0
bus_off_count=0
```

dump mailbox 두 개가 계속 적재된 상태에서도 E-stop 3회가 모두 송신되었다.
최초 송신 완료 지연은 475 us였다. 이 시간에는 현재 전송 중이던 dump의 남은
시간과 E-stop 프레임 자체의 전송 시간이 포함된다.

## 6. 최종 비교

| Dump 구조 | 모드 | `dump_in_flight` | E-stop 결과 | 최초 완료 지연 |
| --- | --- | ---: | --- | ---: |
| mailbox 1개 | 정상 | 해당 통계 없음 | 3/3 성공 | 391 us |
| mailbox 1개 | 역전 | 해당 통계 없음 | 3/3 성공 | 466 us |
| mailbox 2개 | 정상 | 2 | 3/3 성공 | 475 us |
| mailbox 2개 | 역전 | 2 | 관측 종료까지 0/1 완료 | timeout/starvation |

2-mailbox 정상/역전 시험은 같은 200 us dump 생성 주기에서 수행되었고 두
모드 모두 CAN 오류와 bus-off가 없었다. 따라서 관찰된 차이는 물리 계층 오류가
아니라 CAN ID 중재 우선순위에 따른 결과로 판단한다.

## 7. 결론

1. 단일 dump mailbox에서는 송신 완료 후 재적재되는 사이에 중재 공백이 생겨
   역전 모드에서도 E-stop이 전송될 수 있었다.
2. 두 dump mailbox를 ping-pong으로 사용하자 `dump_in_flight=2`가 유지되었고,
   다음 중재마다 고우선순위 dump가 참여했다.
3. 정상 모드에서는 버스 부하가 계속된 상태에서도 E-stop이 475 us 안에 최초
   송신 완료되었다.
4. 역전 모드에서는 E-stop이 중재에서 계속 패배하여 starvation이 재현되었다.
5. 중재 패배는 CAN 오류가 아니므로 error 및 bus-off 카운터는 증가하지 않았다.

이번 시험으로 우선순위에 따른 정상 동작과 의도적인 starvation의 정성적 차이는
확인했다. 지연의 최소·평균·최대·p95를 산출하려면 정상 모드에서 서로 다른
sequence의 active 이벤트를 30회 이상 추가 측정해야 한다.

## 8. 코드 및 백업

- 현재 2-mailbox 코드: `teensy_can_load_estop/teensy_can_load_estop.ino`
- 설정: `teensy_can_load_estop/config.h`
- 단일-mailbox 백업:
  `teensy_can_load_estop/backup/teensy_can_load_estop_single_mailbox.ino.bak`
- 백업 설정:
  `teensy_can_load_estop/backup/config_single_mailbox.h.bak`

역전 모드는 실제 위험한 구동부와 분리된 시험 환경에서만 사용한다.
