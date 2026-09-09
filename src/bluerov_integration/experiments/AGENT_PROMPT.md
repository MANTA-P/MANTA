# 다른 AI 에이전트에게 줄 프롬프트

아래 블록을 통째로 복사해 GPT(또는 다른 코딩 에이전트)에 붙여넣으면 된다.
터미널을 실행할 수 있는 환경이어야 한다.

---

```
당신은 ROS 2 Jazzy + Gazebo 환경에서 수중로봇 회피 실험을 대신 돌려주는
역할입니다. 아래 규칙과 절차를 정확히 따라주세요.

## 실행 파일

    ~/manta_ws/src/bluerov_integration/experiments/auto_run.py

**절대경로로 부르면 어느 폴더에서 실행해도 됩니다.** cd 를 하지 않아도
됩니다. 편의상 아래처럼 별칭을 잡아두면 편합니다.

    RUN=~/manta_ws/src/bluerov_integration/experiments/auto_run.py

실험 로그와 결과는 저장소 밖에 쌓입니다:

    ~/manta_experiments/results/

## 절대 하지 말 것

1. `~/manta_ws/src/bluerov_integration/` 안에서 `experiments/` 폴더와
   `src/team_min/`, `include/bluerov_integration/team_min/` 외의 파일을
   수정하지 마세요. 다른 팀원 코드입니다.
2. git commit / push 를 하지 마세요. 변경 내용만 보고하세요.
3. 실험이 도는 중에 `colcon build` 를 하지 마세요. 바이너리가 바뀌면
   앞뒤 판의 조건이 달라져 데이터가 섞입니다.
4. 결과 폴더를 `rm` 으로 지우지 마세요. 아래 --reset-results 를 쓰면
   지우지 않고 보관 폴더로 옮깁니다.

## 빌드

**따로 colcon 명령을 칠 필요가 없습니다.** `--build` 를 붙이면 스크립트가
워크스페이스 루트를 찾아 필요한 패키지만 빌드하고 이어서 실행합니다.

    $RUN --batch --passes 3 --build

소스가 빌드보다 최신인데 `--build` 가 없으면 실행을 막습니다. 과거에 어뢰
바이너리가 10일 묵은 채로 16판을 돌려 전부 무효가 된 적이 있습니다.

colcon 을 직접 치겠다면 반드시 워크스페이스 루트에서 하세요. 다른 폴더에서
치면 거기에 build/ install/ log/ 를 새로 만들어 버립니다.

    cd ~/manta_ws && colcon build --packages-select bluerov_integration torpedo_control_v2

이전에 돌던 프로세스가 남아 있으면 반드시 정리하세요:

    pkill -f auto_run.py; sleep 3
    pkill -9 -f "gz sim"; pkill -9 -f dave_robot; pkill -9 -f rviz2
    pkill -9 -f "topic pub"

특히 `ros2 topic pub ... /mission/target_position` 이 남아 있으면 다음
실험의 목표점을 덮어써서 전부 무효가 됩니다. 반드시 확인하세요:

    ps -ef | grep -E "gz sim|dave_robot|bluerov_integration|topic pub" | grep -v grep

## 실험 실행 — 이 한 줄이면 됩니다

    ~/manta_ws/src/bluerov_integration/experiments/run_experiment.sh

이 스크립트가 잔류 프로세스 정리 → 빌드 → 이전 결과 보관 → 48판 실행 →
결과 보고까지 전부 합니다. 약 3시간 걸립니다.

앞 몇 판만 확인하려면 숫자를 붙이세요:

    ~/manta_ws/src/bluerov_integration/experiments/run_experiment.sh 5

개별 명령이 필요하면:

    RUN=~/manta_ws/src/bluerov_integration/experiments/auto_run.py
    $RUN front astar 3 --torpedo cheongsangeo    # 한 판만

## 결과 확인

    $RUN --report      # 표 + 회피 성공률 통계 (이걸 쓰세요)
    $RUN --coverage    # 48조합 중 무엇을 했고 무엇이 남았는지

결과는 판마다 즉시 `~/manta_experiments/results/results.csv` 에 append
됩니다. 중간에 멈춰도 그때까지의 결과는 남습니다.

## 실험 조건

  재공격 3회 절단   auto_run.py 안에 고정돼 있습니다. 바꿀 수 없습니다
  48조합            4방향 x 2플래너(astar/dvo) x 2모드(2/3) x 3어뢰

`--reuse-gazebo` 옵션은 폐기했습니다. 쓰지 마세요.

## 결과를 읽을 때 주의

  HIT                      피격 (최근접 < 1.0 m)
  AVOIDED                  회피 성공
  INVALID_NO_ENGAGEMENT    어뢰가 교전반경 30 m 안에 못 들어옴 → 무효
  INVALID_INTERRUPTED      실행이 강제 종료됨 → 무효 (성공으로 세지 마세요)
  INVALID_NO_PLAN          ROV가 안 움직임 → 무효
  FAIL_*                   셋업 실패

INVALID 로 시작하는 판은 회피 성능과 무관하므로 성공률 계산에서
제외해야 합니다. `--report` 가 자동으로 제외합니다.

## 문제가 생기면

  "소스가 빌드보다 최신입니다"  → 명령에 --build 를 붙여 재실행
  전부 INVALID_NO_MODE          → torpedo_control_v2 재빌드
  전부 INVALID_NO_PLAN          → 목표가 ROV에 전달 안 됨. topic pub 잔류 확인
  FAIL_GAZEBO                   → pkill -9 -f "gz sim" 후 재시도
  auto_run.py 없다고 나옴       → 절대경로로 부르세요(cd 불필요)

## 보고할 것

실험이 끝나거나 중단되면 아래를 보고하세요.

1. `$RUN --report` 출력 전체
2. 무효 판이 있으면 그 원인 (로그에서 확인)
3. 중간에 이상이 있었으면 그 시점과 증상

추측하지 말고 로그를 근거로 답해주세요. 로그 위치:

    ~/manta_experiments/results/<번호>_<조합>_<시각>_planner.log
```

---

## 프롬프트에 넣지 않은 것 (사람이 알아야 할 배경)

에이전트에게는 실행 절차만 줬다. 아래는 결과를 해석할 때 필요한 맥락이라
사람이 직접 봐야 한다.

- **실험 설계 근거**는 `RUNBOOK.md` 2장(축척)에 있다. 왜 어뢰 속도가
  5.35 / 6.62 / 8.64 m/s 인지, 왜 발사거리가 방향마다 다른지가 거기 있다.
- **지금까지 관측된 것**: 모드 3(PNG)은 회피 성공률 0%, 모드 2는 44%.
  피격은 전부 1회차 공격에서 났고 최근접이 0.55~0.99 m로 피격반경(1.0 m)
  언저리다. A\*는 장애물을 정확히 2.0 m만 비켜 가는 최단경로를 고른다.
- **다음 개선 후보**는 예측 모델이다. A\*/DVO 모두 어뢰를 등속 직진으로
  보는데, PNG 어뢰는 매 틱 재조준하므로 그 가정이 깨진다.
