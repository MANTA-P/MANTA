#!/usr/bin/env bash
#
# 회피 실험 셋업 자동화 + 결과 기록.
#
# A*/DVO/hybrid를 여러 방향에서 오는 어뢰에 대해 비교하려면 매번 manta1~5를
# 좌표까지 맞춰 띄워야 한다. 이 스크립트가 그 셋업을 대신하고, manta3/manta5
# 로그를 결과 폴더에 남겨 나중에 표로 뽑을 수 있게 한다.
#
#   ./avoid_experiment.sh rear --planner astar    # 실험 셋업
#   ./avoid_experiment.sh --summary               # 지금까지 결과를 표로
#
# 어뢰 발사만 사용자가 manta5 창에서 직접 한다(키보드 입력이라 자동화 불가):
#   1/2/3 으로 모드 선택 → r 로 추력
#
# manta.sh(팀 공용)는 수정하지 않고 호출만 한다.

set -Eeuo pipefail

readonly MANTA_SH="${MANTA_SH:-$HOME/manta_ws/manta.sh}"
readonly WORKSPACE="${WORKSPACE:-$HOME/manta_ws}"
# 로그는 저장소 밖(홈)에 쌓는다. 스크립트를 git에 올려도 로그는 안 딸려간다.
readonly RESULTS_DIR="${RESULTS_DIR:-$HOME/manta_experiments/results}"
readonly NODE_NAME="/bluerov_integration_node"

# ── 시나리오 정의 ────────────────────────────────────────────────────────────
# ROV는 원점(0,0,-1)에서 목표를 향해 -Y로 주행한다.
# 어뢰의 전방은 body +Y이고 yaw=0이면 world +Y로 발사되므로, 어뢰가 (tx,ty)에서
# (ax,ay)를 겨냥하게 하려면:  yaw = atan2(-(ax-tx), ay-ty)
#
# 형식: 어뢰x 어뢰y 어뢰z yaw 목표x 목표y 목표z 설명
scenario_front="0 -150 -1 0.0 0 -100 -1 정면충돌(접근15m/s)-예측이가장잘맞는조건"
scenario_rear="0 60 -1 3.1416 0 -100 -1 후방추격(접근9m/s)-A*가반응못할수있는조건"
scenario_side="-80 -50 -1 -1.5708 0 -100 -1 측면횡단-경로를가로지름"
scenario_diag="-70 -120 -1 -0.9505 0 -100 -1 대각접근-비스듬히요격"

usage() {
  cat <<'USAGE'
사용법: ./avoid_experiment.sh <시나리오> [옵션]
        ./avoid_experiment.sh --summary

시나리오:
  front   어뢰가 정면에서 마주 옴      (어뢰 0,-150   yaw 0)
  rear    어뢰가 뒤에서 추격           (어뢰 0,+60    yaw π)
  side    어뢰가 옆에서 가로지름       (어뢰 -80,-50  yaw -π/2)
  diag    어뢰가 비스듬히 접근         (어뢰 -70,-120 yaw -0.95)

옵션:
  --planner <astar|dvo|hybrid>  사용할 알고리즘 (기본 hybrid)
  --no-gazebo                   Gazebo/BlueROV는 이미 떠 있으니 재사용
  --summary                     results/ 로그를 파싱해 비교표 출력
  --list                        시나리오 좌표만 출력
  -h, --help                    이 도움말

실행 후 manta5 창에서:
  1 = Keyboard(수동·직진)   2 = SimpleTracking(추미)   3 = PNG(비례유도)
  r = 추력 증가   f = 감소   space = 정지
  ROV가 어느 정도 진행한 뒤 발사해야 회피 거동을 볼 수 있다.

실험 도중 알고리즘만 바꾸려면(시뮬 유지):
  ros2 param set /bluerov_integration_node planning.planner dvo
USAGE
}

log()  { printf '\033[1;36m[셋업]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[주의]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[오류]\033[0m %s\n' "$*" >&2; exit 1; }

source_ros() {
  set +u
  # shellcheck disable=SC1090,SC1091
  source /opt/ros/jazzy/setup.bash
  [[ -f "$WORKSPACE/install/setup.bash" ]] && source "$WORKSPACE/install/setup.bash"
  set -u
}

# ── 결과 요약 ────────────────────────────────────────────────────────────────
# manta3 로그에서 교전 결과·계획 성능을 뽑아 한 줄로 만든다.
summarize() {
  local files=("$RESULTS_DIR"/*_planner.log)
  if [[ ! -e "${files[0]}" ]]; then
    warn "결과 로그가 없습니다: $RESULTS_DIR"
    return 0
  fi

  printf '%-7s %-7s %-9s %-9s %-11s %-7s %-7s %s\n' \
    시나리오 플래너 결과 최근접m 평균계획ms 최대box VO실패 실행시각
  printf '%.0s─' {1..82}; printf '\n'

  local file
  for file in "${files[@]}"; do
    local base scenario planner stamp
    base="$(basename "$file" _planner.log)"
    scenario="${base%%_*}"
    planner="$(cut -d_ -f2 <<<"$base")"
    stamp="$(cut -d_ -f3-4 <<<"$base")"

    local outcome="진행중" distance="-"
    if grep -q "\[HIT\]" "$file"; then
      outcome="HIT"
      distance="$(grep -o "closest [0-9.]* m" "$file" | tail -1 | grep -o "[0-9.]*" || echo "-")"
    elif grep -q "\[AVOIDED\]" "$file"; then
      outcome="AVOIDED"
      distance="$(grep -o "min distance [0-9.]* m" "$file" | tail -1 | grep -o "[0-9.]*" | tail -1 || echo "-")"
    fi

    # "team_min <planner> time=0.345 ms, waypoints=.., boxes=.."
    local avg_ms max_box vo_fail
    avg_ms="$(grep -oE "time=[0-9.]+ ms" "$file" | grep -oE "[0-9.]+" \
      | awk '{s+=$1; n++} END {if (n) printf "%.2f", s/n; else print "-"}')"
    max_box="$(grep -oE "boxes=[0-9]+" "$file" | grep -oE "[0-9]+" \
      | sort -n | tail -1 || echo "-")"
    vo_fail="$(grep -c "no safe local path" "$file" || true)"

    printf '%-7s %-7s %-9s %-9s %-11s %-7s %-7s %s\n' \
      "$scenario" "$planner" "$outcome" "$distance" \
      "${avg_ms:--}" "${max_box:--}" "$vo_fail" "$stamp"
  done
  printf '\n로그 위치: %s\n' "$RESULTS_DIR"
}

# ── 실행 헬퍼 ────────────────────────────────────────────────────────────────
# 같은 이름의 노드가 둘이면 경로·마커가 이중 발행되어 RViz가 렉 걸리고 제어가
# 흔들린다. 실험 전에 반드시 정리한다.
cleanup_previous() {
  log "이전 실행 정리 중..."
  pkill -INT -f "bluerov_integration.launch.py" 2>/dev/null || true
  pkill -INT -f "target_position_input_node"    2>/dev/null || true
  pkill -INT -f "torpedo_sitl_v2.launch.py"     2>/dev/null || true
  if [[ "$reuse_gazebo" == "false" ]]; then
    pkill -INT -f "dave_robot.launch.py"        2>/dev/null || true
    pkill -INT -f "gz sim"                      2>/dev/null || true
  fi
  sleep 3
}

open_terminal() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '[$title 종료 — 닫으려면 exit]'; exec bash" &
  sleep 1
}

wait_for() {           # wait_for <topic|node> <이름> <제한초>
  local kind="$1" name="$2" limit="${3:-60}" waited=0
  while (( waited < limit )); do
    if ros2 "$kind" list 2>/dev/null | grep -qx "$name"; then return 0; fi
    sleep 2; waited=$((waited + 2)); printf '.'
  done
  printf '\n'; return 1
}

# ── 인자 파싱 ────────────────────────────────────────────────────────────────
scenario=""
planner="hybrid"
reuse_gazebo="false"

while (( $# > 0 )); do
  case "$1" in
    front|rear|side|diag) scenario="$1"; shift ;;
    --planner) [[ $# -ge 2 ]] || fail "--planner 뒤에 값이 필요합니다"; planner="$2"; shift 2 ;;
    --no-gazebo) reuse_gazebo="true"; shift ;;
    --summary) summarize; exit 0 ;;
    --list)
      for name in front rear side diag; do
        eval "value=\$scenario_$name"; printf '%-6s %s\n' "$name" "$value"
      done
      exit 0 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "알 수 없는 인자: $1  (--help 참고)" ;;
  esac
done

[[ -n "$scenario" ]] || { usage; exit 1; }
case "$planner" in
  astar|dvo|hybrid) ;;
  *) fail "planner는 astar, dvo, hybrid 중 하나여야 합니다 (받은 값: $planner)" ;;
esac
[[ -f "$MANTA_SH" ]] || fail "manta.sh를 찾을 수 없습니다: $MANTA_SH"
command -v gnome-terminal >/dev/null || fail "gnome-terminal이 필요합니다"

eval "config=\$scenario_$scenario"
read -r tx ty tz tyaw gx gy gz description <<<"$config"

mkdir -p "$RESULTS_DIR"
run_id="${scenario}_${planner}_$(date +%Y%m%d_%H%M)"
planner_log="$RESULTS_DIR/${run_id}_planner.log"
torpedo_log="$RESULTS_DIR/${run_id}_torpedo.log"

# ── 실행 ─────────────────────────────────────────────────────────────────────
source_ros

cat <<INFO

  시나리오 : $scenario  ($description)
  어뢰     : ($tx, $ty, $tz)  yaw=$tyaw
  목표     : ($gx, $gy, $gz)
  플래너   : $planner
  로그     : $planner_log

INFO

cleanup_previous

if [[ "$reuse_gazebo" == "true" ]]; then
  log "Gazebo 재사용 (--no-gazebo)"
else
  log "Gazebo + BlueROV 실행..."
  open_terminal "manta1 Gazebo+BlueROV" "bash '$MANTA_SH' bluerov 0 0 -1"
  # /model/bluerov2/odometry는 ros_gz 브리지(manta3)가 떠야 생기는 ROS
  # 토픽이라 여기서 기다리면 안 된다. Gazebo 안의 모델을 gz로 확인한다.
  printf '   Gazebo 모델 대기'
  local waited=0
  while (( waited < 180 )); do
    if gz topic -l 2>/dev/null | grep -q "/model/bluerov2/"; then break; fi
    sleep 3; waited=$((waited + 3)); printf '.'
  done
  (( waited < 180 )) || fail "BlueROV가 뜨지 않았습니다. manta1 창 로그를 확인하세요."
  printf ' 완료\n'
fi

# manta2는 yaw를 넘기지 않으므로 launch를 직접 호출한다(방향 지정이 핵심).
log "어뢰 스폰 (yaw=$tyaw)..."
open_terminal "manta2 어뢰스폰" \
  "source /opt/ros/jazzy/setup.bash && source '$WORKSPACE/install/setup.bash' && \
   ros2 launch dave_demos dave_robot.launch.py gui:=false \
   x:=$tx y:=$ty z:=$tz yaw:=$tyaw namespace:=glider_slocum \
   world_name:=dave_ocean_waves paused:=false use_teleop:=false open_qgc:=false"
printf '   어뢰 모델 대기'
waited=0
while (( waited < 90 )); do
  if gz topic -l 2>/dev/null | grep -q "/model/glider_slocum/"; then break; fi
  sleep 3; waited=$((waited + 3)); printf '.'
done
printf ' 완료\n'

log "통합 노드 실행 (로그: $(basename "$planner_log"))..."
open_terminal "manta3 통합노드" \
  "bash '$MANTA_SH' integration 2>&1 | tee '$planner_log'"
printf '   노드 대기'
wait_for node "$NODE_NAME" 120 \
  || fail "통합 노드가 뜨지 않았습니다. manta3 창 로그를 확인하세요."
printf ' 완료\n'

log "플래너 설정: $planner"
sleep 2
if ros2 param set "$NODE_NAME" planning.planner "$planner" >/dev/null 2>&1; then
  log "플래너 적용됨"
else
  warn "플래너 설정 실패 — 노드가 옛 빌드일 수 있습니다."
  warn "manta3 시작 로그에 'team_min planner:'가 있는지 확인하세요."
fi

log "어뢰 제어 창 실행..."
open_terminal "manta5 어뢰제어" \
  "bash '$MANTA_SH' torpedo-control 2>&1 | tee '$torpedo_log'"
sleep 3

log "미션 목표 발행: ($gx, $gy, $gz)"
open_terminal "manta4 목표입력" "bash '$MANTA_SH' target $gx $gy $gz"

cat <<READY

  ─────────────────────────────────────────────
  셋업 완료. manta5 창에서:

    1 / 2 / 3   어뢰 모드
                1=Keyboard(직진)  2=SimpleTracking(추미)  3=PNG(비례유도)
    r           추력 증가 (약 12 m/s까지)
    space       정지

  ROV가 어느 정도 진행한 뒤 발사해야 회피를 볼 수 있습니다.

  실험이 끝나면 결과 확인:
    ./avoid_experiment.sh --summary

  알고리즘만 바꿔 재실험(시뮬 유지):
    ros2 param set /bluerov_integration_node planning.planner dvo
    # 단, 로그 파일은 이 실행 것으로 계속 기록되므로
    # 정확한 비교를 원하면 스크립트를 --planner dvo 로 다시 실행하세요.
  ─────────────────────────────────────────────

READY
