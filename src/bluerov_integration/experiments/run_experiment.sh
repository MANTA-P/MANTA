#!/usr/bin/env bash
# 회피 실험 전체를 한 번에 돌린다.
#
# 에이전트(Codex 등)에게 넘길 때 명령이 하나면 승인도 한 번으로 끝난다.
# 잔류 프로세스 정리 -> 빌드 -> 결과 보관 -> 배치 실행 -> 보고까지 한다.
#
#   ./run_experiment.sh              전체 48판
#   ./run_experiment.sh 5            앞 5판만 (확인용)
#
# 판정 기준(재공격 3회)은 auto_run.py 안에 고정돼 있어 여기서 못 바꾼다.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN="$HERE/auto_run.py"
LIMIT="${1:-}"

say() { printf '\033[1;36m[실험]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[주의]\033[0m %s\n' "$*"; }

# ── 1. 잔류 프로세스 정리 ────────────────────────────────────────────────
# 특히 topic pub 이 남으면 다음 실험의 목표점을 덮어써 전부 무효가 된다.
say "이전 프로세스 정리..."
pkill -f "auto_run.py" 2>/dev/null
sleep 2
for pattern in "dave_robot.launch" "bluerov_integration.launch" "gz sim" \
               "gz-sim" "rviz2" "torpedo_sitl_v2" "topic pub"; do
  pkill -9 -f "$pattern" 2>/dev/null
done
sleep 3

LEFT=$(ps -eo cmd | grep -E "gz sim|dave_robot|bluerov_integration_node|topic pub" \
       | grep -v grep | wc -l)
if [ "$LEFT" -gt 0 ]; then
  warn "정리되지 않은 프로세스 ${LEFT}개가 남아 있다. 계속하면 결과가 오염된다."
  ps -eo pid,cmd | grep -E "gz sim|dave_robot|bluerov_integration_node|topic pub" \
    | grep -v grep
  exit 1
fi
say "정리 완료"

# ── 2. 이전 결과 보관 ────────────────────────────────────────────────────
"$RUN" --reset-results || exit 1

# ── 3. 실행 (빌드는 --build 가 알아서 한다) ──────────────────────────────
if [ -n "$LIMIT" ]; then
  say "앞 ${LIMIT}판만 실행"
  "$RUN" --batch --limit "$LIMIT" --build
else
  say "전체 48판 실행 (약 3시간)"
  "$RUN" --batch --build
fi

# ── 4. 보고 ──────────────────────────────────────────────────────────────
echo
say "최종 결과"
"$RUN" --report
