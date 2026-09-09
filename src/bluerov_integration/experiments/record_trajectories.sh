#!/usr/bin/env bash
# 이미 돌고 있는 배치에 궤적 기록을 나중에 붙인다.
#
# auto_run.py 를 고쳐도 이미 실행 중인 파이썬 프로세스에는 반영되지 않는다
# (파이썬이 파일을 메모리에 올린 뒤라서). 그래서 이 스크립트를 옆에서
# 따로 띄워, 토픽이 살아 있는 동안 계속 기록한다.
#
#   ./record_trajectories.sh &
#
# 판이 바뀌면 토픽이 잠깐 끊겼다 다시 살아난다. 그때마다 새 파일로 나눈다.
# 파일명에 시각이 들어가므로, 나중에 planner 로그의 시각과 맞춰 어느 판인지
# 알 수 있다.

# set -u 를 쓰면 안 된다. ROS setup.bash 가 미설정 변수(COLCON_TRACE 등)를
# 참조해서 source 하는 순간 죽는다.
OUT="${RESULTS_DIR:-$HOME/manta_experiments/results}/traj"
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/manta_ws/install/setup.bash" 2>/dev/null

echo "[기록] 시작 — $OUT"
echo "[기록] 중단하려면 이 프로세스를 종료하세요"

cleanup() {
  echo "[기록] 정리 중..."
  pkill -P $$ 2>/dev/null
  exit 0
}
trap cleanup INT TERM

while true; do
  # 통합 노드가 떠서 어뢰 odometry 가 살아날 때까지 기다린다
  until ros2 topic list 2>/dev/null | grep -q "/torpedo/state/odometry"; do
    sleep 3
  done

  STAMP=$(date +%Y%m%d_%H%M%S)
  echo "[기록] 판 시작 감지 → ${STAMP}"

  ros2 topic echo --csv /torpedo/state/odometry nav_msgs/msg/Odometry \
    > "$OUT/${STAMP}_torpedo.csv" 2>/dev/null &
  TORPEDO_PID=$!
  ros2 topic echo --csv /model/bluerov2/odometry nav_msgs/msg/Odometry \
    > "$OUT/${STAMP}_bluerov.csv" 2>/dev/null &
  ROV_PID=$!

  # 토픽이 사라질 때까지(= 판이 끝날 때까지) 유지
  while ros2 topic list 2>/dev/null | grep -q "/torpedo/state/odometry"; do
    sleep 3
  done

  kill $TORPEDO_PID $ROV_PID 2>/dev/null
  wait $TORPEDO_PID $ROV_PID 2>/dev/null
  SIZE=$(du -h "$OUT/${STAMP}_torpedo.csv" 2>/dev/null | cut -f1)
  echo "[기록] 판 종료 → ${STAMP}_torpedo.csv (${SIZE:-0})"
  sleep 2
done
