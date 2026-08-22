#!/usr/bin/env python3
"""회피 실험 자동 실행기.

어뢰 제어 노드가 /dev/tty에서 키를 읽으므로, pexpect로 가상 터미널(pty)에
띄워서 모드 선택(2/3)과 발사(r)를 프로그램으로 넣는다. 사람 손 없이
A*/DVO를 같은 조건으로 비교할 수 있다.

  ./auto_run.py rear astar 3          # 한 판
  ./auto_run.py --step                # 한 판씩 보며 판정·메모(중단/재개 가능)
  ./auto_run.py --batch               # 전체 조합 자동
  ./auto_run.py --summary             # 결과 표

실제 어뢰를 축소해 조건을 유도하려면 --torpedo를 쓴다. 속도·발사거리·
관찰시간이 torpedo_scaling.py의 축척에서 전부 계산된다.

  ./auto_run.py rear astar 3 --torpedo cheongsangeo

manta.sh(팀 공용)는 수정하지 않고 호출만 한다.
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import pexpect

import torpedo_scaling

HOME = Path.home()
MANTA_SH = Path(os.environ.get("MANTA_SH", HOME / "manta_ws" / "manta.sh"))
WORKSPACE = Path(os.environ.get("WORKSPACE", HOME / "manta_ws"))
# 로그는 저장소 밖(홈)에 쌓는다. 스크립트를 git에 올려도 실험 로그가
# 딸려가지 않고, 팀원끼리 로그가 섞이지도 않는다.
RESULTS = Path(os.environ.get("RESULTS_DIR",
                              HOME / "manta_experiments" / "results"))
NODE = "/bluerov_integration_node"

# 연료 소진(추력 차단) 후 타력으로 달리는 어뢰를 지켜보는 시간(초).
# 청상어 조건 6.62 m/s에서 추력을 끊으면 항력만으로 급감속한다
# (m dv/dt = -(8v + 12.474v²), m = 74.4 kg):
#   5초 -> 0.73 m/s (9.9 m),  10초 -> 0.29 m/s (12.2 m)
# 10초면 위협이 사라지므로 그때 판을 마감한다.
COAST_SEC = 10.0

# 어뢰 전방은 body +Y이고 yaw=0이면 world +Y로 발사된다.
# (tx,ty)에서 (ax,ay)를 겨냥: yaw = atan2(-(ax-tx), ay-ty)
SCENARIOS = {
    #          어뢰 x,   y,  z,   yaw,     목표 x,   y,  z,  설명
    "front": ((0.0, -150.0, -1.0, 0.0),    (0.0, -100.0, -1.0),
              "정면충돌 접근15m/s"),
    "rear":  ((0.0, 60.0, -1.0, 3.1416),   (0.0, -100.0, -1.0),
              "후방추격 접근9m/s"),
    "side":  ((-80.0, -50.0, -1.0, -1.5708), (0.0, -100.0, -1.0),
              "측면횡단"),
    "diag":  ((-70.0, -120.0, -1.0, -0.9505), (0.0, -100.0, -1.0),
              "대각접근"),
}
PLANNERS = ["astar", "dvo"]        # 배치 비교 대상(둘을 맞대어 본다)
# 대화형 선택에서는 둘을 섞은 hybrid도 고를 수 있다.
ALL_PLANNERS = ["astar", "dvo", "hybrid"]
PLANNER_DESC = {
    "astar": "A* 전역경로 (충돌 때만 재계획)",
    "dvo": "Dynamic VO 반응회피 (매 틱 재계산)",
    "hybrid": "A* 경로 + 위험하면 DVO 국소회피",
}
TORPEDO_MODES = {2: "SimpleTracking", 3: "PNG"}
# 어뢰 thrust_step=100, thrust_max=1000이라 'r' 한 번이 최대의 10%다.
# 어뢰 코드를 고치지 않고 누르는 횟수만으로 속도 단계를 만든다.
# 아래 속도는 speed_probe.py로 직접 측정한 값이다(모드1 직진 기준).
# glider_slocum은 최대 12.2 m/s(23.7 kt)가 천장이라, 실제 어뢰의 공격
# 속도(40~50 kt)는 재현되지 않는다. 탐색~고속순항 구간을 3등분했다.
THRUST_LEVELS = {"slow": 3, "mid": 5, "fast": 10}
SPEED_TABLE = {"slow": "6.5 m/s (12.7 kt)",
               "mid": "8.5 m/s (16.5 kt)",
               "fast": "12.2 m/s (23.7 kt)"}


def log(message):
    print(f"\033[1;36m[실험]\033[0m {message}", flush=True)


def warn(message):
    print(f"\033[1;33m[주의]\033[0m {message}", flush=True)


def ros_env():
    """ROS 환경을 상속한 env를 만든다(서브프로세스용)."""
    command = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WORKSPACE}/install/setup.bash && env"
    )
    output = subprocess.run(
        ["bash", "-c", command], capture_output=True, text=True, check=True
    ).stdout
    env = {}
    for line in output.splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            env[key] = value
    return env


ENV = None  # ros_env() 결과 캐시


def run_ros(args, timeout=20):
    return subprocess.run(
        args, env=ENV, capture_output=True, text=True, timeout=timeout
    )


def check_builds_fresh():
    """소스가 바이너리보다 최신이면 알린다.

    어뢰 바이너리가 10일 묵은 상태로 16판을 돌려 전부 무효가 된 적이 있다.
    실험 전에 반드시 확인한다.
    """
    stale = []
    for package in ("bluerov_integration", "torpedo_control_v2"):
        build_dir = WORKSPACE / "build" / package
        source_dir = WORKSPACE / "src" / package
        if not build_dir.exists() or not source_dir.exists():
            continue
        build_time = build_dir.stat().st_mtime
        newest = 0.0
        for pattern in ("**/*.cpp", "**/*.hpp"):
            for path in source_dir.glob(pattern):
                newest = max(newest, path.stat().st_mtime)
        if newest > build_time:
            stale.append(package)
    return stale


def cleanup(kill_gazebo=True):
    """이전 실행을 정리한다. 노드가 둘이면 경로가 이중 발행되어 실험이 망가진다."""
    patterns = [
        "bluerov_integration.launch.py",
        "target_position_input_node",
        "torpedo_sitl_v2.launch.py",
    ]
    if kill_gazebo:
        patterns += ["dave_robot.launch.py", "gz sim", "ruby.*gz"]
    for pattern in patterns:
        subprocess.run(["pkill", "-INT", "-f", pattern], capture_output=True)
    time.sleep(4)


def wait_for(kind, name, limit):
    """ROS topic/node가 나타날 때까지 기다린다."""
    deadline = time.time() + limit
    while time.time() < deadline:
        result = run_ros(["ros2", kind, "list"])
        if name in result.stdout.split():
            return True
        time.sleep(2)
    return False


def wait_for_log(path, marker, limit):
    """launch 로그에 특정 문구가 찍힐 때까지 기다린다.

    Gazebo가 떴는지는 ROS 토픽으로 알 수 없다 — /model/bluerov2/odometry는
    ros_gz 브리지(manta3)가 떠야 생기는 ROS 토픽이라서, 브리지보다 먼저
    기다리면 영원히 안 나온다. 그래서 launch 로그로 판단한다.
    """
    path = Path(path)
    deadline = time.time() + limit
    while time.time() < deadline:
        if path.exists() and marker in path.read_text(errors="ignore"):
            return True
        time.sleep(2)
    return False


def wait_for_gz_model(model, limit):
    """Gazebo 안에 모델이 실제로 올라왔는지 gz CLI로 확인한다."""
    deadline = time.time() + limit
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"], env=ENV,
                capture_output=True, text=True, timeout=15)
            if f"/model/{model}/" in result.stdout:
                return True
        except Exception:
            pass
        time.sleep(2)
    return False


def spawn_background(command, logfile=None):
    """백그라운드 프로세스를 띄운다(창 없이)."""
    handle = open(logfile, "w") if logfile else subprocess.DEVNULL
    return subprocess.Popen(
        ["bash", "-c", command],
        env=ENV, stdout=handle, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop(process):
    if process and process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass


def run_once(scenario, planner, mode, fire_delay=2.0, watch=45.0,
             reuse_gazebo=False, speed="fast", scaling=None):
    """실험 한 판. 결과 dict를 돌려준다.

    scaling을 주면(torpedo_scaling.scale() 결과) 어뢰 시작좌표·추력·관찰시간을
    실제 어뢰 제원에서 유도한 값으로 덮어쓴다. 없으면 기존 slow/mid/fast를 쓴다.
    """
    (_, _, _, _), (gx, gy, gz), description = SCENARIOS[scenario]
    if scaling:
        # 발사거리와 방향에서 시작좌표를 만든다. 방향마다 거리가 같아야
        # 방향끼리 공정하게 비교된다.
        tx, ty, tz, tyaw = torpedo_scaling.scenario_start(
            scenario, scaling["launch_sim"])
        # 임무거리도 축척에서 나온다. 판정이 끝날 때까지 ROV가 계속
        # 기동할 수 있는 길이라야 한다(아래에서 다시 검사한다).
        gx, gy, gz = 0.0, -round(scaling["mission_sim"], 1), -1.0
        presses = scaling["presses"]
        speed_key = scaling["key"]
        speed_label = (f"{scaling['spec']['label']} "
                       f"{scaling['speed_sim']:.2f} m/s")
        description = torpedo_scaling.DIRECTIONS[scenario][1]
    else:
        (tx, ty, tz, tyaw), _, _ = SCENARIOS[scenario]
        presses = THRUST_LEVELS[speed]
        speed_key = speed
        speed_label = SPEED_TABLE[speed]

    index = combo_index(scenario, planner, mode, speed_key)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_id = (f"{index:02d}_{scenario}_{planner}_mode{mode}_"
              f"{speed_key}_{stamp}")
    RESULTS.mkdir(exist_ok=True)
    planner_log = RESULTS / f"{run_id}_planner.log"
    torpedo_log = RESULTS / f"{run_id}_torpedo.log"

    log(f"── [{combo_label(index)}] {scenario}/{planner}/"
        f"모드{mode}({TORPEDO_MODES[mode]})"
        f"/{speed_key}({speed_label}) — {description}")

    # ROV가 관찰이 끝나기 전에 목표에 도착해 멈추면, 남은 시간 동안 어뢰가
    # 정지표적을 때린다. 그건 회피 성능이 아니라 도착 시간을 재는 것이므로
    # 아예 시작하지 않는다. (임무거리는 축척에서 이보다 길게 나오지만,
    # --watch를 손으로 늘렸을 때 이 조건이 깨질 수 있다.)
    mission_distance = abs(gy)
    arrival_sec = mission_distance / torpedo_scaling.ROV_SPEED
    if arrival_sec < watch:
        warn(f"   임무거리 {mission_distance:.0f} m는 {arrival_sec:.0f}초면 도착 "
             f"— 관찰 {watch:.0f}초를 못 채우고 ROV가 멈춘다")
        warn(f"   정지표적을 때리는 판이 되므로 중단한다. "
             f"목표를 {watch * torpedo_scaling.ROV_SPEED:.0f} m 이상으로 두거나 "
             f"--watch를 {arrival_sec:.0f}초 아래로 낮출 것")
        return {"run_id": run_id, "outcome": "INVALID_GOAL_TOO_NEAR",
                "distance": None, "plans": 0}
    processes = []
    try:
        cleanup(kill_gazebo=not reuse_gazebo)

        if not reuse_gazebo:
            log("   Gazebo + BlueROV (headless)...")
            gazebo_log = RESULTS / f"{run_id}_gazebo.log"
            processes.append(spawn_background(
                f"bash '{MANTA_SH}' bluerov 0 0 -1", gazebo_log))
            # ROS 토픽이 아니라 launch 로그/gz 토픽으로 판단해야 한다.
            if not wait_for_log(gazebo_log, "Robot Model Uploaded", 180):
                warn("   Gazebo 기동 실패")
                return {"run_id": run_id, "outcome": "FAIL_GAZEBO"}
            if not wait_for_gz_model("bluerov2", 60):
                warn("   BlueROV 모델이 Gazebo에 안 올라옴")
                return {"run_id": run_id, "outcome": "FAIL_GAZEBO"}
            log("   Gazebo 준비 완료")

        log(f"   어뢰 스폰 ({tx}, {ty}) yaw={tyaw}...")
        spawn_log = RESULTS / f"{run_id}_spawn.log"
        processes.append(spawn_background(
            f"ros2 launch dave_demos dave_robot.launch.py gui:=false "
            f"x:={tx} y:={ty} z:={tz} yaw:={tyaw} namespace:=glider_slocum "
            f"world_name:=dave_ocean_waves paused:=false use_teleop:=false "
            f"open_qgc:=false",
            spawn_log))
        if not wait_for_gz_model("glider_slocum", 90):
            warn("   어뢰 모델이 Gazebo에 안 올라옴")
            return {"run_id": run_id, "outcome": "FAIL_TORPEDO"}
        log("   어뢰 스폰 완료")

        log("   통합 노드...")
        processes.append(spawn_background(
            f"bash '{MANTA_SH}' integration", planner_log))
        if not wait_for("node", NODE, 150):
            warn("   통합 노드 기동 실패")
            return {"run_id": run_id, "outcome": "FAIL_NODE"}
        # 브리지가 붙어야 odometry가 ROS로 나온다.
        if not wait_for("topic", "/model/bluerov2/odometry", 60):
            warn("   브리지가 odometry를 못 내보냄")
            return {"run_id": run_id, "outcome": "FAIL_BRIDGE"}
        log("   통합 노드 준비 완료")
        time.sleep(3)

        result = run_ros(
            ["ros2", "param", "set", NODE, "planning.planner", planner])
        if "Set parameter successful" not in result.stdout:
            warn(f"   플래너 설정 실패: {result.stdout.strip()[:80]}")
        else:
            log(f"   플래너 = {planner}")

        # 어뢰 제어를 pty로 띄운다. /dev/tty가 이 pty가 되므로 키를 넣을 수 있다.
        log("   어뢰 제어(pty)...")
        child = pexpect.spawn(
            "bash", ["-c", f"bash '{MANTA_SH}' torpedo-control"],
            env=ENV, encoding="utf-8", timeout=30, dimensions=(40, 120))
        with open(torpedo_log, "w") as handle:
            child.logfile_read = handle

            def screen():
                """pty를 비우고 방금 나온 화면을 돌려준다."""
                try:
                    return child.read_nonblocking(size=400000, timeout=2)
                except Exception:
                    return ""

            def current_mode_line():
                clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", screen())
                lines = [l.strip() for l in clean.splitlines()
                         if l.strip().startswith("MODE    :")]
                return lines[-1] if lines else ""

            # 준비(센서 확인·모드 전환)를 목표 발행 '전에' 끝낸다.
            # 목표를 먼저 주면 ROV가 그만큼 앞서가 어뢰가 불리해진다.
            log("   센서 OK 대기...")
            buffer, sensors_ok = "", False
            deadline = time.time() + 45
            while time.time() < deadline:
                buffer += screen()
                clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", buffer)
                sensor_lines = [l for l in clean.splitlines()
                                if l.strip().startswith("SENSOR")]
                if sensor_lines and "FAIL" not in sensor_lines[-1]:
                    sensors_ok = True
                    break
            if not sensors_ok:
                warn("   센서 OK 못 봄 — 모드가 안 먹을 수 있음")

            # 모드 전환은 한 번에 안 먹을 수 있으므로 확인될 때까지 재시도한다.
            mode_ok = False
            for _ in range(5):
                child.send(str(mode))
                until = time.time() + 4
                while time.time() < until:
                    if f"[{mode}]" in current_mode_line():
                        mode_ok = True
                        break
                if mode_ok:
                    break
            log(f"   모드 {mode}({TORPEDO_MODES[mode]}) "
                f"{'전환 확인' if mode_ok else '전환 실패'}")

            # 이제 목표를 주고, 짧게 기다렸다 바로 발사한다.
            # manta4(target)는 대화형이라 stdin 없이 띄우면 즉시 끝나고
            # transient_local 목표가 퍼블리셔와 함께 사라지므로 직접 발행한다.
            log(f"   목표 발행 ({gx}, {gy}, {gz})")
            processes.append(spawn_background(
                "ros2 topic pub -r 1 /mission/target_position "
                "geometry_msgs/msg/PointStamped "
                f"'{{header: {{frame_id: map}}, "
                f"point: {{x: {gx}, y: {gy}, z: {gz}}}}}' "
                "--qos-durability transient_local --qos-reliability reliable",
                RESULTS / f"{run_id}_target.log"))

            log(f"   {fire_delay}초 후 발사...")
            fire_at = time.time() + fire_delay
            while time.time() < fire_at:
                screen()

            # 속도 단계만큼 'r'을 눌러 추력을 올린다(어뢰 코드는 그대로).
            for _ in range(presses):
                child.send("r")
                time.sleep(0.25)
            thrust_line = ""
            clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", screen())
            for line in clean.splitlines():
                if line.strip().startswith("THRUST"):
                    thrust_line = line.strip()
            log(f"   발사 (모드 {mode} = {TORPEDO_MODES[mode]}, {thrust_line})")

            # 어뢰 연료가 다할 때까지 지켜본다. 회피 성공은 "한 번 피했다"가
            # 아니라 "연료가 마를 때까지 버텼다"이므로, AVOIDED가 떠도 끊지
            # 않는다(유도어뢰는 선회해 재공격한다). HIT이면 그 자리에서 끝.
            outcome, distance, passes = "TIMEOUT", None, 0
            deadline = time.time() + watch
            while time.time() < deadline:
                # 관찰 중에도 pty를 비운다. 버퍼가 차면 어뢰 노드가 화면
                # 출력에서 막혀 제어를 멈춘다.
                screen()
                if not planner_log.exists():
                    continue
                text = planner_log.read_text(errors="ignore")
                hit = re.search(r"\[HIT\].*?closest ([0-9.]+) m", text)
                if hit:
                    outcome, distance = "HIT", float(hit.group(1))
                    passes = len(re.findall(r"engagement started", text))
                    log(f"   피격 — {passes}회차 공격에서 (최근접 "
                        f"{distance} m)")
                    break

            if outcome != "HIT":
                # === 연료 소진 ===
                # 시뮬레이터에는 연료 모델이 없어 어뢰가 무한히 달린다.
                # 그래서 축척에서 계산한 운행시간이 지나면 스페이스(=
                # ThrottleStop)를 넣어 추력을 0으로 만든다. 어뢰 코드는
                # 그대로 두고 키 입력만 흉내내는 방식이다.
                # 추력이 끊긴 어뢰는 항력으로 급감속한다(10초에 0.29 m/s).
                child.send(" ")
                log(f"   연료 소진 ({watch:.0f}초) — 추력 차단, 타력 주행")

                # 타력으로 달리는 동안에도 맞을 수 있으므로 계속 본다.
                coast_deadline = time.time() + COAST_SEC
                while time.time() < coast_deadline:
                    screen()
                    text = planner_log.read_text(errors="ignore")
                    hit = re.search(r"\[HIT\].*?closest ([0-9.]+) m", text)
                    if hit:
                        outcome, distance = "HIT", float(hit.group(1))
                        passes = len(re.findall(r"engagement started", text))
                        log(f"   타력 주행 중 피격 (최근접 {distance} m)")
                        break

            if outcome != "HIT":
                # 연료 소진까지 버텼다.
                #
                # 공격 횟수는 [AVOIDED]가 아니라 "engagement started"로 센다.
                # 어뢰가 교전반경 안에 계속 붙어 쫓아다니면 교전이 끝나지
                # 않아 [AVOIDED]가 한 번도 안 찍히는데, 그걸 "교전 없음"으로
                # 보면 정상적인 회피 성공을 무효 처리하게 된다.
                text = planner_log.read_text(errors="ignore")
                attacks = len(re.findall(r"engagement started", text))
                margins = [float(v) for v in re.findall(
                    r"\[AVOIDED\].*?min distance ([0-9.]+) m", text)]
                passes = attacks
                if attacks:
                    outcome = "AVOIDED"
                    if margins:
                        distance = min(margins)
                        log(f"   연료 소진까지 생존 — 공격 {attacks}회, "
                            f"최소 이격 {distance} m")
                    else:
                        # 교전이 끝나지 않은 채 연료가 말랐다. 최근접거리는
                        # 코어가 교전 종료 때만 로그하므로 알 수 없다.
                        entry = re.search(
                            r"engagement started \(torpedo ([0-9.]+) m", text)
                        log(f"   연료 소진까지 생존 — 교전 지속 중 종료 "
                            f"(진입 {entry.group(1) if entry else '?'} m, "
                            f"최근접 미상)")
                else:
                    # 어뢰가 교전반경(30 m) 안에 한 번도 못 들어왔다.
                    # 회피 성능을 잰 판이 아니다.
                    outcome = "INVALID_NO_ENGAGEMENT"
                    warn("   어뢰가 교전반경 안에 한 번도 못 들어왔다 — 결과 무효")
            # ROV가 실제로 계획·주행했는지 확인한다. 목표가 전달되지 않아
            # 제자리에 서 있었는데 어뢰가 빗나간 것도 [AVOIDED]로 찍히므로,
            # 계획이 돌지 않았으면 결과를 믿을 수 없다.
            text = planner_log.read_text(errors="ignore")
            plan_count = len(re.findall(rf"team_min {planner} time=", text))
            if plan_count == 0:
                warn("   경로계획 0회 — ROV가 움직이지 않았으므로 결과 무효")
                outcome = "INVALID_NO_PLAN"
            elif not mode_ok:
                warn("   어뢰 모드 전환 실패 — 유도가 안 걸렸으므로 결과 무효")
                outcome = "INVALID_NO_MODE"
            log(f"   결과: {outcome}"
                + (f" (최근접 {distance} m, 계획 {plan_count}회)"
                   if distance else f" (계획 {plan_count}회)"))

        child.close(force=True)
        return {"run_id": run_id, "scenario": scenario, "planner": planner,
                "mode": mode, "speed": speed_key, "outcome": outcome,
                "distance": distance, "plans": plan_count,
                "passes": passes, "log": planner_log}
    finally:
        for process in reversed(processes):
            stop(process)
        cleanup(kill_gazebo=not reuse_gazebo)


# ── 대화형 모드(--step) ──────────────────────────────────────────────────────
# RViz로 한 판씩 직접 보고 판단·메모한 뒤 다음으로 넘어간다.
# 48회를 한 번에 다 볼 수 없으므로 CSV에 즉시 기록하고 재개를 지원한다.
STEP_CSV = RESULTS / "step_results.csv"
STEP_FIELDS = ["timestamp", "scenario", "planner", "mode", "speed",
               "auto_outcome", "auto_distance", "plans", "verdict",
               "memo", "run_id"]


def load_done_combos():
    """이미 기록된 조합을 (시나리오, 플래너, 모드, 속도) 집합으로 돌려준다."""
    if not STEP_CSV.exists():
        return set()
    done = set()
    with open(STEP_CSV, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                done.add((row["scenario"], row["planner"],
                          int(row["mode"]), row["speed"]))
            except (KeyError, ValueError):
                continue
    return done


def append_step_row(row):
    """한 판 끝날 때마다 즉시 append 한다(중간에 죽어도 앞선 결과 보존)."""
    STEP_CSV.parent.mkdir(parents=True, exist_ok=True)
    is_new = not STEP_CSV.exists()
    with open(STEP_CSV, "a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=STEP_FIELDS)
        if is_new:
            writer.writeheader()
        writer.writerow(row)


def ask_verdict(index, total, scenario, planner, mode, speed, result):
    """자동판정을 보여주고 사용자 확인·메모를 받는다.

    돌려주는 값: ("record", verdict, memo) / ("retry", ...) /
                 ("skip", ...) / ("quit", ...)
    """
    outcome = result.get("outcome", "?")
    distance = result.get("distance")
    plans = result.get("plans", 0)
    detail = f"{outcome}"
    if distance is not None:
        detail += f" (최근접 {distance} m"
        detail += f", 계획 {plans}회)"
    else:
        detail += f" (계획 {plans}회)"

    print("\n" + "─" * 60)
    print(f"[{index}/{total}] {scenario} / {planner} / "
          f"모드{mode}({TORPEDO_MODES[mode]}) / {speed}({SPEED_TABLE[speed]})")
    print(f"자동판정: {detail}")
    print()
    print("  Enter  자동판정대로 기록하고 다음")
    print("  a      AVOIDED로 정정")
    print("  h      HIT으로 정정")
    print("  r      이 판 다시 실행")
    print("  s      건너뛰기(기록 안 함)")
    print("  q      중단 (다음에 이어서)")
    print()

    while True:
        try:
            choice = input("판정 [Enter/a/h/r/s/q] > ").strip().lower()
        except EOFError:
            return "quit", None, ""
        if choice in ("", "a", "h", "r", "s", "q"):
            break
        print("  Enter, a, h, r, s, q 중에서 골라주세요.")

    if choice == "q":
        return "quit", None, ""
    if choice == "r":
        return "retry", None, ""
    if choice == "s":
        return "skip", None, ""

    verdict = {"a": "AVOIDED", "h": "HIT"}.get(choice, outcome)
    try:
        memo = input("메모 > ").strip()
    except EOFError:
        memo = ""
    return "record", verdict, memo


def all_combos():
    """전체 실험 조합을 정해진 순서로 나열한다(시나리오 -> 플래너 -> 모드 -> 어뢰).

    4 x 2 x 2 x 3 = 48회. 이 순서가 곧 실험 번호가 되므로, 어떤 순서로
    돌리든 같은 조합은 늘 같은 번호를 갖는다.
    """
    return [(s, p, m, v) for s in SCENARIOS for p in PLANNERS
            for m in TORPEDO_MODES for v in torpedo_scaling.TORPEDOES]


def combo_index(scenario, planner, mode, speed_key):
    """이 조합이 48회 중 몇 번째인지. 표에 없는 조합(hybrid 등)은 0."""
    key = (scenario, planner, mode, speed_key)
    combos = all_combos()
    return combos.index(key) + 1 if key in combos else 0


def combo_label(index):
    return f"{index:02d}/{len(all_combos())}" if index else "번외"


def combo_speeds(args):
    """배치/단계 실행에서 돌 속도축을 정한다.

    기본은 어뢰 3종(축척 기반)이다. --legacy-speed를 주면 예전 slow/mid/fast를
    쓴다(이전에 모은 데이터와 이어붙일 때만).
    """
    if args.legacy_speed:
        return [args.speed] if args.speed else list(THRUST_LEVELS)
    if args.torpedo:
        return [args.torpedo]
    return list(torpedo_scaling.TORPEDOES)


def scaling_for(speed_key, args, setting=None):
    """속도축 값이 어뢰 이름이면 축척을 만들고, 아니면 None."""
    if speed_key not in torpedo_scaling.TORPEDOES:
        return None
    return torpedo_scaling.scale(
        speed_key, args.tau, launch_sim=args.launch,
        passes_target=args.passes, search_fraction=args.search,
        speed_setting=setting or args.speed_setting or "high")


def run_step_mode(args):
    """조합을 한 판씩 돌리며 사용자 판단을 받는다."""
    # 인자를 주면 그 축만 돌린다(예: rear만, dvo만). 안 주면 전체 조합.
    scenarios = [args.scenario] if args.scenario else list(SCENARIOS)
    planners = [args.planner] if args.planner else PLANNERS
    modes = [args.mode] if args.mode else list(TORPEDO_MODES)
    speeds = combo_speeds(args)
    combos = [(s, p, m, v) for s in scenarios for p in planners
              for m in modes for v in speeds]

    done = set() if args.restart else load_done_combos()
    if args.restart and STEP_CSV.exists():
        STEP_CSV.unlink()
        warn("기존 기록을 지우고 처음부터 시작합니다.")

    remaining = [c for c in combos if c not in done]
    if not remaining:
        log(f"모든 조합({len(combos)}개)이 이미 기록되어 있습니다.")
        log(f"다시 하려면: ./auto_run.py --step --restart")
        return

    log(f"전체 {len(combos)}개 중 {len(remaining)}개 남음 "
        f"(완료 {len(done)}개)")
    log("RViz 창을 보며 판단하세요. q로 중단해도 다음에 이어집니다.\n")

    for offset, (scenario, planner, mode, speed) in enumerate(remaining, 1):
        index = len(done) + offset
        while True:                       # 'r'(재실행) 처리
            try:
                scaling = scaling_for(speed, args)
                watch = (scaling["judge_time"] if scaling else args.watch)
                result = run_once(scenario, planner, mode,
                                  args.fire_delay, watch, speed=speed,
                                  scaling=scaling)
            except Exception as error:
                warn(f"실행 실패: {error}")
                result = {"outcome": "ERROR", "distance": None,
                          "plans": 0, "run_id": "-"}

            action, verdict, memo = ask_verdict(
                index, len(combos), scenario, planner, mode, speed, result)

            if action == "retry":
                log("다시 실행합니다...")
                continue
            if action == "quit":
                log(f"중단했습니다. 지금까지 {index - 1}개 기록됨.")
                log(f"이어서 하려면: ./auto_run.py --step")
                return
            if action == "skip":
                log("건너뜁니다(기록 안 함).")
                break

            append_step_row({
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "scenario": scenario, "planner": planner,
                "mode": mode, "speed": speed,
                "auto_outcome": result.get("outcome", "?"),
                "auto_distance": result.get("distance", ""),
                "plans": result.get("plans", 0),
                "verdict": verdict, "memo": memo,
                "run_id": result.get("run_id", "-"),
            })
            log(f"기록됨 → {STEP_CSV.name}")
            break

    log("\n모든 조합을 마쳤습니다.")
    show_step_results()


def show_step_results():
    """사용자 판정이 담긴 CSV를 표로 보여준다."""
    if not STEP_CSV.exists():
        return
    with open(STEP_CSV, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return
    print(f"\n{'시나리오':<8}{'플래너':<7}{'모드':<4}{'속도':<6}"
          f"{'판정':<9}{'최근접m':<9}{'계획':<6}메모")
    print("─" * 78)
    for row in rows:
        differs = row["verdict"] != row["auto_outcome"]
        verdict = row["verdict"] + ("*" if differs else "")
        print(f"{row['scenario']:<8}{row['planner']:<7}{row['mode']:<4}"
              f"{row['speed']:<6}{verdict:<9}"
              f"{(row['auto_distance'] or '-'):<9}{row['plans']:<6}"
              f"{row['memo']}")
    print(f"\n* = 자동판정과 다르게 정정한 판")
    print(f"기록: {STEP_CSV}\n")


def show_coverage():
    """48조합 중 무엇을 했고 무엇이 남았는지 보여준다.

    번호는 조합 고유번호라 실행 순서와 무관하다. 그래서 여러 날에 나눠
    돌려도 "전체 중 어디까지 왔는지"를 한눈에 볼 수 있다.
    """
    done = {}
    for path in sorted(RESULTS.glob("*_planner.log")):
        parts = path.name.replace("_planner.log", "").split("_")
        if not parts or not parts[0].isdigit():
            continue
        index = int(parts[0])
        text = path.read_text(errors="ignore")
        hit = re.search(r"\[HIT\].*?closest ([0-9.]+) m", text)
        if hit:
            done[index] = f"HIT {hit.group(1)}m"
        elif "engagement started" in text:
            done[index] = "AVOIDED"
        else:
            done[index] = "무효"

    combos = all_combos()
    print(f"\n전체 {len(combos)}조합 중 {len(done)}개 완료 "
          f"({len(done)*100//len(combos)}%)\n")
    print(f"{'번호':<7}{'시나리오':<9}{'플래너':<8}{'모드':<6}{'어뢰':<14}결과")
    print("─" * 62)
    for number, (scenario, planner, mode, speed) in enumerate(combos, 1):
        mark = done.get(number, "")
        print(f"{number:02d}/{len(combos)}  {scenario:<9}{planner:<8}"
              f"{mode:<6}{speed:<14}{mark if mark else '· 미실행'}")
    print()


def summarize():
    logs = sorted(RESULTS.glob("*_planner.log"))
    if not logs:
        warn(f"결과 로그 없음: {RESULTS}")
        return
    print(f"\n{'번호':<8}{'시나리오':<9}{'플래너':<8}{'모드':<18}"
          f"{'어뢰':<14}{'결과':<10}{'최근접m':<9}{'평균ms':<8}"
          f"{'box':<6}{'VO실패'}")
    print("─" * 100)
    for path in logs:
        name = path.name.replace("_planner.log", "")
        parts = name.split("_")
        # 새 이름은 "05_front_astar_mode3_cheongsangeo_시각", 옛 이름은
        # 번호 없이 "front_astar_...". 앞이 숫자면 번호를 떼어낸다.
        if parts and parts[0].isdigit():
            parts = parts[1:]
        if len(parts) < 3:
            continue
        scenario, planner = parts[0], parts[1]
        mode = parts[2].replace("mode", "")
        # 속도 자리에는 slow/mid/fast 또는 --torpedo의 어뢰 이름이 온다.
        speed_keys = set(THRUST_LEVELS) | set(torpedo_scaling.TORPEDOES)
        speed = parts[3] if len(parts) > 3 and parts[3] in speed_keys else "-"
        mode_label = f"{mode}({TORPEDO_MODES.get(int(mode), '?')})" \
            if mode.isdigit() else mode
        text = path.read_text(errors="ignore")

        hit = re.search(r"\[HIT\].*?closest ([0-9.]+) m", text)
        avoided = re.search(r"\[AVOIDED\].*?min distance ([0-9.]+) m", text)
        if hit:
            outcome, distance = "HIT", hit.group(1)
        elif avoided:
            outcome, distance = "AVOIDED", avoided.group(1)
        else:
            outcome, distance = "미판정", "-"

        times = [float(v) for v in re.findall(r"time=([0-9.]+) ms", text)]
        boxes = [int(v) for v in re.findall(r"boxes=([0-9]+)", text)]
        vo_fail = text.count("no safe local path")
        # 계획이 0회면 ROV가 안 움직인 것이므로 결과를 신뢰할 수 없다.
        if not times:
            outcome = "무효(계획0)"
        index = combo_index(scenario, planner,
                            int(mode) if mode.isdigit() else 0, speed)
        print(f"{combo_label(index):<8}{scenario:<9}{planner:<8}"
              f"{mode_label:<18}{speed:<14}{outcome:<10}{distance:<9}"
              f"{(f'{sum(times)/len(times):.2f}' if times else '-'):<8}"
              f"{(str(max(boxes)) if boxes else '-'):<6}{vo_fail}")
    print(f"\n로그: {RESULTS}\n")


# ── 대화형 선택 ─────────────────────────────────────────────────────────────
# 인자 없이 실행하면 방향·알고리즘·어뢰모드·속도를 차례로 고르게 한다.
# 좌표나 추력 횟수를 외우지 않아도 한 판을 돌릴 수 있다.
def choose(title, rows, default_key=None):
    """rows = [(키, 이름, 설명), ...]. 번호나 이름으로 고른다."""
    print(f"\n\033[1m{title}\033[0m")
    for number, (key, name, description) in enumerate(rows, 1):
        mark = "  ← 기본" if key == default_key else ""
        print(f"  {number}) {name:<9}{description}{mark}")
    keys = [row[0] for row in rows]
    hint = f"[1-{len(rows)}, Enter=기본]" if default_key else f"[1-{len(rows)}]"
    while True:
        try:
            answer = input(f"선택 {hint} > ").strip().lower()
        except EOFError:
            sys.exit("\n입력이 없어 중단합니다.")
        if not answer and default_key:
            return default_key
        if answer.isdigit() and 1 <= int(answer) <= len(rows):
            return keys[int(answer) - 1]
        if answer in keys:
            return answer
        print("  번호나 이름으로 골라주세요.")


def pick_settings(args):
    """명령줄에 없는 항목만 물어본다.

    --legacy-speed면 예전 slow/mid/fast를, 아니면 어뢰 -> 속도설정 순으로
    고른다. 어뢰를 고르면 최대거리가, 속도를 고르면 운행시간이 정해진다.
    """
    scenario = args.scenario or choose(
        "1. 어뢰가 오는 방향",
        [(key, key, SCENARIOS[key][2]) for key in SCENARIOS])
    planner = args.planner or choose(
        "2. 회피 알고리즘",
        [(key, key, PLANNER_DESC[key]) for key in ALL_PLANNERS])
    mode = args.mode or int(choose(
        "3. 어뢰 유도 모드",
        [(str(key), f"모드{key}", name) for key, name in TORPEDO_MODES.items()]))

    if args.legacy_speed:
        speed = args.speed or choose(
            "4. 어뢰 속도",
            [(key, key, SPEED_TABLE[key]) for key in THRUST_LEVELS],
            default_key="fast")
        print(f"\n\033[1m→ {scenario} / {planner} / 모드{mode}"
              f"({TORPEDO_MODES[mode]}) / {speed}({SPEED_TABLE[speed]})\033[0m\n")
        return scenario, planner, mode, speed, None

    # 어뢰를 고르면 공표 속도와 최대거리가 정해진다.
    torpedo = args.torpedo or choose(
        "4. 어뢰 (공표 제원 = 고속 설정 기준)",
        [(key, spec["label"],
          f"{spec['kind']}  {spec['speed_kt']:.0f} kt / {spec['range_km']:.0f} km")
         for key, spec in torpedo_scaling.TORPEDOES.items()])

    # 속도를 낮추면 같은 연료로 더 멀리 간다(R ∝ 1/v²). 그래서 설정마다
    # 최대거리와 운행시간이 달라지고, 그게 곧 판정시간이 된다.
    setting = args.speed_setting or choose(
        f"5. {torpedo_scaling.TORPEDOES[torpedo]['label']} 속도 설정",
        [(key, key, _setting_summary(torpedo, key, args))
         for key in torpedo_scaling.SPEED_SETTINGS],
        default_key="high")

    scaling = scaling_for(torpedo, args, setting)
    print(f"\n\033[1m→ {scenario} / {planner} / 모드{mode}"
          f"({TORPEDO_MODES[mode]}) / "
          f"{scaling['spec']['label']} {scaling['speed_setting_label']}\033[0m")
    print(torpedo_scaling.describe(scaling) + "\n")
    return scenario, planner, mode, torpedo, scaling


def _setting_summary(torpedo, setting, args):
    """속도 설정 하나를 고르면 어떻게 되는지 한 줄로."""
    s = scaling_for(torpedo, args, setting)
    return (f"{s['speed_real']/torpedo_scaling.KNOT:.0f} kt  "
            f"최대 {s['range_real']/1000:.1f} km  "
            f"운행 {s['endurance_real']/60:.0f}분  →  "
            f"판정 {s['judge_time']:.0f}초 / 임무 {s['mission_sim']:.0f} m "
            f"/ 한 판 {(90 + s['judge_time'])/60:.1f}분")


def main():
    global ENV
    parser = argparse.ArgumentParser(description="회피 실험 자동 실행")
    parser.add_argument("scenario", nargs="?", choices=list(SCENARIOS),
                        help="생략하면 물어본다")
    parser.add_argument("planner", nargs="?", choices=ALL_PLANNERS,
                        help="생략하면 물어본다")
    parser.add_argument("mode", nargs="?", type=int, choices=[2, 3],
                        help="어뢰 유도 모드. 생략하면 물어본다")
    parser.add_argument("--speed", choices=list(THRUST_LEVELS),
                        help="어뢰 속도 단계 (slow 6.5 / mid 8.5 / fast 12.2 m/s). "
                             "물어볼 때 Enter를 치면 fast. "
                             "--batch/--step에서 생략하면 3단계 전부")
    parser.add_argument("--pick", action="store_true",
                        help="인자를 줬어도 나머지를 대화형으로 고른다")
    parser.add_argument("--torpedo", choices=list(torpedo_scaling.TORPEDOES),
                        help="실제 어뢰를 축소해 조건을 유도한다. 속도·발사거리·"
                             "관찰시간이 전부 계산되며 --speed를 대체한다")
    parser.add_argument("--speed-setting",
                        choices=list(torpedo_scaling.SPEED_SETTINGS),
                        help="어뢰 속도 설정 (저속/중속/고속). 낮출수록 최대거리와 "
                             "운행시간이 늘어난다. 생략하면 high(공격)")
    parser.add_argument("--tau", type=float, default=0.30,
                        help="--torpedo의 시간척도 (기본 0.30)")
    parser.add_argument("--launch", type=float, default=150.0,
                        help="--torpedo의 발사거리 m (기본 150). 연료 대비 "
                             "비율은 자동 역산된다")
    parser.add_argument("--search", type=float,
                        help="어뢰가 탐색 단계에서 소모하는 연료 비율. "
                             "생략하면 어뢰별 기본값(경어뢰 0.40 / 중어뢰 0.50)")
    parser.add_argument("--passes", type=int,
                        help="재공격 N회까지만 보고 끊는다. 생략하면 "
                             "연료 소진까지 본다")
    parser.add_argument("--legacy-speed", action="store_true",
                        help="--step/--batch에서 예전 slow/mid/fast를 쓴다. "
                             "생략하면 어뢰 3종(축척 기반)")
    parser.add_argument("--reset-results", action="store_true",
                        help="이전 결과를 보관 폴더로 옮기고 1회부터 새로 시작")
    parser.add_argument("--scale-info", action="store_true",
                        help="축척표만 출력하고 끝낸다")
    parser.add_argument("--batch", action="store_true", help="전체 조합 자동 실행")
    parser.add_argument("--step", action="store_true",
                        help="한 판씩 실행하고 RViz로 보며 판정·메모 (중단/재개 가능)")
    parser.add_argument("--restart", action="store_true",
                        help="--step 기록을 지우고 처음부터")
    parser.add_argument("--summary", action="store_true", help="결과 표 출력")
    parser.add_argument("--coverage", action="store_true",
                        help="48조합 중 무엇을 했고 무엇이 남았는지")
    parser.add_argument("--fire-delay", type=float, default=2.0,
                        help="목표 발행 후 발사까지 대기(초). 길면 ROV가 멀어져 "
                             "어뢰가 불리해진다(3 m/s x 대기초 만큼 앞서감)")
    parser.add_argument("--watch", type=float, default=45.0,
                        help="발사 후 결과 관찰 시간(초)")
    args = parser.parse_args()

    if args.reset_results:
        if not RESULTS.exists():
            log("지울 결과가 없다.")
            return
        # 지우지 않고 옮긴다. 되돌릴 수 있어야 실수해도 복구된다.
        archive = RESULTS.parent / (
            "results_보관_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        RESULTS.rename(archive)
        RESULTS.mkdir(parents=True, exist_ok=True)
        log(f"이전 결과를 옮겼다 → {archive}")
        log("이제 1회부터 새로 쌓인다. 보관본이 필요 없으면 직접 지우면 된다.")
        return

    if args.scale_info:
        for key in torpedo_scaling.TORPEDOES:
            print(torpedo_scaling.describe(torpedo_scaling.scale(
                key, args.tau, launch_sim=args.launch,
                passes_target=args.passes, search_fraction=args.search,
                speed_setting=args.speed_setting or "high")))
            print()
        return

    if args.coverage:
        show_coverage()
        return

    if args.summary:
        show_step_results()
        summarize()
        return

    ENV = ros_env()
    if not MANTA_SH.exists():
        sys.exit(f"manta.sh 없음: {MANTA_SH}")

    stale = check_builds_fresh()
    if stale:
        warn(f"소스가 빌드보다 최신입니다: {', '.join(stale)}")
        warn("옛 바이너리로 실험하면 결과가 무효가 됩니다. 먼저 빌드하세요:")
        warn(f"  cd {WORKSPACE} && colcon build --packages-select {' '.join(stale)}")
        sys.exit(1)

    if args.step:
        run_step_mode(args)
        return

    if args.batch:
        speeds = combo_speeds(args)
        combos = [(s, p, m, v) for s in SCENARIOS for p in PLANNERS
                  for m in TORPEDO_MODES for v in speeds]
        log(f"전체 {len(combos)}회 시작")
        for index, (scenario, planner, mode, speed) in enumerate(combos, 1):
            log(f"[{index}/{len(combos)}]")
            try:
                scaling = scaling_for(speed, args)
                watch = (scaling["judge_time"] if scaling else args.watch)
                run_once(scenario, planner, mode,
                         args.fire_delay, watch, speed=speed, scaling=scaling)
            except Exception as error:  # 한 판이 죽어도 나머지는 계속
                warn(f"실패: {error}")
        summarize()
        return

    # --torpedo를 쓰면 속도는 축척에서 나오므로 묻지 않는다.
    scaling = None
    watch = args.watch
    if args.torpedo:
        scaling = scaling_for(args.torpedo, args)
        log(torpedo_scaling.describe(scaling))
        # 연료가 다할 때까지가 실제 회피 판정 기준이므로 관찰시간을
        # 운행시간에 맞춘다. 사용자가 --watch를 직접 준 경우는 존중한다.
        if args.watch == parser.get_default("watch"):
            watch = scaling["judge_time"]
            log(f"   관찰시간을 판정시간 {watch:.0f}초로 맞춤 "
                f"(총 운행 {scaling['endurance_sim']:.0f}초 중 "
                f"탐색 {scaling['search_fraction']*100:.0f}% 제외)")

    # 인자로 다 주면 그대로 돌리고, 빠진 게 있으면 그것만 물어본다.
    if args.pick or not (args.scenario and args.planner and args.mode
                         and (args.legacy_speed or args.torpedo)):
        scenario, planner, mode, speed, picked = pick_settings(args)
        if picked:
            scaling = picked
            watch = scaling["judge_time"]
    else:
        scenario, planner, mode = args.scenario, args.planner, args.mode
        speed = args.torpedo or args.speed or "fast"

    result = run_once(scenario, planner, mode,
                      args.fire_delay, watch, speed=speed, scaling=scaling)
    if result.get("outcome", "").startswith("FAIL"):
        warn(f"실험 실패: {result['outcome']} — results/ 로그를 확인하세요")
        sys.exit(1)
    summarize()


if __name__ == "__main__":
    main()
