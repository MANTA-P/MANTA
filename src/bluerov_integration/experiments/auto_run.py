#!/usr/bin/env python3
"""회피 실험 자동 실행기.

어뢰 제어 노드가 /dev/tty에서 키를 읽으므로, pexpect로 가상 터미널(pty)에
띄워서 모드 선택(2/3)과 발사(r)를 프로그램으로 넣는다. 사람 손 없이
A*/DVO를 같은 조건으로 비교할 수 있다.

  ./auto_run.py rear astar 3          # 한 판
  ./auto_run.py --batch               # 전체 조합
  ./auto_run.py --summary             # 결과 표

manta.sh(팀 공용)는 수정하지 않고 호출만 한다.
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import pexpect

HOME = Path.home()
MANTA_SH = Path(os.environ.get("MANTA_SH", HOME / "manta_ws" / "manta.sh"))
WORKSPACE = Path(os.environ.get("WORKSPACE", HOME / "manta_ws"))
# 로그는 저장소 밖(홈)에 쌓는다. 스크립트를 git에 올려도 실험 로그가
# 딸려가지 않고, 팀원끼리 로그가 섞이지도 않는다.
RESULTS = Path(os.environ.get("RESULTS_DIR",
                              HOME / "manta_experiments" / "results"))
NODE = "/bluerov_integration_node"

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
PLANNERS = ["astar", "dvo"]
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
             reuse_gazebo=False, speed="fast"):
    """실험 한 판. 결과 dict를 돌려준다."""
    (tx, ty, tz, tyaw), (gx, gy, gz), description = SCENARIOS[scenario]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_id = f"{scenario}_{planner}_mode{mode}_{speed}_{stamp}"
    RESULTS.mkdir(exist_ok=True)
    planner_log = RESULTS / f"{run_id}_planner.log"
    torpedo_log = RESULTS / f"{run_id}_torpedo.log"

    log(f"── {scenario}/{planner}/모드{mode}({TORPEDO_MODES[mode]})"
        f"/{speed}({SPEED_TABLE[speed]}) — {description}")
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
            for _ in range(THRUST_LEVELS[speed]):
                child.send("r")
                time.sleep(0.25)
            thrust_line = ""
            clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", screen())
            for line in clean.splitlines():
                if line.strip().startswith("THRUST"):
                    thrust_line = line.strip()
            log(f"   발사 (모드 {mode} = {TORPEDO_MODES[mode]}, {thrust_line})")

            # 결과가 로그에 찍힐 때까지 지켜본다.
            outcome, distance = "TIMEOUT", None
            deadline = time.time() + watch
            while time.time() < deadline:
                # 관찰 중에도 pty를 비운다. 버퍼가 차면 어뢰 노드가 화면
                # 출력에서 막혀 제어를 멈춘다.
                screen()
                if not planner_log.exists():
                    continue
                text = planner_log.read_text(errors="ignore")
                hit = re.search(r"\[HIT\].*?closest ([0-9.]+) m", text)
                avoided = re.search(r"\[AVOIDED\].*?min distance ([0-9.]+) m", text)
                if hit:
                    outcome, distance = "HIT", float(hit.group(1))
                    break
                if avoided:
                    outcome, distance = "AVOIDED", float(avoided.group(1))
                    break
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
                "mode": mode, "outcome": outcome, "distance": distance,
                "plans": plan_count, "log": planner_log}
    finally:
        for process in reversed(processes):
            stop(process)
        cleanup(kill_gazebo=not reuse_gazebo)


def summarize():
    logs = sorted(RESULTS.glob("*_planner.log"))
    if not logs:
        warn(f"결과 로그 없음: {RESULTS}")
        return
    header = ("시나리오", "플래너", "모드", "속도", "결과", "최근접m",
              "평균계획ms", "최대box", "VO실패")
    print(f"\n{header[0]:<8}{header[1]:<7}{header[2]:<14}{header[3]:<6}"
          f"{header[4]:<9}{header[5]:<9}{header[6]:<11}{header[7]:<9}{header[8]}")
    print("─" * 88)
    for path in logs:
        name = path.name.replace("_planner.log", "")
        parts = name.split("_")
        scenario, planner = parts[0], parts[1]
        mode = parts[2].replace("mode", "") if len(parts) > 2 else "?"
        speed = parts[3] if len(parts) > 4 and parts[3] in THRUST_LEVELS else "-"
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
        print(f"{scenario:<8}{planner:<7}{mode_label:<14}{speed:<6}{outcome:<9}"
              f"{distance:<9}"
              f"{(f'{sum(times)/len(times):.2f}' if times else '-'):<11}"
              f"{(str(max(boxes)) if boxes else '-'):<9}{vo_fail}")
    print(f"\n로그: {RESULTS}\n")


def main():
    global ENV
    parser = argparse.ArgumentParser(description="회피 실험 자동 실행")
    parser.add_argument("scenario", nargs="?", choices=list(SCENARIOS))
    parser.add_argument("planner", nargs="?", choices=PLANNERS)
    parser.add_argument("mode", nargs="?", type=int, choices=[2, 3])
    parser.add_argument("--speed", choices=list(THRUST_LEVELS),
                        help="어뢰 속도 단계 (slow 6.5 / mid 8.5 / fast 12.2 m/s). "
                             "단일 실행 기본 fast, --batch에서 생략하면 3단계 전부")
    parser.add_argument("--batch", action="store_true", help="전체 조합 실행")
    parser.add_argument("--summary", action="store_true", help="결과 표 출력")
    parser.add_argument("--fire-delay", type=float, default=2.0,
                        help="목표 발행 후 발사까지 대기(초). 길면 ROV가 멀어져 "
                             "어뢰가 불리해진다(3 m/s x 대기초 만큼 앞서감)")
    parser.add_argument("--watch", type=float, default=45.0,
                        help="발사 후 결과 관찰 시간(초)")
    args = parser.parse_args()

    if args.summary:
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

    if args.batch:
        speeds = [args.speed] if args.speed else list(THRUST_LEVELS)
        combos = [(s, p, m, v) for s in SCENARIOS for p in PLANNERS
                  for m in TORPEDO_MODES for v in speeds]
        log(f"전체 {len(combos)}회 시작 (1회당 약 2분)")
        for index, (scenario, planner, mode, speed) in enumerate(combos, 1):
            log(f"[{index}/{len(combos)}]")
            try:
                run_once(scenario, planner, mode,
                         args.fire_delay, args.watch, speed=speed)
            except Exception as error:  # 한 판이 죽어도 나머지는 계속
                warn(f"실패: {error}")
        summarize()
        return

    if not (args.scenario and args.planner and args.mode):
        parser.error("시나리오·플래너·모드를 지정하거나 --batch / --summary")
    result = run_once(args.scenario, args.planner, args.mode,
                      args.fire_delay, args.watch,
                      speed=args.speed or "fast")
    if result.get("outcome", "").startswith("FAIL"):
        warn(f"실험 실패: {result['outcome']} — results/ 로그를 확인하세요")
        sys.exit(1)
    summarize()


if __name__ == "__main__":
    main()
