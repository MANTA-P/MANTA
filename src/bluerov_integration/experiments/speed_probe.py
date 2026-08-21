#!/usr/bin/env python3
"""추력 단계별 어뢰 실제 속도를 잰다.

'r' 누르는 횟수(추력)와 실제 m/s의 관계를 측정해, 실제 어뢰 속도에 맞는
단계를 고를 수 있게 한다. 어뢰 코드는 수정하지 않고 키 입력만 흉내낸다.
"""

import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import pexpect

HOME = Path.home()
MANTA_SH = HOME / "manta_ws" / "manta.sh"
WORKSPACE = HOME / "manta_ws"
LOGS = Path(os.environ.get("RESULTS_DIR",
                           HOME / "manta_experiments" / "results")) / "speed_probe"

PRESS_LEVELS = [3, 5, 7, 10, 12]   # 추력 300/500/700/1000/1000(상한)


def log(message):
    print(f"\033[1;36m[속도측정]\033[0m {message}", flush=True)


def ros_env():
    command = (f"source /opt/ros/jazzy/setup.bash && "
               f"source {WORKSPACE}/install/setup.bash && env")
    output = subprocess.run(["bash", "-c", command],
                            capture_output=True, text=True, check=True).stdout
    return dict(line.split("=", 1) for line in output.splitlines() if "=" in line)


ENV = ros_env()
LOGS.mkdir(parents=True, exist_ok=True)
processes = []


def spawn(command, name):
    handle = open(LOGS / f"{name}.log", "w")
    process = subprocess.Popen(["bash", "-c", command], env=ENV,
                               stdout=handle, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid)
    processes.append(process)
    return process


def stop_all():
    for process in reversed(processes):
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except Exception:
            pass
    time.sleep(2)
    for pattern in ["bluerov_integration.launch", "dave_robot.launch",
                    "gz sim", "torpedo_sitl_v2.launch", "topic pub"]:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)


def wait_gz(model, limit):
    deadline = time.time() + limit
    while time.time() < deadline:
        result = subprocess.run(["gz", "topic", "-l"], env=ENV,
                                capture_output=True, text=True, timeout=15)
        if f"/model/{model}/" in result.stdout:
            return True
        time.sleep(3)
    return False


def wait_ros(kind, name, limit):
    deadline = time.time() + limit
    while time.time() < deadline:
        result = subprocess.run(["ros2", kind, "list"], env=ENV,
                                capture_output=True, text=True, timeout=20)
        if name in result.stdout.split():
            return True
        time.sleep(2)
    return False


def torpedo_position():
    """어뢰 위치를 한 번 읽는다."""
    result = subprocess.run(
        ["ros2", "topic", "echo", "/torpedo/state/odometry",
         "--field", "pose.pose.position", "--once"],
        env=ENV, capture_output=True, text=True, timeout=20)
    values = {}
    for line in result.stdout.splitlines():
        match = re.match(r"\s*([xyz]):\s*(-?[0-9.eE+]+)", line)
        if match:
            values[match.group(1)] = float(match.group(2))
    return values if len(values) == 3 else None


try:
    log("Gazebo + BlueROV...")
    spawn(f"bash '{MANTA_SH}' bluerov 0 0 -1", "gazebo")
    if not wait_gz("bluerov2", 200):
        raise SystemExit("Gazebo 실패")

    # 어뢰를 멀리 두고 직선으로 달리게 한다(벽·표적 간섭 없이 속도만 측정).
    log("어뢰 스폰 (0, 200, -5) yaw=pi ...")
    spawn("ros2 launch dave_demos dave_robot.launch.py gui:=false "
          "x:=0 y:=200 z:=-5 yaw:=3.1416 namespace:=glider_slocum "
          "world_name:=dave_ocean_waves paused:=false use_teleop:=false "
          "open_qgc:=false", "spawn")
    if not wait_gz("glider_slocum", 120):
        raise SystemExit("어뢰 스폰 실패")

    log("브리지(통합 노드)...")
    spawn(f"bash '{MANTA_SH}' integration", "integration")
    if not wait_ros("topic", "/torpedo/state/odometry", 180):
        raise SystemExit("어뢰 odometry 없음")

    log("어뢰 제어(pty)...")
    child = pexpect.spawn("bash", ["-c", f"bash '{MANTA_SH}' torpedo-control"],
                          env=ENV, encoding="utf-8", timeout=30,
                          dimensions=(45, 200))

    def screen(seconds=2):
        try:
            return child.read_nonblocking(size=400000, timeout=seconds)
        except Exception:
            return ""

    # 센서 OK 대기
    deadline = time.time() + 45
    buffer = ""
    while time.time() < deadline:
        buffer += screen()
        clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", buffer)
        sensor = [l for l in clean.splitlines() if l.strip().startswith("SENSOR")]
        if sensor and "FAIL" not in sensor[-1]:
            break

    # 모드 1(Keyboard) = 유도 없이 직진. 순수 추력-속도 관계만 본다.
    for _ in range(5):
        child.send("1")
        time.sleep(1)
        clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", screen())
        modes = [l.strip() for l in clean.splitlines()
                 if l.strip().startswith("MODE    :")]
        if modes and "[1]" in modes[-1]:
            break
    log("   모드 1(Keyboard) 설정 — 직진 상태로 측정")

    print(f"\n{'r 횟수':<8}{'추력':<8}{'속도(m/s)':<12}{'속도(kt)':<10}")
    print("─" * 40)

    pressed = 0
    for target in PRESS_LEVELS:
        for _ in range(target - pressed):
            child.send("r")
            time.sleep(0.25)
        pressed = target
        screen()

        time.sleep(6)                    # 가속이 붙을 시간
        first = torpedo_position()
        t0 = time.time()
        time.sleep(5)
        second = torpedo_position()
        elapsed = time.time() - t0
        screen()

        if not first or not second:
            print(f"{target:<8}{'?':<8}{'(위치 읽기 실패)':<12}")
            continue
        distance = math.dist((first["x"], first["y"], first["z"]),
                             (second["x"], second["y"], second["z"]))
        speed = distance / elapsed
        thrust = min(target * 100, 1000)
        print(f"{target:<8}{thrust:<8}{speed:<12.2f}{speed * 1.944:<10.1f}")

    child.close(force=True)
finally:
    log("정리 중...")
    stop_all()
    log("완료")
