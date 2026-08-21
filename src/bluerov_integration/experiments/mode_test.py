#!/usr/bin/env python3
"""어뢰 유도 모드(2/3) 전환이 실제로 되는지 확인하는 진단 스크립트.

배치 실험 16판이 전부 mode=MANUAL로 돌아 무효였다. 원인이
 (a) 숫자키가 아예 안 먹히는 것인지
 (b) 표적(BlueROV) odometry가 없어 TARGET_WAIT 상태라 유도로 못 넘어가는 것인지
가리기 위해, manta3(브리지)까지 띄워 표적 데이터가 살아있는 상태에서 시험한다.
"""

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
                           HOME / "manta_experiments" / "results")) / "mode_test"


def log(message):
    print(f"\033[1;36m[진단]\033[0m {message}", flush=True)


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
    time.sleep(3)
    for pattern in ["bluerov_integration.launch", "dave_robot.launch",
                    "gz sim", "torpedo_sitl_v2.launch", "topic pub"]:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)


def wait_gz_model(model, limit):
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


def status_line(text):
    """새 CLI 화면에서 MODE/STATE/SENSOR 줄을 뽑는다."""
    clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", text)
    picked = {}
    for line in clean.splitlines():
        for key in ("MODE    :", "STATE   :", "SENSOR  :"):
            if line.strip().startswith(key.strip()):
                picked[key.split()[0]] = line.strip()
    return picked


def brief(picked):
    if not picked:
        return "(상태 없음)"
    return " | ".join(picked.get(k, "") for k in ("MODE", "STATE", "SENSOR")
                      if picked.get(k))


try:
    log("Gazebo + BlueROV 기동...")
    spawn(f"bash '{MANTA_SH}' bluerov 0 0 -1", "gazebo")
    if not wait_gz_model("bluerov2", 200):
        raise SystemExit("Gazebo 기동 실패")
    log("   BlueROV 준비")

    log("어뢰 스폰 (0, 60, -1) yaw=pi ...")
    spawn("ros2 launch dave_demos dave_robot.launch.py gui:=false "
          "x:=0 y:=60 z:=-1 yaw:=3.1416 namespace:=glider_slocum "
          "world_name:=dave_ocean_waves paused:=false use_teleop:=false "
          "open_qgc:=false", "spawn")
    if not wait_gz_model("glider_slocum", 120):
        raise SystemExit("어뢰 스폰 실패")
    log("   어뢰 준비")

    # 핵심: manta3의 ros_gz 브리지가 /model/bluerov2/odometry를 ROS로 내보낸다.
    log("통합 노드(브리지) 기동...")
    spawn(f"bash '{MANTA_SH}' integration", "integration")
    if not wait_ros("node", "/bluerov_integration_node", 180):
        raise SystemExit("통합 노드 실패")
    if not wait_ros("topic", "/model/bluerov2/odometry", 60):
        raise SystemExit("표적 토픽 없음")
    log("   표적 토픽 살아있음")

    log("어뢰 제어를 pty로 기동...")
    child = pexpect.spawn("bash", ["-c", f"bash '{MANTA_SH}' torpedo-control"],
                          env=ENV, encoding="utf-8", timeout=30,
                          dimensions=(40, 200))

    def drain(seconds=3):
        try:
            return child.read_nonblocking(size=400000, timeout=seconds)
        except Exception:
            return ""

    # TARGET_OK가 될 때까지 기다린다 — 배치에서는 6초만 기다려 실패했을 수 있다.
    log("   센서 OK 대기 중...")
    accumulated, target_ok = "", False
    deadline = time.time() + 60
    while time.time() < deadline:
        accumulated += drain(2)
        sensor = status_line(accumulated).get("SENSOR", "")
        if sensor and "FAIL" not in sensor:
            target_ok = True
            break
    log(f"   키 입력 전: {brief(status_line(accumulated))}"
        f"  (센서 OK 도달: {target_ok})")

    child.send("3")
    time.sleep(4)
    after_mode = drain(3)
    log(f"   '3' 입력 후: {brief(status_line(after_mode))}")

    child.send("r")
    time.sleep(4)
    after_fire = drain(3)
    log(f"   'r' 입력 후: {brief(status_line(after_fire))}")

    combined = after_mode + after_fire
    clean = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", combined)
    keys = [l.strip()[:150] for l in clean.splitlines() if "[KEY]" in l]
    print("\n=== 수신된 키 이벤트 ===")
    print("\n".join(keys[-5:]) if keys else "(없음)")

    final_mode = status_line(after_fire).get("MODE", "")
    print("\n=== 판정 ===")
    if final_mode and "[0]" not in final_mode and "NONE" not in final_mode.upper():
        print(f"모드 전환 성공: {final_mode}")
    else:
        print(f"모드 전환 실패: {final_mode or '(상태 없음)'}")

    child.close(force=True)
finally:
    log("정리 중...")
    stop_all()
    log("완료")
