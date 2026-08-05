#!/usr/bin/env bash

# MANTA installation and command runner.
# Run `bash manta.sh help` to see all commands.

set -Eeuo pipefail

readonly MANTA_ROS_SETUP="/opt/ros/jazzy/setup.bash"
readonly MANTA_DAVE_INSTALLER_URL="https://raw.githubusercontent.com/IOES-Lab/dave/refs/heads/ros2/extras/ros-jazzy-gz-harmonic-install.sh"
MANTA_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/$(basename -- "${BASH_SOURCE[0]}")"
MANTA_WS="$(dirname -- "$MANTA_SCRIPT")"
export MANTA_WS

fail() {
  printf '오류: %s\n' "$*" >&2
  exit 1
}

source_setup_file() {
  local setup_file="$1"
  local restore_nounset=0

  case "$-" in
    *u*) restore_nounset=1 ;;
  esac

  # Some ROS-generated setup files reference variables before assigning them.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  if ((restore_nounset)); then
    set -u
  fi
}

require_workspace() {
  [[ -d "$MANTA_WS/src" ]] || fail "저장소의 src 디렉터리를 찾을 수 없습니다: $MANTA_WS/src"
}

load_ros_environment() {
  [[ -f "$MANTA_ROS_SETUP" ]] || \
    fail "ROS 2 Jazzy 환경을 찾을 수 없습니다. 먼저 'bash manta.sh install'을 실행하세요."
  [[ -f "$MANTA_WS/install/setup.bash" ]] || \
    fail "MANTA 빌드 환경을 찾을 수 없습니다. 먼저 'bash manta.sh rebuild'를 실행하세요."

  source_setup_file "$MANTA_ROS_SETUP"
  source_setup_file "$MANTA_WS/install/setup.bash"
}

is_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

require_number() {
  local name="$1"
  local value="$2"
  is_number "$value" || fail "$name 값은 숫자여야 합니다: $value"
}

show_help() {
  cat <<'EOF'
MANTA 설치 및 실행 도구

사용법:
  bash manta.sh [명령] [인자]

  명령을 생략하면 install을 기본으로 실행합니다.

설치와 관리:
  install [--yes] [--skip-system]  시스템 의존성 설치, rosdep, 빌드, alias 등록
  rebuild                         MANTA 전체 워크스페이스 재빌드
  help                            이 도움말 표시

시뮬레이션 실행:
  bluerov [Z | X Y Z]
      BlueROV와 Gazebo 실행
      인자가 없으면 기존 기본 위치와 Z=-0.5 사용
      숫자 1개는 Z, 숫자 3개는 X Y Z로 사용

  torpedo-spawn [X Y Z]
      위협 어뢰 생성. 기본 위치: -30 -30 -5

  integration
      BlueROV 브리지, 경로 계획, 제어, 기록 및 RViz 실행

  target [X Y Z]
      인자가 없으면 목표 좌표를 대화형으로 입력
      좌표 3개를 주면 첫 목표로 자동 입력한 뒤 대화형 입력 유지

  torpedo-control
      위협 어뢰 상태 전달 및 조종/유도 실행

설치 후 사용할 수 있는 alias:
  manta1 / manta_bluerov
  manta2 / manta_torpedo_spawn
  manta3 / manta_integration
  manta4 / manta_target
  manta5 / manta_torpedo_control
  manta_rebuild
  manta_help

예:
  manta1
  manta1 -1.0
  manta1 5 3 -1.0
  manta2 -40 -10 -8
  manta4
  manta4 10 0 -0.5
EOF
}

register_bash_environment() {
  local bashrc="$HOME/.bashrc"
  local start_marker="# >>> MANTA project >>>"
  local end_marker="# <<< MANTA project <<<"

  touch "$bashrc"
  if grep -Fqx "$start_marker" "$bashrc" 2>/dev/null; then
    printf '  기존 MANTA 설정 블록을 유지합니다: %s\n' "$bashrc"
    return 0
  fi

  {
    printf '\n%s\n' "$start_marker"
    printf 'if [ -f %q ]; then\n' "$MANTA_ROS_SETUP"
    printf '  source %q\n' "$MANTA_ROS_SETUP"
    printf 'fi\n'
    printf 'if [ -f %q ]; then\n' "$MANTA_WS/install/setup.bash"
    printf '  source %q\n' "$MANTA_WS/install/setup.bash"
    printf 'fi\n'
    printf 'alias manta=%q\n' "bash $MANTA_SCRIPT"
    printf 'alias manta1=%q\n' "bash $MANTA_SCRIPT bluerov"
    printf 'alias manta2=%q\n' "bash $MANTA_SCRIPT torpedo-spawn"
    printf 'alias manta3=%q\n' "bash $MANTA_SCRIPT integration"
    printf 'alias manta4=%q\n' "bash $MANTA_SCRIPT target"
    printf 'alias manta5=%q\n' "bash $MANTA_SCRIPT torpedo-control"
    printf 'alias manta_bluerov=%q\n' "bash $MANTA_SCRIPT bluerov"
    printf 'alias manta_torpedo_spawn=%q\n' "bash $MANTA_SCRIPT torpedo-spawn"
    printf 'alias manta_integration=%q\n' "bash $MANTA_SCRIPT integration"
    printf 'alias manta_target=%q\n' "bash $MANTA_SCRIPT target"
    printf 'alias manta_torpedo_control=%q\n' "bash $MANTA_SCRIPT torpedo-control"
    printf 'alias manta_rebuild=%q\n' "bash $MANTA_SCRIPT rebuild"
    printf 'alias manta_help=%q\n' "bash $MANTA_SCRIPT help"
    printf '%s\n' "$end_marker"
  } >> "$bashrc"

  printf '  MANTA 환경과 alias를 추가했습니다: %s\n' "$bashrc"
}

command_install() {
  local assume_yes=0
  local skip_system_install=0
  local system_ready=1
  local answer
  local installer_file=""

  while (($#)); do
    case "$1" in
      -y|--yes)
        assume_yes=1
        ;;
      --skip-system)
        skip_system_install=1
        ;;
      -h|--help)
        cat <<'EOF'
사용법: bash manta.sh install [옵션]

  -y, --yes       시스템 설치 확인 질문에 자동으로 동의
  --skip-system   DAVE/ROS/Gazebo 시스템 설치 단계 생략
EOF
        return 0
        ;;
      *)
        fail "install의 알 수 없는 옵션: $1"
        ;;
    esac
    shift
  done

  if ((EUID == 0)); then
    fail "manta.sh 전체를 sudo로 실행하지 마세요. 일반 사용자로 실행하면 필요한 단계에서만 sudo를 요청합니다."
  fi

  [[ -r /etc/os-release ]] || fail "/etc/os-release를 읽을 수 없습니다. Ubuntu 24.04에서 실행하세요."
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]] || \
    fail "지원 환경은 Ubuntu 24.04 LTS입니다. 현재 환경: ${PRETTY_NAME:-알 수 없음}"

  require_workspace

  printf '\n[1/5] MANTA 작업공간 확인\n'
  printf '  작업공간: %s\n' "$MANTA_WS"

  [[ -f "$MANTA_ROS_SETUP" ]] || system_ready=0
  command -v gz >/dev/null 2>&1 || system_ready=0
  [[ -d /opt/ardusub_ws/ardupilot ]] || system_ready=0

  if ((skip_system_install)); then
    printf '\n[2/5] 요청에 따라 DAVE/ROS/Gazebo 시스템 설치를 생략합니다.\n'
  elif ((system_ready)); then
    printf '\n[2/5] ROS 2 Jazzy, Gazebo, ArduSub 설치가 감지되어 시스템 설치를 생략합니다.\n'
  else
    printf '\n[2/5] DAVE/ROS/Gazebo 시스템 설치가 필요합니다.\n'
    printf '%s\n' '  이 단계는 apt 전체 업그레이드와 ROS 2, Gazebo, ArduSub 설치를 수행합니다.'
    printf '%s\n' '  인터넷 연결과 sudo 권한이 필요하며 시간이 오래 걸릴 수 있습니다.'

    if ((!assume_yes)); then
      read -r -p "계속 설치할까요? [y/N] " answer
      case "$answer" in
        y|Y|yes|YES) ;;
        *) fail "사용자가 시스템 설치를 취소했습니다." ;;
      esac
    fi

    if ! command -v wget >/dev/null 2>&1; then
      printf 'wget을 설치합니다.\n'
      sudo apt-get update
      sudo apt-get install -y wget
    fi

    installer_file="$(mktemp --suffix=.sh)"
    trap 'rm -f -- "$installer_file"' EXIT

    printf '공식 DAVE 설치 스크립트를 내려받습니다.\n'
    wget -qO "$installer_file" "$MANTA_DAVE_INSTALLER_URL"
    [[ -s "$installer_file" ]] || fail "DAVE 설치 스크립트 다운로드에 실패했습니다."
    sudo /bin/bash "$installer_file"

    rm -f -- "$installer_file"
    trap - EXIT
  fi

  [[ -f "$MANTA_ROS_SETUP" ]] || fail "ROS 2 Jazzy 설치 파일을 찾을 수 없습니다: $MANTA_ROS_SETUP"
  source_setup_file "$MANTA_ROS_SETUP"

  command -v rosdep >/dev/null 2>&1 || fail "rosdep 명령을 찾을 수 없습니다. 시스템 설치 로그를 확인하세요."
  command -v colcon >/dev/null 2>&1 || fail "colcon 명령을 찾을 수 없습니다. 시스템 설치 로그를 확인하세요."

  printf '\n[3/5] rosdep 의존성 설치\n'
  if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
  rosdep install --from-paths "$MANTA_WS/src" --ignore-src -r -y --rosdistro jazzy

  printf '\n[4/5] MANTA 전체 빌드\n'
  (
    cd "$MANTA_WS"
    colcon build --symlink-install
  )
  [[ -f "$MANTA_WS/install/setup.bash" ]] || fail "빌드 후 install/setup.bash가 생성되지 않았습니다."

  printf '\n[5/5] Bash 환경과 MANTA alias 등록\n'
  register_bash_environment

  printf '\n============================================================\n'
  printf 'MANTA 설치와 빌드가 완료되었습니다.\n'
  printf '새 터미널을 열거나 다음 명령을 실행하세요:\n\n'
  printf '  source ~/.bashrc\n'
  printf '  manta_help\n\n'
  printf '그다음 별도 터미널에서 manta1부터 manta5까지 순서대로 실행하세요.\n'
  printf '============================================================\n'
}

command_rebuild() {
  require_workspace
  [[ -f "$MANTA_ROS_SETUP" ]] || fail "ROS 2 Jazzy 환경을 찾을 수 없습니다: $MANTA_ROS_SETUP"

  source_setup_file "$MANTA_ROS_SETUP"
  (
    cd "$MANTA_WS"
    colcon build --symlink-install
  )
  [[ -f "$MANTA_WS/install/setup.bash" ]] || fail "빌드 후 install/setup.bash가 생성되지 않았습니다."
  printf 'MANTA 재빌드가 완료되었습니다.\n'
}

command_bluerov() {
  local x=""
  local y=""
  local z="-0.5"
  local -a position_arguments=()

  case "$#" in
    0) ;;
    1)
      z="$1"
      ;;
    3)
      x="$1"
      y="$2"
      z="$3"
      require_number "X" "$x"
      require_number "Y" "$y"
      position_arguments=("x:=$x" "y:=$y")
      ;;
    *) fail "사용법: manta1 [Z | X Y Z]" ;;
  esac
  require_number "Z" "$z"

  load_ros_environment
  exec ros2 launch dave_demos dave_robot.launch.py \
    headless:=true \
    "${position_arguments[@]}" \
    z:="$z" \
    namespace:=bluerov2 \
    world_name:=dave_ocean_waves \
    paused:=false \
    open_virtual_joystick:=false \
    open_qgc:=false
}

command_torpedo_spawn() {
  local x="-30"
  local y="-30"
  local z="-5"

  case "$#" in
    0) ;;
    3)
      x="$1"
      y="$2"
      z="$3"
      ;;
    *) fail "사용법: manta2 [X Y Z]" ;;
  esac
  require_number "X" "$x"
  require_number "Y" "$y"
  require_number "Z" "$z"

  load_ros_environment
  exec ros2 launch dave_demos dave_robot.launch.py \
    gui:=false \
    x:="$x" \
    y:="$y" \
    z:="$z" \
    namespace:=glider_slocum \
    world_name:=dave_ocean_waves \
    paused:=false \
    use_teleop:=false \
    open_qgc:=false
}

command_integration() {
  (($# == 0)) || fail "사용법: manta3"
  load_ros_environment
  exec ros2 launch bluerov_integration bluerov_integration.launch.py
}

command_target() {
  local x
  local y
  local z

  case "$#" in
    0)
      load_ros_environment
      printf 'map 기준 목표 절대좌표를 x y z 순서로 입력하세요. 예: 10 0 -0.5\n'
      exec ros2 run bluerov_integration target_position_input_node
      ;;
    3)
      x="$1"
      y="$2"
      z="$3"
      require_number "X" "$x"
      require_number "Y" "$y"
      require_number "Z" "$z"
      load_ros_environment
      printf '첫 목표 좌표를 자동 입력합니다: %s %s %s\n' "$x" "$y" "$z"
      { printf '%s %s %s\n' "$x" "$y" "$z"; cat; } | \
        ros2 run bluerov_integration target_position_input_node
      ;;
    *)
      fail "사용법: manta4 [X Y Z]"
      ;;
  esac
}

command_torpedo_control() {
  (($# == 0)) || fail "사용법: manta5"
  load_ros_environment
  exec ros2 launch torpedo_control_v2 torpedo_sitl_v2.launch.py
}

main() {
  local command

  case "${1:-}" in
    "")
      command="install"
      ;;
    -y|--yes|--skip-system)
      command="install"
      ;;
    *)
      command="$1"
      shift
      ;;
  esac

  case "$command" in
    install) command_install "$@" ;;
    rebuild) command_rebuild "$@" ;;
    bluerov) command_bluerov "$@" ;;
    torpedo-spawn) command_torpedo_spawn "$@" ;;
    integration) command_integration "$@" ;;
    target) command_target "$@" ;;
    torpedo-control) command_torpedo_control "$@" ;;
    help|-h|--help) show_help ;;
    *)
      show_help >&2
      fail "알 수 없는 명령: $command"
      ;;
  esac
}

main "$@"
