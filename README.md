# MANTA 프로젝트

BlueROV 회피 제어와 위협 어뢰 제어를 DAVE 시뮬레이션에서 시험하는 ROS 2 Jazzy 프로젝트입니다. Gazebo GUI와 조이스틱은 사용하지 않으며, RViz에서 상태를 확인합니다.

## 처음 설치하기

### 준비 사항

설치할 컴퓨터에 다음 항목이 필요합니다.

- Ubuntu 24.04 LTS
- 인터넷 연결
- `sudo` 권한
- 충분한 저장 공간

최초 설치는 시스템 전체 업그레이드, ROS 2 Jazzy, Gazebo Harmonic 및 ArduSub 설치와 빌드를 포함하므로 시간이 오래 걸릴 수 있습니다.

### 한 줄 자동 설치—권장

터미널에서 다음 한 줄을 일반 사용자로 실행합니다.

```bash
sudo apt update && sudo apt install -y curl && /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/MANTA-P/MANTA/main/manta.sh)"
```

> 명령 전체 앞에 `sudo`를 붙이지 마세요. 스크립트가 시스템 변경이 필요한 단계에서만 `sudo` 암호를 요청합니다.

원격으로 받은 `manta.sh`는 다음 작업을 순서대로 수행합니다.

1. Ubuntu 24.04 및 일반 사용자 실행 여부 확인
2. Git 설치 여부 확인
3. MANTA 저장소를 `~/manta_ws`에 clone
4. ROS 2 Jazzy, Gazebo Harmonic 및 ArduSub 설치
5. `rosdep`으로 MANTA 의존성 설치
6. `colcon build --symlink-install`로 `~/manta_ws/src` 전체 빌드
7. `~/.bashrc`에 ROS 환경과 MANTA 명령어 등록

시스템 설치 여부를 묻는 메시지가 나오면 내용을 확인한 뒤 `y`를 입력합니다. 설치 중 오류가 발생하면 마지막 오류를 해결한 뒤 같은 한 줄 명령을 다시 실행하면 됩니다.

`~/manta_ws`에 정상적인 MANTA 저장소가 이미 있으면 기존 파일과 브랜치를 변경하지 않고 해당 작업공간을 사용합니다. 다른 파일이 들어 있는 폴더라면 내용을 덮어쓰지 않고 오류와 함께 중단합니다.

### 수동 설치—대체 방법

한 줄 설치를 사용하지 않으려면 저장소를 직접 받은 뒤 로컬 스크립트를 실행합니다.

```bash
sudo apt update
sudo apt install -y git

git clone https://github.com/MANTA-P/MANTA.git ~/manta_ws
cd ~/manta_ws
bash manta.sh
```

`~/manta_ws`가 이미 존재하고 비어 있지 않다면 삭제하거나 강제로 초기화하지 말고 기존 작업 내용을 먼저 확인하세요.

### 개발 브랜치 설치 시험

`dev` 브랜치의 스크립트를 시험할 때는 내려받는 스크립트와 clone할 브랜치를 모두 `dev`로 지정합니다.

```bash
MANTA_REF=dev /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/MANTA-P/MANTA/dev/manta.sh)"
```

일반 사용자용 README와 정식 배포에서는 `main` 명령을 사용합니다.

### 설치된 명령어 적용

설치가 끝나면 새 터미널을 열거나 다음 명령을 실행합니다.

```bash
source ~/.bashrc
```

등록된 명령어는 다음과 같이 확인할 수 있습니다.

```bash
manta_help
```

## 실행하기

아래 1~5는 동시에 계속 실행되어야 하므로 **각각 별도 터미널**에서 순서대로 실행합니다. 각 새 터미널에는 설치 과정에서 ROS와 MANTA 환경이 자동으로 적용됩니다.

| 순서 | 간단한 명령 | 설명형 명령 | 실제 스크립트 명령 | 기능 |
|---:|---|---|---|---|
| 1 | `manta1` | `manta_bluerov` | `manta bluerov` | BlueROV와 Gazebo 실행 |
| 2 | `manta2` | `manta_torpedo_spawn` | `manta torpedo-spawn` | 위협 어뢰 생성 |
| 3 | `manta3` | `manta_integration` | `manta integration` | 통합 패키지와 RViz 실행 |
| 4 | `manta4` | `manta_target` | `manta target` | BlueROV 목표 위치 입력 |
| 5 | `manta5` | `manta_torpedo_control` | `manta torpedo-control` | 위협 어뢰 상태 및 제어 실행 |

숫자 명령, 설명형 명령 및 `manta` 하위 명령은 서로 동일합니다. 모든 alias는 저장소 루트의 `manta.sh`를 실행하므로 각 명령이 자체적으로 ROS 환경을 확인하고 불러옵니다.

### 1. BlueROV와 Gazebo 실행

첫 번째 터미널에서 실행합니다.

```bash
manta1
```

BlueROV와 `dave_ocean_waves` Gazebo 환경을 GUI 없이 실행합니다. Gazebo가 완전히 시작될 때까지 기다린 뒤 2번을 실행하세요.

기본 깊이 `-0.5` 대신 다른 초기 Z 좌표만 지정하려면 숫자 하나를 추가합니다.

```bash
manta1 -1.0
```

초기 `X Y Z` 좌표를 모두 지정하려면 숫자 3개를 순서대로 추가합니다.

```bash
manta1 5 3 -1.0
```

정리하면 `manta1`은 인자 0개, 1개 또는 3개를 받습니다.

```text
manta1              기존 기본 X/Y, Z=-0.5
manta1 Z            Z만 지정
manta1 X Y Z        X/Y/Z 모두 지정
```

### 2. 위협 어뢰 생성

두 번째 터미널을 열어 실행합니다.

```bash
manta2
```

위협 어뢰인 `glider_slocum`을 초기 위치 `(-30, -30, -5)`에 생성합니다.

다른 초기 위치를 사용하려면 `X Y Z` 좌표 3개를 추가합니다.

```bash
manta2 -40 -10 -8
```

### 3. BlueROV 통합 패키지 실행

세 번째 터미널을 열어 실행합니다.

```bash
manta3
```

BlueROV 브리지, DataHub, A* 경로 계획, 경로 추종, PPID, CSV 기록 및 RViz를 함께 실행합니다.

### 4. BlueROV 목표 위치 입력

네 번째 터미널을 열어 실행합니다.

```bash
manta4
```

프롬프트가 나타나면 `map` 좌표계 기준 절대좌표 `x y z`를 공백으로 구분해 입력합니다.

```text
target> 10 0 -0.5
```

첫 번째 목표 좌표를 명령에 직접 전달할 수도 있습니다. 해당 좌표가 자동으로 입력된 뒤에는 같은 터미널에서 다음 목표를 계속 입력할 수 있습니다.

```bash
manta4 10 0 -0.5
```

### 5. 위협 어뢰 상태 및 제어 실행

다섯 번째 터미널을 열어 실행합니다.

```bash
manta5
```

어뢰의 Odometry와 JointState를 ROS 2로 전달하고 어뢰 조종 또는 유도 기능을 실행합니다. 통합 패키지는 어뢰 위치를 받은 뒤 A* 경로 계산과 회피 제어를 시작합니다.

정상 상태에서는 3번 통합 노드 터미널에 다음과 같은 로그가 표시됩니다.

```text
torpedo=1 mission=1 path=1
```

## 종료하기

각 터미널에서 `Ctrl+C`를 눌러 실행 중인 프로세스를 종료합니다. 일반적으로 실행 순서의 역순인 5번부터 1번까지 종료하면 됩니다.

## 코드 업데이트 및 재빌드

원격 저장소의 최신 코드를 받은 뒤 등록된 재빌드 명령을 실행합니다.

```bash
cd ~/manta_ws
git pull --ff-only origin main
manta_rebuild
```

로컬 수정 때문에 `git pull`이 실패하면 `git reset --hard` 같은 명령으로 강제 초기화하지 마세요. 수정 내용을 먼저 확인하고 팀에 공유하세요.

## 자주 발생하는 문제

### `manta1: command not found`

새 터미널을 열거나 환경 설정을 다시 적용합니다.

```bash
source ~/.bashrc
manta_help
```

### ROS 패키지를 찾을 수 없음

전체 워크스페이스를 다시 빌드합니다.

```bash
manta_rebuild
```

재빌드도 실패하면 오류가 발생한 패키지 이름과 터미널의 마지막 오류 내용을 확인합니다.

### 시스템 설치를 이미 완료한 경우

스크립트가 ROS 2 Jazzy, Gazebo 및 ArduSub 설치를 감지하면 시스템 설치를 자동으로 생략합니다. 시스템 설치를 의도적으로 건너뛰려면 다음 옵션을 사용할 수 있습니다.

```bash
bash manta.sh --skip-system
```

이 옵션은 필요한 시스템 의존성이 이미 설치된 것이 확실할 때만 사용하세요.

## 저장소 구조

GitHub 저장소에는 `manta_ws` 폴더 자체가 아니라 그 안의 파일을 올립니다.

```text
GitHub 저장소 루트/
├── README.md
├── manta.sh
└── src/
    ├── bluerov_integration/
    ├── dave/
    ├── torpedo_control/
    └── torpedo_control_v2/
```

`manta.sh`는 원격 bootstrap, 로컬 설치 프로그램 및 모든 MANTA 실행 명령을 함께 제공합니다. 원격으로 실행하면 저장소를 받은 뒤 clone된 로컬 `manta.sh`로 전환합니다. 설치 시 `~/.bashrc`에 등록되는 alias도 이 로컬 스크립트의 하위 명령을 호출합니다. 따라서 파일 이름이나 위치를 변경하면 alias를 다시 등록해야 합니다.

## 최신 변경 사항
- feat: 어뢰 유도·제어 구조 개편 및 시뮬레이션 센서 발행 최적화 ([#9](https://github.com/MANTA-P/MANTA/pull/9))
