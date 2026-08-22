#!/usr/bin/env python3
"""실제 어뢰 제원을 축소해 시뮬 실험 조건을 유도한다.

속도·거리·시간을 따로 고르면 "왜 그 값이냐"에 답할 수 없다. 그래서 척도를
먼저 정의하고 나머지를 전부 여기서 유도한다.

  기준 앵커 : 잠수함 순항 20 kt  ↔  우리 ROV 3.0 m/s
  속도척도 σ = 3.0 / (20 x 0.5144) = 0.2916
  시간척도 τ = 선택값(실험 시간 제약에서 정한다)
  거리척도 λ = σ x τ

이 앵커 하나로 어뢰 속도가 정해지고, 최대거리에서 운행시간이 나오고,
운행시간에서 발사거리와 관찰시간이 나온다.

단독 실행하면 축척표를 출력한다:
    ./torpedo_scaling.py
    ./torpedo_scaling.py --tau 0.4 --sub-speed 15
"""

import argparse
import math

KNOT = 0.5144            # m/s
ROV_SPEED = 3.0          # 우리 BlueROV2 순항속도 (m/s)

# ── 실제 어뢰 제원 (공개 자료) ──────────────────────────────────────────────
# speed_kt는 공표 최고속, range_km는 그 속도에서의 최대 항주거리다.
# Mk48은 공식 제원이 의도적으로 낮게 발표되어 있어 실측 추정치를 쓴다.
TORPEDOES = {
    "baeksangeo": {
        "label": "백상어 K731", "kind": "중어뢰",
        "speed_kt": 35.0, "range_km": 30.0,
        "length_m": 6.0, "diameter_mm": 483, "mass_kg": 1100,
        "search": 0.50,   # 중어뢰: 원거리 발사라 탐색·전이가 길다
        "note": "",
    },
    "cheongsangeo": {
        "label": "청상어 K745", "kind": "경어뢰",
        "speed_kt": 45.0, "range_km": 9.0,
        "length_m": 2.7, "diameter_mm": 324, "mass_kg": 280,
        # 경어뢰: 헬기·함정이 표적 위치(datum)를 알고 투하하므로 탐색이 짧다.
        # 이 값이면 재공격 1회가 확보된다(중어뢰 0.5로는 0.8회에 그친다).
        "search": 0.40,
        "note": "",
    },
    # 범상어 K761(60 kt / 50 km)은 사용자 요청으로 제외했다. 되살리려면
    # 아래 주석을 풀면 된다 — 재공격 3회 판정 기준으로는 임무 434 m /
    # 121초라 청상어와 비슷하다(연료 소진 기준일 때만 1750 m로 길어졌다).
    # "beomsangeo": {
    #     "label": "범상어 K761", "kind": "중어뢰",
    #     "speed_kt": 60.0, "range_km": 50.0,
    #     "length_m": 6.5, "diameter_mm": 533, "mass_kg": 1619,
    #     "note": "속도 공표치 55~60 kt 중 상한",
    # },
    "mk48": {
        "label": "Mk 48 ADCAP", "kind": "중어뢰",
        "speed_kt": 55.0, "range_km": 50.0,
        "length_m": 5.79, "diameter_mm": 533, "mass_kg": 1663,
        "search": 0.50,   # 중어뢰
        "note": "공식 제원(28kt/8km)은 축소 발표치라 실측 추정 사용",
    },
}

# ── 어뢰 속도 설정 ──────────────────────────────────────────────────────────
# 실제 어뢰는 속도를 골라 쏜다. 연료(에너지)가 고정이라 속도를 올리면
# 최대거리가 줄어든다. 항력이 v^2에 비례하므로 에너지 E = F x R = k v^2 R,
# 즉 R ∝ 1/v^2 이고 운행시간 T = R/v ∝ 1/v^3 이다.
#
# 검증: Mk48 공개 제원 "40 kt에서 38마일" -> 55 kt 환산 38x(40/55)^2 = 20.1마일.
#       공표치 21마일과 일치한다.
#
# TORPEDOES의 speed_kt/range_km은 "고속(공격)" 설정 기준이다.
# 비율을 0.6까지 낮추면 최대거리가 2.8배로 늘어 외삽이 과해진다(백상어
# 83 km 같은 비현실적 값). 실제 어뢰의 전이/공격 속도비도 0.7~0.75 수준이라,
# 설계점 근처에 머무는 0.80~1.00 구간만 쓴다(최대거리 외삽 1.56배 이내).
SPEED_SETTINGS = {
    "low":    (0.80, "저속 (전이)"),
    "medium": (0.90, "중속 (순항)"),
    "high":   (1.00, "고속 (공격)"),
}


def range_at_speed(range_spec_m, fraction):
    """속도를 공표치의 fraction으로 낮췄을 때의 최대거리 (R ∝ 1/v^2)."""
    return range_spec_m / (fraction ** 2)


# ── glider_slocum 추력-속도 모델 ────────────────────────────────────────────
# model.sdf의 Hydrodynamics 계수에서 유도했다(+Y가 전방이므로 surge는 y축).
#   yV      = -8       선형 항력
#   yVabsV  = -12.474  이차 항력
# 추력 명령 c는 앞/뒤 프로펠러 양쪽에 같은 값이 브리지되므로 총추력은 2c다.
#   2c = 8v + 12.474 v^2
# speed_probe.py 실측 4점과 전부 1.5% 이내로 일치한다.
DRAG_LINEAR = 8.0
DRAG_QUADRATIC = 12.474
THRUST_STEP = 100        # 'r' 한 번이 올리는 추력
THRUST_MAX = 1000        # 어뢰 컨트롤러 상한


def speed_from_thrust(command):
    """추력 명령값 -> 정상상태 속도 (m/s)."""
    discriminant = (DRAG_LINEAR ** 2) + 4.0 * DRAG_QUADRATIC * 2.0 * command
    return (-DRAG_LINEAR + math.sqrt(discriminant)) / (2.0 * DRAG_QUADRATIC)


def thrust_from_speed(speed):
    """목표 속도 -> 필요한 추력 명령값(프로펠러당)."""
    return (DRAG_LINEAR * speed + DRAG_QUADRATIC * speed * speed) / 2.0


def presses_for_speed(speed):
    """목표 속도에 가장 가까운 'r' 누름 횟수와 실제 도달 속도.

    추력이 100 단위로 양자화되어 있어 임의 속도를 정확히 낼 수는 없다.
    가장 가까운 단계를 고르고 오차를 함께 돌려준다.
    """
    ideal = thrust_from_speed(speed)
    presses = max(1, min(THRUST_MAX // THRUST_STEP,
                         round(ideal / THRUST_STEP)))
    actual = speed_from_thrust(presses * THRUST_STEP)
    error = (actual - speed) / speed * 100.0 if speed else 0.0
    saturated = ideal > THRUST_MAX
    return presses, actual, error, saturated


# ── 거리 기준 (실제) ────────────────────────────────────────────────────────
# 시뮬 거리를 손으로 고르면 "왜 그 값이냐"가 남는다. 실제 교전의 거리를
# 먼저 적고 λ로 축소한다. 각 값의 근거와 신뢰도를 함께 적어 둔다.
#
#   확정 = 공개 제원에서 나온 값
#   가정 = 공개 자료가 없어 통상값을 가정한 값 (논문에 가정으로 명시할 것)
REAL_DISTANCES = {
    "launch":  (1700.0, "발사거리 = 발사 플랫폼의 표적 탐지거리", "가정"),
    "mission": (1200.0, "임무 구간 = 잠수함 기동 1개 레그", "가정"),
    "engage":  (350.0, "교전 판정 = 어뢰 종말유도 획득거리", "가정"),
    "hit":     (10.0, "피격 판정 = 근접신관 치명반경", "가정"),
    "barrier": (30.0, "A* 배리어 = 치명반경의 3배 이격", "설계값"),
}

# 지금 시뮬에서 실제로 쓰고 있는 값 (일관성 검사용)
CURRENT_SIM = {
    "launch": 150.0,     # SCENARIOS 좌표
    "mission": 100.0,    # 임무 목표 (0,-100,-1)
    "engage": 30.0,      # planning.avoid.engage_radius
    "hit": 1.0,          # planning.avoid.hit_radius
    "barrier": 3.0,      # planning.barrier.size_*
}


def distances(lam):
    """실제 거리를 λ로 축소하고, 지금 쓰는 값과 대조한다."""
    rows = []
    for key, (real, why, confidence) in REAL_DISTANCES.items():
        scaled = real * lam
        current = CURRENT_SIM[key]
        error = (current - scaled) / scaled * 100.0 if scaled else 0.0
        rows.append({
            "key": key, "real": real, "why": why, "confidence": confidence,
            "scaled": scaled, "current": current, "error": error,
            "current_real": current / lam,
        })
    return rows


def scale(torpedo_key, tau, sub_speed_kt=20.0, approach_fraction=0.20,
          launch_sim=None, mission_margin=1.2, passes_target=None,
          search_fraction=None, speed_setting="high",
          coast_sec=10.0):
    """실제 어뢰 제원을 축소해 시뮬 실험 조건을 만든다.

    tau               시간척도. 실제 운행시간을 이 비율로 줄인다.
    sub_speed_kt      앵커가 되는 잠수함 순항속도.
    approach_fraction 연료(운행시간) 중 표적까지 접근에 쓰는 비율.
    launch_sim        발사거리를 직접 못박을 때(m). 주면 approach_fraction
                      대신 이 값을 쓰고, 그게 연료의 몇 %인지 역산한다.
                      중어뢰는 최대사거리에서 쏘지 않으므로 이쪽이 자연스럽다.
    """
    spec = TORPEDOES[torpedo_key]
    if search_fraction is None:
        search_fraction = spec.get("search", 0.5)

    sigma = ROV_SPEED / (sub_speed_kt * KNOT)     # 속도척도
    lam = sigma * tau                              # 거리척도

    # 속도 설정에 따라 실제 속도와 최대거리가 함께 바뀐다.
    fraction, speed_label = SPEED_SETTINGS[speed_setting]
    speed_spec = spec["speed_kt"] * KNOT
    speed_real = speed_spec * fraction
    range_real = range_at_speed(spec["range_km"] * 1000.0, fraction)
    endurance_real = range_real / speed_real       # 실제 운행시간(초)

    speed_ideal = speed_real * sigma               # 축소된 목표 속도
    presses, speed_sim, speed_error, saturated = presses_for_speed(speed_ideal)

    endurance_sim = endurance_real * tau           # 축소된 총 운행시간
    range_sim = speed_sim * endurance_sim          # 총 항주거리

    # 실제 어뢰는 발사 직후 표적을 잡지 못한다. 탐색 패턴을 돌며 이동하다
    # 음향탐색기가 표적을 획득한 뒤에야 종말유도로 들어간다. 그동안 연료를
    # 쓰므로, 우리 시뮬이 재현하는 "획득 이후" 구간에 남은 연료는 그만큼 적다.
    usable_endurance = endurance_sim * (1.0 - search_fraction)
    usable_range = speed_sim * usable_endurance

    # 발사거리는 무기 사거리가 아니라 발사 플랫폼의 탐지거리로 정해진다
    # (어뢰가 아니라 잠수함 소나가 결정하므로 어뢰 종류와 무관하다).
    # 그래서 실제 탐지거리를 λ로 축소해 쓰고, 그게 그 어뢰 연료의 몇 %인지는
    # 역산한다. 중어뢰일수록 연료 여유가 커진다.
    if launch_sim is None:
        launch_sim = REAL_DISTANCES["launch"][0] * lam
    approach_fraction = launch_sim / usable_range if usable_range else 0.0

    # 접근에 쓰고 남은 연료로 재공격을 몇 번 할 수 있는가.
    # 빗나간 어뢰는 선회해 되돌아오므로 1주기에 대략 발사거리의 2배를 달린다.
    approach_time = launch_sim / speed_sim if speed_sim else 0.0
    cycle_distance = max(launch_sim * 2.0, 1.0)
    cycle_time = cycle_distance / speed_sim if speed_sim else 0.0
    passes_possible = max(0.0, (usable_range - launch_sim) / cycle_distance)

    # 판정 기준은 "어뢰 연료가 다할 때까지 생존"이다. 실제 교리와 같다 —
    # 유도어뢰는 빗나가면 선회해 계속 재공격하므로, 몇 번 피했는지가 아니라
    # 연료가 마를 때까지 버텼는지가 회피 성공이다.
    # passes_target을 주면 그 횟수까지만 보고 끊는다(실험 시간 단축용).
    if passes_target is None:
        judge_time = usable_endurance
    else:
        judge_time = min(usable_endurance,
                         approach_time + passes_target * cycle_time)

    # 임무거리는 그 시간 내내 ROV가 계속 기동할 수 있을 만큼 길어야 한다.
    # 짧으면 ROV가 목표에 먼저 도착해 멈추고, 남은 시간 동안 어뢰가
    # 정지표적을 때리는 판이 된다 — 회피 성능이 아니라 도착 시간을 재게 된다.
    # 연료 소진 뒤 타력 주행 구간에도 ROV는 계속 기동해야 한다. 그때
    # 목표에 도착해 멈추면 어뢰가 정지표적을 향해 굴러오게 된다.
    mission_floor = ROV_SPEED * (judge_time + coast_sec) * mission_margin
    mission_scaled = REAL_DISTANCES["mission"][0] * lam
    mission_sim = max(mission_scaled, mission_floor)
    mission_limited = mission_floor > mission_scaled

    return {
        "key": torpedo_key,
        "spec": spec,
        "sigma": sigma, "tau": tau, "lam": lam,
        "speed_real": speed_real,
        "speed_setting": speed_setting,
        "speed_fraction": fraction,
        "speed_setting_label": speed_label,
        "speed_ideal": speed_ideal,
        "speed_sim": speed_sim,
        "speed_error": speed_error,
        "presses": presses,
        "thrust": presses * THRUST_STEP,
        "saturated": saturated,
        "endurance_real": endurance_real,
        "endurance_sim": endurance_sim,
        "range_real": range_real,
        "range_sim": range_sim,
        "usable_endurance": usable_endurance,
        "usable_range": usable_range,
        "search_fraction": search_fraction,
        "launch_sim": launch_sim,
        "launch_real": launch_sim / lam,
        "mission_sim": mission_sim,
        "mission_real": mission_sim / lam,
        "mission_limited": mission_limited,
        "approach_fraction": approach_fraction,
        "approach_time": approach_time,
        "passes": passes_possible, "cycle_time": cycle_time,
        "judge_time": judge_time, "passes_target": passes_target,
        "judge_label": ("연료 소진까지" if passes_target is None
                        else f"재공격 {passes_target}회까지"),
    }


# ── 시나리오 시작좌표 생성 ──────────────────────────────────────────────────
# ROV는 (0,0,-1)에서 출발해 -Y 방향으로 간다. 어뢰를 ROV 출발점 기준
# 같은 발사거리 위에, 방향만 바꿔 배치한다. 그래야 방향끼리 공정하게
# 비교된다(지금 하드코딩 좌표는 방향마다 거리가 60~150 m로 제각각이다).
DIRECTIONS = {
    "front": (0.0, "정면충돌"),        # ROV 진행방향 정면
    "diag": (45.0, "대각접근"),
    "side": (90.0, "측면횡단"),
    "rear": (180.0, "후방추격"),
}


def scenario_start(direction, launch_m, depth=-1.0):
    """방향과 발사거리로 어뢰 시작 위치·yaw를 만든다.

    돌려주는 값: (x, y, z, yaw). yaw는 ROV 출발점을 겨냥한다.
    어뢰 전방은 body +Y이므로 yaw = atan2(-(ax-tx), ay-ty).
    """
    bearing_deg, _ = DIRECTIONS[direction]
    bearing = math.radians(bearing_deg)
    # 0도 = ROV 정면(-Y), 90도 = 좌현(-X)
    x = -launch_m * math.sin(bearing)
    y = -launch_m * math.cos(bearing)
    yaw = math.atan2(-(0.0 - x), 0.0 - y)
    return (round(x, 2), round(y, 2), depth, round(yaw, 4))


def describe(result):
    """한 어뢰의 축척 결과를 사람이 읽는 문단으로."""
    spec = result["spec"]
    lines = [
        f"■ {spec['label']} ({spec['kind']})"
        + (f"  — {spec['note']}" if spec["note"] else ""),
        f"   설정  {result['speed_setting_label']}"
        f" — 공표 {spec['speed_kt']:.0f} kt의"
        f" {result['speed_fraction']*100:.0f}%",
        f"   실제  {result['speed_real']/KNOT:.0f} kt"
        f" ({result['speed_real']:.1f} m/s)"
        f" / 최대거리 {result['range_real']/1000:.1f} km"
        f" / 운행시간 {result['endurance_real']/60:.1f} 분",
        f"   축소  {result['speed_sim']:.2f} m/s"
        f" (목표 {result['speed_ideal']:.2f}, 오차 {result['speed_error']:+.1f}%)"
        f"  ← 'r' x {result['presses']}회 (추력 {result['thrust']})",
        f"         운행시간 {result['endurance_sim']:.0f} 초"
        f" / 총 항주거리 {result['range_sim']:.0f} m",
        f"         탐색에 {result['search_fraction']*100:.0f}% 소모 →"
        f" 획득 후 남은 연료 {result['usable_endurance']:.0f} 초"
        f" ({result['usable_range']:.0f} m)",
        f"         발사거리 {result['launch_sim']:.0f} m"
        f" (= 실제 {result['launch_real']/1000:.2f} km,"
        f" 연료의 {result['approach_fraction']*100:.1f}%)",
        f"         접근 {result['approach_time']:.0f}초 후 교전,"
        f" 남은 연료로 재공격 약 {result['passes']:.1f}회",
        f"         판정: {result['judge_label']}"
        f" — {result['judge_time']:.0f}초 (재공격 1주기 {result['cycle_time']:.0f}초)",
        f"         임무거리 {result['mission_sim']:.0f} m"
        f" (= 실제 {result['mission_real']/1000:.1f} km)"
        + ("  ← 판정 끝까지 계속 기동하려면 이만큼 필요"
           if result["mission_limited"] else ""),
    ]
    if result["saturated"]:
        lines.append("   ⚠ 추력 상한 초과 — 이 속도는 현재 모델로 재현 불가")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="실제 어뢰 제원을 축소해 실험 조건을 유도한다")
    parser.add_argument("--tau", type=float, default=0.30,
                        help="시간척도 (기본 0.30). 크게 잡을수록 한 판이 길어진다")
    parser.add_argument("--sub-speed", type=float, default=20.0,
                        help="앵커가 되는 잠수함 순항속도 kt (기본 20)")
    parser.add_argument("--search", type=float, default=0.5,
                        help="탐색 단계에서 소모하는 연료 비율 (기본 0.5). "
                             "실제 어뢰는 발사 직후 표적을 잡지 못한다")
    parser.add_argument("--approach", type=float, default=0.20,
                        help="연료 중 접근에 쓰는 비율 (기본 0.20)")
    parser.add_argument("--launch-km", type=float,
                        help="실제 발사거리(km). λ로 축소해 쓴다. 기본은 "
                             f"{REAL_DISTANCES['launch'][0]/1000:.1f} km "
                             "(CRAW급 근접 발사). 중어뢰 교전은 5~20 km")
    parser.add_argument("--launch", type=float,
                        help="발사거리를 시뮬 m로 직접 못박는다(디버그용). "
                             "--launch-km이 있으면 그쪽이 우선")
    args = parser.parse_args()

    if args.launch_km:
        REAL_DISTANCES["launch"] = (
            args.launch_km * 1000.0, REAL_DISTANCES["launch"][1], "지정")
        args.launch = None

    sigma = ROV_SPEED / (args.sub_speed * KNOT)
    lam = sigma * args.tau
    print(f"\n앵커   잠수함 {args.sub_speed:.0f} kt "
          f"({args.sub_speed*KNOT:.2f} m/s)  ↔  ROV {ROV_SPEED:.1f} m/s")
    print(f"척도   속도 σ={sigma:.4f}   시간 τ={args.tau:.2f}   "
          f"거리 λ=στ={lam:.4f}  (1:{1/lam:.1f})")
    print(f"접근   연료의 {args.approach*100:.0f}%를 표적 접근에 사용\n")

    for key in TORPEDOES:
        print(describe(scale(key, args.tau, args.sub_speed, args.approach,
                             args.launch, search_fraction=args.search)))
        print()

    print("거리 축척 — 실제 거리를 λ로 줄이고 현재 값과 대조:")
    print(f"  {'항목':<9}{'실제':<10}{'축소':<10}{'현재':<10}{'차이':<9}"
          f"{'신뢰도':<7}근거")
    for row in distances(lam):
        print(f"  {row['key']:<9}{row['real']:<10.0f}{row['scaled']:<10.2f}"
              f"{row['current']:<10.2f}{row['error']:<+9.1f}"
              f"{row['confidence']:<7}{row['why']}")
    print("  (차이는 현재값이 축소값보다 몇 % 큰가. 양수 = 회피에 불리 = 보수적)")
    print()

    launch = args.launch or REAL_DISTANCES["launch"][0] * lam
    if True:
        print(f"시나리오 시작좌표 (발사거리 {launch:.0f} m, "
              f"ROV는 (0,0,-1)에서 -Y로 진행):")
        print(f"  {'방향':<8}{'방위':<7}{'x':<10}{'y':<10}{'yaw':<9}설명")
        for name, (bearing, label) in DIRECTIONS.items():
            x, y, z, yaw = scenario_start(name, launch)
            print(f"  {name:<8}{bearing:<7.0f}{x:<10.1f}{y:<10.1f}"
                  f"{yaw:<9.4f}{label}")
        print()

    print("도달 가능한 속도 단계 ('r' 누름 횟수별):")
    print(f"  {'r':<4}{'추력':<8}{'속도 m/s':<11}{'실제 환산 kt':<14}")
    for presses in range(1, THRUST_MAX // THRUST_STEP + 1):
        speed = speed_from_thrust(presses * THRUST_STEP)
        print(f"  {presses:<4}{presses*THRUST_STEP:<8}{speed:<11.2f}"
              f"{speed/sigma/KNOT:<14.1f}")
    print()


if __name__ == "__main__":
    main()
