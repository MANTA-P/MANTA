#!/usr/bin/env python3
"""results.csv 를 남에게 보낼 수 있는 표로 만든다.

노션·문서에 그대로 붙일 수 있게 마크다운으로 낸다. 기준선이 있으면
같은 조건끼리 짝지어 비교표와 유의성(양측 Fisher 정확검정)까지 붙인다.

  ./make_report.py                                   현재 결과
  ./make_report.py <결과폴더>                         지정 폴더
  ./make_report.py <결과폴더> --base <기준선폴더>      비교표 포함
"""
import argparse
import csv
import pathlib
import sys
from math import comb

DEFAULT = pathlib.Path.home() / "manta_experiments/results"
TORPEDO_NAME = {"baeksangeo": "백상어", "cheongsangeo": "청상어", "mk48": "Mk 48"}
MODE_NAME = {"2": "순수추적", "3": "PNG"}
SCENARIO_NAME = {"front": "정면", "rear": "후방", "side": "측면",
                 "side_left": "측면좌", "diag": "대각"}


def load(folder):
    path = pathlib.Path(folder) / "results.csv"
    if not path.exists():
        sys.exit(f"results.csv 가 없다: {path}")
    return list(csv.DictReader(open(path)))


def valid(rows):
    return [r for r in rows if not r["결과"].startswith("INVALID")]


def rescore_by_passes(rows, limit=3):
    """연료소진 기준으로 모은 기준선을 '공격 N회 생존' 기준으로 다시 채점한다.

    두 실험의 판정 기준이 다르면 성공률을 나란히 놓을 수 없다. 기준선
    로그에 피격 회차가 남아 있어 offline 으로 환산할 수 있다.
    """
    for row in rows:
        if row["결과"] == "HIT" and row["피격회차"]:
            try:
                if int(row["피격회차"]) > limit:
                    row["결과"] = "AVOIDED"
            except ValueError:
                pass
    return rows


def fisher(a, b, c, d):
    """양측 Fisher 정확검정. a/b 는 성공/실패."""
    n, row1, col1 = a + b + c + d, a + b, a + c
    def p(x):
        return comb(row1, x) * comb(n - row1, col1 - x) / comb(n, col1)
    observed = p(a)
    return sum(p(x) for x in range(max(0, col1 - (n - row1)),
                                    min(row1, col1) + 1)
               if p(x) <= observed + 1e-12)


def rate(rows):
    rows = valid(rows)
    if not rows:
        return None
    ok = sum(1 for r in rows if r["결과"] == "AVOIDED")
    return ok, len(rows)


def margins(rows):
    values = [float(r["최근접m"]) for r in valid(rows) if r["최근접m"]]
    return values


def section(out, rows, base, title, key):
    """한 축(플래너/모드/방향/어뢰)으로 쪼갠 표."""
    values = sorted({r[key] for r in rows})
    out.append(f"\n### {title}\n")
    header = "| 구분 | 성공 | 성공률 |"
    align = "|---|---|---|"
    if base:
        header += " 기준선 | p |"
        align += "---|---|"
    out.append(header)
    out.append(align)
    for value in values:
        subset = [r for r in rows if r[key] == value]
        got = rate(subset)
        if not got:
            continue
        ok, total = got
        label = {"모드": MODE_NAME, "어뢰": TORPEDO_NAME,
                 "시나리오": SCENARIO_NAME}.get(key, {}).get(value, value)
        line = f"| {label} | {ok}/{total} | {100 * ok / total:.0f}% |"
        if base:
            prior = rate([r for r in base if r[key] == value])
            if prior:
                p0, n0 = prior
                pv = fisher(p0, n0 - p0, ok, total - ok)
                line += f" {p0}/{n0} ({100 * p0 / n0:.0f}%) | {pv:.3f} |"
            else:
                line += " - | - |"
        out.append(line)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", default=str(DEFAULT))
    parser.add_argument("--base", help="비교할 기준선 폴더")
    parser.add_argument("--rescore-base", type=int, default=3,
                        help="기준선을 '공격 N회 생존' 기준으로 환산")
    parser.add_argument("--out", help="저장 경로(기본: 결과폴더/결과표.md)")
    args = parser.parse_args()

    rows = load(args.folder)
    base = None
    if args.base:
        base = rescore_by_passes(load(args.base), args.rescore_base)

    out = ["# 회피 실험 결과", ""]
    got = rate(rows)
    if got:
        ok, total = got
        out.append(f"**전체 {ok}/{total} ({100 * ok / total:.0f}%)**")
        dropped = len(rows) - total
        if dropped:
            out.append(f"  무효 {dropped}판 제외")
        values = margins(rows)
        if values:
            out.append(f"  최근접 평균 {sum(values) / len(values):.2f} m "
                       f"(최소 {min(values):.2f} m, 피격반경 1.0 m)")
    if base:
        prior = rate(base)
        if prior and got:
            pv = fisher(prior[0], prior[1] - prior[0], got[0], got[1] - got[0])
            out.append(f"  기준선 {prior[0]}/{prior[1]} "
                       f"({100 * prior[0] / prior[1]:.0f}%), p={pv:.3f}")

    for title, key in (("플래너별", "플래너"), ("어뢰 모드별", "모드"),
                       ("방향별", "시나리오"), ("어뢰별", "어뢰")):
        section(out, rows, base, title, key)

    out.append("\n### 판별 내역\n")
    out.append("| 번호 | 방향 | 플래너 | 모드 | 어뢰 | 결과 | 최근접 | 피격회차 | 교전 | 계획 |")
    out.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        result = {"AVOIDED": "○ 회피", "HIT": "✗ 피격"}.get(
            r["결과"], f"− {r['결과']}")
        out.append(
            f"| {r['번호']} | {SCENARIO_NAME.get(r['시나리오'], r['시나리오'])} "
            f"| {r['플래너']} | {MODE_NAME.get(r['모드'], r['모드'])} "
            f"| {TORPEDO_NAME.get(r['어뢰'], r['어뢰'])} | {result} "
            f"| {r['최근접m'] or '-'} | {r['피격회차'] or '-'} "
            f"| {r['교전횟수']} | {r['계획횟수']} |")

    out.append("\n### 판정 기준\n")
    out.append("- 피격반경 1.0 m, 교전반경 33 m")
    out.append("- 공격 3회를 버티면 회피 성공")
    out.append("- 어뢰 운행시간은 실제 제원을 축척(τ=0.30)해 계산")
    if base:
        out.append(f"- 기준선은 연료소진 기준으로 모은 것을 "
                   f"'공격 {args.rescore_base}회 생존'으로 환산해 비교")

    target = pathlib.Path(args.out) if args.out else \
        pathlib.Path(args.folder) / "결과표.md"
    target.write_text("\n".join(out) + "\n")
    print(f"저장: {target}")


if __name__ == "__main__":
    main()
