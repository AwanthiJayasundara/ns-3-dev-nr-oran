#!/usr/bin/env python3
"""
Compare secrecy outage probability vs time (TN+NTN combined)
for TOTAL 150 UEs vs TOTAL 100 UEs, with optional confidence intervals (CIs).

IMPORTANT (your mapping):
- TOTAL 150 UEs case is produced by 75 UEs in TN + 75 UEs in NTN  -> (75_ues folders)
- TOTAL 100 UEs case is produced by 50 UEs in TN + 50 UEs in NTN  -> (50_ues folders)

Only consider samples up to --tmax seconds (default: 30s).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from statistics import NormalDist
import math

import matplotlib.pyplot as plt


def _is_float(s: str) -> bool:
    try:
        float(s)
        return True
    except Exception:
        return False


def _safe_int(x: Optional[str]) -> Optional[int]:
    if x is None:
        return None
    try:
        return int(float(x))
    except Exception:
        return None


def parse_secrecy_file(path: Path, domain: str, tmax: float) -> List[Dict]:
    rows: List[Dict] = []
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            parts = [p.strip() for p in line.split(",") if p.strip() != ""]
            if len(parts) < 4:
                continue

            # skip header/malformed
            if not _is_float(parts[0]):
                continue

            t = float(parts[0])
            if t > tmax:
                continue

            typ = parts[1] if len(parts) > 1 else ""

            try:
                ue_id = int(parts[2])
            except Exception:
                continue

            kv: Dict[str, str] = {}
            i = 3
            while i + 1 < len(parts):
                k = parts[i]
                v = parts[i + 1]
                kv[k] = v
                i += 2

            cell_id = _safe_int(kv.get("cell", None))

            outage = 0
            outage_raw = kv.get("outage", None)
            if outage_raw is not None:
                try:
                    outage = int(float(outage_raw))
                except Exception:
                    outage = 0

            rows.append(
                {
                    "time": t,
                    "type": typ,
                    "id": ue_id,
                    "cell": cell_id,
                    "outage": outage,
                    "domain": domain,
                    **kv,
                }
            )

    return rows


def mark_handovers(rows: List[Dict]) -> List[Dict]:
    rows_sorted = sorted(
        rows, key=lambda r: (round(float(r["time"]), 6), r["domain"], int(r["id"]))
    )

    last_cell: Dict[Tuple[str, int], Optional[int]] = {}

    for r in rows_sorted:
        key = (r["domain"], int(r["id"]))
        cur_cell = r.get("cell", None)
        prev_cell = last_cell.get(key, None)

        # handover if cell changes between successive samples for same UE in same domain
        r["handover"] = (prev_cell is not None) and (cur_cell is not None) and (cur_cell != prev_cell)

        if cur_cell is not None:
            last_cell[key] = cur_cell

    return rows_sorted


def outage_stats_ho_outage_over_all_ues(rows: List[Dict]) -> List[Tuple[float, int, int, float]]:
    total_by_time: Dict[float, int] = {}
    ho_out_by_time: Dict[float, int] = {}

    for r in rows:
        if r.get("type") != "UE":
            continue

        t = round(float(r["time"]), 6)
        total_by_time[t] = total_by_time.get(t, 0) + 1

        if r.get("handover", False) and int(r.get("outage", 0)) == 1:
            ho_out_by_time[t] = ho_out_by_time.get(t, 0) + 1

    all_times = sorted(total_by_time.keys())

    stats: List[Tuple[float, int, int, float]] = []
    for t in all_times:
        total = total_by_time.get(t, 0)
        ho_out = ho_out_by_time.get(t, 0)
        prob = (ho_out / total) if total > 0 else 0.0
        stats.append((t, total, ho_out, prob))

    return stats


def compute_case(ntn_path: Path, tn_path: Path, tmax: float) -> List[Tuple[float, int, int, float]]:
    rows_ntn = parse_secrecy_file(ntn_path, domain="ntn", tmax=tmax)
    rows_tn = parse_secrecy_file(tn_path, domain="tn", tmax=tmax)
    rows_all = mark_handovers(rows_ntn + rows_tn)
    return outage_stats_ho_outage_over_all_ues(rows_all)


def _mean(xs: List[float]) -> float:
    return sum(xs) / len(xs) if xs else 0.0


def wilson_ci(x: int, n: int, conf_level: float = 0.95) -> Tuple[float, float]:
    if n <= 0:
        return (0.0, 0.0)
    x = max(0, min(x, n))
    p = x / n

    alpha = 1.0 - conf_level
    z = NormalDist().inv_cdf(1.0 - alpha / 2.0)

    denom = 1.0 + (z * z) / n
    center = (p + (z * z) / (2.0 * n)) / denom
    half = (z * math.sqrt((p * (1.0 - p) / n) + (z * z) / (4.0 * n * n))) / denom

    lo = max(0.0, center - half)
    hi = min(1.0, center + half)
    return lo, hi


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare secrecy outage probability vs time for TOTAL 150 UEs vs TOTAL 100 UEs (TN+NTN combined)"
    )

    # 150 UEs (75 TN + 75 NTN)
    parser.add_argument("--ues75-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/ntn/75_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--ues75-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/tn/75_ues/secrecy-sinr-vs-time.txt"))

    # 100 UEs (50 TN + 50 NTN)
    parser.add_argument("--ues100-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--ues100-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/tn/50_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--out-dir", type=Path, default=Path("results/nr/tn-ntn"))

    # Change this anytime: e.g. --ylim "0,0.2" or --ylim auto
    parser.add_argument("--ylim", type=str, default="0,0.15",
                        help='Y-axis limits as "ymin,ymax" (e.g., "0,0.2"). Use "auto" for no fixed limits.')

    # Only consider up to 30 seconds
    parser.add_argument("--tmax", type=float, default=30.0,
                        help="Only include samples up to this time (seconds). Default: 30.")

    parser.add_argument("--ci", type=float, default=0.25,
                        help="Confidence level for CI band (e.g., 0.25). Use 0 to disable CI shading.")

    args = parser.parse_args()

    # compute cases (TOTAL 150 uses 75+75, TOTAL 100 uses 50+50)
    stats_150 = compute_case(args.ues75_ntn, args.ues75_tn, tmax=args.tmax)
    stats_100 = compute_case(args.ues100_ntn, args.ues100_tn, tmax=args.tmax)

    times_150 = {s[0] for s in stats_150}
    times_100 = {s[0] for s in stats_100}
    times_all = sorted(t for t in times_150.union(times_100) if t <= args.tmax)

    m150 = {t: (total, ho_out, prob) for (t, total, ho_out, prob) in stats_150}
    m100 = {t: (total, ho_out, prob) for (t, total, ho_out, prob) in stats_100}

    probs_150: List[float] = []
    probs_100: List[float] = []

    lo_150: List[float] = []
    hi_150: List[float] = []
    lo_100: List[float] = []
    hi_100: List[float] = []

    for t in times_all:
        total150, ho150, p150 = m150.get(t, (0, 0, 0.0))
        total100, ho100, p100 = m100.get(t, (0, 0, 0.0))

        probs_150.append(p150)
        probs_100.append(p100)

        if args.ci and args.ci > 0 and total150 > 0:
            lo, hi = wilson_ci(ho150, total150, conf_level=args.ci)
        else:
            lo, hi = p150, p150
        lo_150.append(lo)
        hi_150.append(hi)

        if args.ci and args.ci > 0 and total100 > 0:
            lo, hi = wilson_ci(ho100, total100, conf_level=args.ci)
        else:
            lo, hi = p100, p100
        lo_100.append(lo)
        hi_100.append(hi)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    # CSV
    csv_path = args.out_dir / "ho_outage_compare_100_vs_150.csv"
    with csv_path.open("w", encoding="utf-8") as f:
        f.write(
            "time,"
            "ues150_total_ues,ues150_ho_outage_count,ues150_prob,ues150_ci_lo,ues150_ci_hi,"
            "ues100_total_ues,ues100_ho_outage_count,ues100_prob,ues100_ci_lo,ues100_ci_hi\n"
        )
        for i, t in enumerate(times_all):
            total150, ho150, p150 = m150.get(t, (0, 0, 0.0))
            total100, ho100, p100 = m100.get(t, (0, 0, 0.0))
            f.write(
                f"{t},"
                f"{total150},{ho150},{p150},{lo_150[i]},{hi_150[i]},"
                f"{total100},{ho100},{p100},{lo_100[i]},{hi_100[i]}\n"
            )

    # Plot
    plt.figure()
    plt.plot(times_all, probs_150, marker="o", label="150 UEs (75 TN + 75 NTN)")
    plt.plot(times_all, probs_100, marker="o", label="100 UEs (50 TN + 50 NTN)")

    if args.ci and args.ci > 0:
        plt.fill_between(times_all, lo_150, hi_150, alpha=0.20)
        plt.fill_between(times_all, lo_100, hi_100, alpha=0.20)

    plt.xlabel("Time (s)")
    plt.ylabel("Secrecy outage probability")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()

    if args.ylim.strip().lower() != "auto":
        y0, y1 = [float(x.strip()) for x in args.ylim.split(",")]
        plt.ylim(y0, y1)

    fig_path = args.out_dir / "ho_outage_compare_100_vs_150.png"
    plt.tight_layout()
    plt.savefig(fig_path, dpi=200)
    plt.close()

    # Summary
    avg_150 = _mean(probs_150)
    avg_100 = _mean(probs_100)
    change_pct = ((avg_150 - avg_100) / avg_100 * 100.0) if avg_100 > 0 else None

    total150_all = sum(m150.get(t, (0, 0, 0.0))[0] for t in times_all)
    ho150_all = sum(m150.get(t, (0, 0, 0.0))[1] for t in times_all)
    total100_all = sum(m100.get(t, (0, 0, 0.0))[0] for t in times_all)
    ho100_all = sum(m100.get(t, (0, 0, 0.0))[1] for t in times_all)

    p150_all = (ho150_all / total150_all) if total150_all > 0 else 0.0
    p100_all = (ho100_all / total100_all) if total100_all > 0 else 0.0
    change_total_pct = ((p150_all - p100_all) / p100_all * 100.0) if p100_all > 0 else None

    print("Saved:")
    print(f"  {fig_path}")
    print(f"  {csv_path}")

    print("\n=== Summary (time <= {:.2f}s) ===".format(args.tmax))
    print(f"Avg per-time prob (150 UEs): {avg_150:.6f}")
    print(f"Avg per-time prob (100 UEs): {avg_100:.6f}")
    if change_pct is None:
        print("Change (avg prob): N/A (baseline avg_100 is 0)")
    else:
        print(f"Change (avg prob): {change_pct:.2f}%  (positive = worse with more UEs)")

    print("\nTotal-count-based (more robust):")
    print(f"150 UEs: total={total150_all}, HO-outage={ho150_all}, prob={p150_all:.6f}")
    print(f"100 UEs: total={total100_all}, HO-outage={ho100_all}, prob={p100_all:.6f}")
    if change_total_pct is None:
        print("Change (total prob): N/A (baseline p100_all is 0)")
    else:
        print(f"Change (total prob): {change_total_pct:.2f}%  (positive = worse with more UEs)")


if __name__ == "__main__":
    main()
