#!/usr/bin/env python3
"""
Compare secrecy HO-outage probability vs time (TN+NTN combined)
for TOTAL 150 UEs vs TOTAL 100 UEs, for BOTH:
  (A) with secrecy xApp (non-ONNX)
  (B) secrecy + ONNX ML gate

Mapping:
- TOTAL 150 UEs = 75 TN + 75 NTN -> (75_ues folders)
- TOTAL 100 UEs = 50 TN + 50 NTN -> (50_ues folders)

Only consider samples up to --tmax seconds (default: 30s).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from statistics import NormalDist
import math

import matplotlib.pyplot as plt

# Use consistent LaTeX-compatible style
plt.style.use('./latex_style.mplstyle')

plt.rcParams.update({
    'font.size': 20,         # base font size for all text
    'axes.titlesize': 20,    # title
    'axes.labelsize': 20,    # x and y labels
    'xtick.labelsize': 20,   # x tick labels
    'ytick.labelsize': 20,   # y tick labels
    'legend.fontsize': 20,   # legend
})


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


def _build_time_map(stats: List[Tuple[float, int, int, float]]) -> Dict[float, Tuple[int, int, float]]:
    return {t: (total, ho_out, prob) for (t, total, ho_out, prob) in stats}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare secrecy HO-outage probability vs time for 100 vs 150 UEs, for xApp vs ONNX (TN+NTN combined)"
    )

    # ---- With-secrecy xApp (non-ONNX) ----
    parser.add_argument("--ues75-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/ntn/75_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--ues75-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/tn/75_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--ues100-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--ues100-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/tn/50_ues/secrecy-sinr-vs-time.txt"))

    # ---- Secrecy + ONNX ML gate ----
    parser.add_argument("--onnx75-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/ntn/75_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--onnx75-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/tn/75_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--onnx100-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--onnx100-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/tn/50_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--out-dir", type=Path, default=Path("results/nr/tn-ntn"))
    parser.add_argument("--ylim", type=str, default="0,0.15",
                        help='Y-axis limits as "ymin,ymax" (e.g., "0,0.2"). Use "auto" for no fixed limits.')
    parser.add_argument("--tmax", type=float, default=30.0,
                        help="Only include samples up to this time (seconds). Default: 30.")
    parser.add_argument("--ci", type=float, default=0.05,
                        help="Confidence level for CI band (e.g., 0.95). Use 0 to disable CI shading.")

    args = parser.parse_args()

    # --- compute all 4 series ---
    stats_xapp_150 = compute_case(args.ues75_ntn, args.ues75_tn, tmax=args.tmax)
    stats_xapp_100 = compute_case(args.ues100_ntn, args.ues100_tn, tmax=args.tmax)

    stats_onnx_150 = compute_case(args.onnx75_ntn, args.onnx75_tn, tmax=args.tmax)
    stats_onnx_100 = compute_case(args.onnx100_ntn, args.onnx100_tn, tmax=args.tmax)

    mx150 = _build_time_map(stats_xapp_150)
    mx100 = _build_time_map(stats_xapp_100)
    mo150 = _build_time_map(stats_onnx_150)
    mo100 = _build_time_map(stats_onnx_100)

    # unified time axis (only times that appear in any series)
    times_all = sorted(set(mx150) | set(mx100) | set(mo150) | set(mo100))
    times_all = [t for t in times_all if t <= args.tmax]

    # build prob + CI arrays for each series
    def build_series(m: Dict[float, Tuple[int, int, float]]):
        probs: List[float] = []
        lo: List[float] = []
        hi: List[float] = []
        for t in times_all:
            total, ho_out, p = m.get(t, (0, 0, 0.0))
            probs.append(p)
            if args.ci and args.ci > 0 and total > 0:
                l, h = wilson_ci(ho_out, total, conf_level=args.ci)
            else:
                l, h = p, p
            lo.append(l)
            hi.append(h)
        return probs, lo, hi

    probs_x150, lo_x150, hi_x150 = build_series(mx150)
    probs_x100, lo_x100, hi_x100 = build_series(mx100)
    probs_o150, lo_o150, hi_o150 = build_series(mo150)
    probs_o100, lo_o100, hi_o100 = build_series(mo100)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    # CSV output (all 4 series)
    csv_path = args.out_dir / "ho_outage_compare_100_vs_150_xapp_vs_onnx.csv"
    with csv_path.open("w", encoding="utf-8") as f:
        f.write(
            "time,"
            "xapp150_total,xapp150_ho_out,xapp150_prob,xapp150_ci_lo,xapp150_ci_hi,"
            "xapp100_total,xapp100_ho_out,xapp100_prob,xapp100_ci_lo,xapp100_ci_hi,"
            "onnx150_total,onnx150_ho_out,onnx150_prob,onnx150_ci_lo,onnx150_ci_hi,"
            "onnx100_total,onnx100_ho_out,onnx100_prob,onnx100_ci_lo,onnx100_ci_hi\n"
        )
        for i, t in enumerate(times_all):
            tx150, hx150, px150 = mx150.get(t, (0, 0, 0.0))
            tx100, hx100, px100 = mx100.get(t, (0, 0, 0.0))
            to150, ho150, po150 = mo150.get(t, (0, 0, 0.0))
            to100, ho100, po100 = mo100.get(t, (0, 0, 0.0))
            f.write(
                f"{t},"
                f"{tx150},{hx150},{px150},{lo_x150[i]},{hi_x150[i]},"
                f"{tx100},{hx100},{px100},{lo_x100[i]},{hi_x100[i]},"
                f"{to150},{ho150},{po150},{lo_o150[i]},{hi_o150[i]},"
                f"{to100},{ho100},{po100},{lo_o100[i]},{hi_o100[i]}\n"
            )

    # Plot (4 curves)
    plt.figure()

    line_x150, = plt.plot(times_all, probs_x150, marker="o", label="Baseline 150 UEs")
    line_x100, = plt.plot(times_all, probs_x100, marker="o", label="Baseline 100 UEs")
    line_o150, = plt.plot(times_all, probs_o150, marker="o", label="ONNX-ML 150 UEs")
    line_o100, = plt.plot(times_all, probs_o100, marker="o", label="ONNX-ML 100 UEs")

    if args.ci and args.ci > 0:
        plt.fill_between(times_all, lo_x150, hi_x150, alpha=0.18, color=line_x150.get_color())
        plt.fill_between(times_all, lo_x100, hi_x100, alpha=0.18, color=line_x100.get_color())
        plt.fill_between(times_all, lo_o150, hi_o150, alpha=0.18, color=line_o150.get_color())
        plt.fill_between(times_all, lo_o100, hi_o100, alpha=0.18, color=line_o100.get_color())

    plt.xlabel("Time (s)")
    plt.ylabel("Secrecy outage probability")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()

    if args.ylim.strip().lower() != "auto":
        y0, y1 = [float(x.strip()) for x in args.ylim.split(",")]
        plt.ylim(y0, y1)

    fig_path = args.out_dir / "ho_outage_compare_100_vs_150_xapp_vs_onnx.png"
    plt.tight_layout()
    plt.savefig(fig_path, dpi=200)
    plt.close()

    # Summary prints
    def summarize(name: str, m: Dict[float, Tuple[int, int, float]], probs: List[float]):
        avg = _mean(probs)
        total_all = sum(m.get(t, (0, 0, 0.0))[0] for t in times_all)
        ho_all = sum(m.get(t, (0, 0, 0.0))[1] for t in times_all)
        p_all = (ho_all / total_all) if total_all > 0 else 0.0
        print(f"{name}: avg(per-time)={avg:.6f} | total={total_all} ho_out={ho_all} prob(total)={p_all:.6f}")
        return avg, p_all

    print("Saved:")
    print(f"  {fig_path}")
    print(f"  {csv_path}")
    print("\n=== Summary (time <= {:.2f}s) ===".format(args.tmax))

    ax150, px150 = summarize("xApp 150", mx150, probs_x150)
    ax100, px100 = summarize("xApp 100", mx100, probs_x100)
    ao150, po150 = summarize("ONNX 150", mo150, probs_o150)
    ao100, po100 = summarize("ONNX 100", mo100, probs_o100)

    # helpful deltas
    if px100 > 0:
        print(f"\nΔ(xApp 150 vs 100) total-prob: {((px150 - px100)/px100*100):.2f}% (positive=worse with more UEs)")
    if po100 > 0:
        print(f"Δ(ONNX 150 vs 100) total-prob: {((po150 - po100)/po100*100):.2f}% (positive=worse with more UEs)")
    if px100 > 0:
        print(f"Δ(ONNX vs xApp) @100 total-prob: {((po100 - px100)/px100*100):.2f}% (negative=ONNX better)")
    if px150 > 0:
        print(f"Δ(ONNX vs xApp) @150 total-prob: {((po150 - px150)/px150*100):.2f}% (negative=ONNX better)")


if __name__ == "__main__":
    main()
