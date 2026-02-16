#!/usr/bin/env python3
"""
Window-first HO-outage comparison (TN+NTN combined)

Cases:
1) WITHOUT secrecy (TN+NTN)
2) WITH secrecy (baseline TN+NTN)
3) WITH secrecy (ONNX TN+NTN)

Window-first HO definition (recommended):
- First filter rows to [tmin,tmax]
- Then mark handover per (domain, UE) using ONLY within-window consecutive samples
- HO-outage event at time t: (handover == True) AND (outage == 1)

Outputs (in out_dir):
- PNG: ho_outage_prob_vs_time_windowfirst_<tmin>to<tmax>_TN_NTN_with_ONNX.png
- CSV: ho_outage_prob_vs_time_windowfirst_<tmin>to<tmax>_TN_NTN_with_ONNX.csv

Also prints TOTAL HO-outage counts + improvements for each case.
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


def in_window(t: float, tmin: Optional[float], tmax: Optional[float]) -> bool:
    if tmin is not None and t < tmin:
        return False
    if tmax is not None and t > tmax:
        return False
    return True


def wilson_ci(x: int, n: int, conf_level: float = 0.95) -> Tuple[float, float]:
    """Wilson score interval for binomial proportion."""
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


def parse_secrecy_file(path: Path, domain: str) -> List[Dict]:
    """
    Parses 'time,type,id,cell,<k,v>...' style.
    Keeps only rows where type == 'UE' and extracts 'cell' and 'outage'.
    """
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")

    rows: List[Dict] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            parts = [p.strip() for p in line.split(",") if p.strip() != ""]
            if len(parts) < 4:
                continue

            # Skip header / malformed
            if not _is_float(parts[0]):
                continue

            t = float(parts[0])
            typ = parts[1] if len(parts) > 1 else ""
            if typ != "UE":
                continue

            try:
                ue_id = int(float(parts[2]))
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
                }
            )

    return rows


def mark_handovers_windowfirst(rows: List[Dict]) -> List[Dict]:
    """
    Mark handovers using ONLY provided rows (already filtered to window).
    HO is True if cell changes between consecutive samples for same UE in same domain,
    within this filtered set.
    """
    rows_sorted = sorted(
        rows,
        key=lambda r: (round(float(r["time"]), 6), r["domain"], int(r["id"]))
    )

    last_cell: Dict[Tuple[str, int], Optional[int]] = {}
    for r in rows_sorted:
        key = (r["domain"], int(r["id"]))
        cur_cell = r.get("cell", None)
        prev_cell = last_cell.get(key, None)

        r["handover"] = (
            (prev_cell is not None)
            and (cur_cell is not None)
            and (cur_cell != prev_cell)
        )

        if cur_cell is not None:
            last_cell[key] = cur_cell

    return rows_sorted


def outage_stats(rows: List[Dict], ci_level: float = 0.95) -> List[Tuple[float, int, int, float, float, float]]:
    """
    Returns per-time:
      (t, total_ues, ho_outage_count, prob, ci_lo, ci_hi)
    where event=1 iff (handover==True AND outage==1).
    """
    total_by_time: Dict[float, int] = {}
    ho_out_by_time: Dict[float, int] = {}

    for r in rows:
        t = round(float(r["time"]), 6)
        total_by_time[t] = total_by_time.get(t, 0) + 1
        if r.get("handover", False) and int(r.get("outage", 0)) == 1:
            ho_out_by_time[t] = ho_out_by_time.get(t, 0) + 1

    all_times = sorted(total_by_time.keys())
    out: List[Tuple[float, int, int, float, float, float]] = []
    for t in all_times:
        total = total_by_time[t]
        ho_out = ho_out_by_time.get(t, 0)
        prob = ho_out / total if total > 0 else 0.0
        if ci_level and ci_level > 0 and total > 0:
            lo, hi = wilson_ci(ho_out, total, conf_level=ci_level)
        else:
            lo, hi = prob, prob
        out.append((t, total, ho_out, prob, lo, hi))
    return out


def compute_case_tn_ntn_windowfirst(
    ntn_path: Path,
    tn_path: Path,
    tmin: float,
    tmax: float,
    ci_level: float,
) -> List[Tuple[float, int, int, float, float, float]]:
    rows_ntn = parse_secrecy_file(ntn_path, domain="ntn")
    rows_tn = parse_secrecy_file(tn_path, domain="tn")
    rows_all = rows_ntn + rows_tn

    # Window-first: filter BEFORE HO marking
    rows_all = [r for r in rows_all if in_window(float(r["time"]), tmin, tmax)]
    rows_all = mark_handovers_windowfirst(rows_all)

    return outage_stats(rows_all, ci_level=ci_level)


def _mean(xs: List[float]) -> float:
    return sum(xs) / len(xs) if xs else 0.0


def _totals_from_map(times: List[float], mp: Dict[float, Tuple[int, int, float, float, float]]) -> Tuple[int, int]:
    total_all = sum(mp.get(t, (0, 0, 0.0, 0.0, 0.0))[0] for t in times)
    ho_out_all = sum(mp.get(t, (0, 0, 0.0, 0.0, 0.0))[1] for t in times)
    return total_all, ho_out_all


def pct_reduction(base: float, new: float) -> Optional[float]:
    """Percent reduction (lower is better)."""
    return ((base - new) / base * 100.0) if base > 0 else None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Window-first HO-outage probability vs time: WO vs W vs ONNX (all TN+NTN)."
    )

    # Baselines (TN+NTN)
    parser.add_argument("--wo-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withoutsecrecylm/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--wo-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withoutsecrecylm/tn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--w-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--w-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/tn/50_ues/secrecy-sinr-vs-time.txt"))

    # ONNX (TN+NTN)
    parser.add_argument("--onnx-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--onnx-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/tn/50_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--tmin", type=float, default=2.0)
    parser.add_argument("--tmax", type=float, default=30.0)

    parser.add_argument("--out-dir", type=Path, default=Path("results/nr/tn-ntn"))
    parser.add_argument("--ylim", type=str, default="0,1",
                        help='Y-axis limits "ymin,ymax" (e.g., "0,0.2"). Use "auto" for no fixed limits.')

    parser.add_argument("--ci", type=float, default=0.50,
                        help="Confidence level for CI band (e.g., 0.50). Use 0 to disable CI shading.")

    args = parser.parse_args()
    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    tmin, tmax = args.tmin, args.tmax
    ci_level = args.ci

    # Compute three series
    stats_wo   = compute_case_tn_ntn_windowfirst(args.wo_ntn,   args.wo_tn,   tmin, tmax, ci_level)
    stats_w    = compute_case_tn_ntn_windowfirst(args.w_ntn,    args.w_tn,    tmin, tmax, ci_level)
    stats_onnx = compute_case_tn_ntn_windowfirst(args.onnx_ntn, args.onnx_tn, tmin, tmax, ci_level)

    wo_map   = {t: (total, ho_out, prob, lo, hi) for (t, total, ho_out, prob, lo, hi) in stats_wo}
    w_map    = {t: (total, ho_out, prob, lo, hi) for (t, total, ho_out, prob, lo, hi) in stats_w}
    onnx_map = {t: (total, ho_out, prob, lo, hi) for (t, total, ho_out, prob, lo, hi) in stats_onnx}

    # Common time axis across all three
    times_all = sorted(set(wo_map.keys()).union(set(w_map.keys())).union(set(onnx_map.keys())))
    times_all = [t for t in times_all if in_window(t, tmin, tmax)]

    # Build plot arrays
    probs_wo, probs_w, probs_onnx = [], [], []
    wo_lo, wo_hi, w_lo, w_hi, onnx_lo, onnx_hi = [], [], [], [], [], []

    for t in times_all:
        wo_total, wo_ho_out, wo_prob, lo0, hi0 = wo_map.get(t, (0, 0, 0.0, 0.0, 0.0))
        w_total,  w_ho_out,  w_prob,  lo1, hi1 = w_map.get(t, (0, 0, 0.0, 0.0, 0.0))
        o_total,  o_ho_out,  o_prob,  lo2, hi2 = onnx_map.get(t, (0, 0, 0.0, 0.0, 0.0))

        probs_wo.append(wo_prob)
        probs_w.append(w_prob)
        probs_onnx.append(o_prob)

        wo_lo.append(lo0); wo_hi.append(hi0)
        w_lo.append(lo1);  w_hi.append(hi1)
        onnx_lo.append(lo2); onnx_hi.append(hi2)

    # CSV output
    csv_path = out_dir / f"ho_outage_prob_vs_time_windowfirst_{int(tmin)}to{int(tmax)}_TN_NTN_with_ONNX.csv"
    with csv_path.open("w", encoding="utf-8") as f:
        f.write(
            "time,"
            "wo_total_ues,wo_ho_outage,wo_prob,wo_ci_lo,wo_ci_hi,"
            "w_total_ues,w_ho_outage,w_prob,w_ci_lo,w_ci_hi,"
            "onnx_total_ues,onnx_ho_outage,onnx_prob,onnx_ci_lo,onnx_ci_hi\n"
        )
        for t in times_all:
            wo_total, wo_ho_out, wo_prob, lo0, hi0 = wo_map.get(t, (0, 0, 0.0, 0.0, 0.0))
            w_total,  w_ho_out,  w_prob,  lo1, hi1 = w_map.get(t, (0, 0, 0.0, 0.0, 0.0))
            o_total,  o_ho_out,  o_prob,  lo2, hi2 = onnx_map.get(t, (0, 0, 0.0, 0.0, 0.0))
            f.write(
                f"{t},"
                f"{wo_total},{wo_ho_out},{wo_prob},{lo0},{hi0},"
                f"{w_total},{w_ho_out},{w_prob},{lo1},{hi1},"
                f"{o_total},{o_ho_out},{o_prob},{lo2},{hi2}\n"
            )

    # Plot
    fig_path = out_dir / f"ho_outage_prob_vs_time_windowfirst_{int(tmin)}to{int(tmax)}_TN_NTN_with_ONNX.png"
    plt.figure()
    plt.plot(times_all, probs_wo,   marker="o", label="Without secrecy")
    plt.plot(times_all, probs_w,    marker="o", label="With secrecy (baseline)")
    plt.plot(times_all, probs_onnx, marker="o", label="With secrecy (ONNX ML)")

    if ci_level and ci_level > 0:
        plt.fill_between(times_all, wo_lo, wo_hi, alpha=0.20)
        plt.fill_between(times_all, w_lo,  w_hi,  alpha=0.20)
        plt.fill_between(times_all, onnx_lo, onnx_hi, alpha=0.20)

    plt.xlabel("Time (s)")
    plt.ylabel("Secrecy outage probability")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()

    if args.ylim.strip().lower() != "auto":
        y0, y1 = [float(x.strip()) for x in args.ylim.split(",")]
        plt.ylim(y0, y1)

    plt.tight_layout()
    plt.savefig(fig_path, dpi=200)
    plt.close()

    # Totals (window-first)
    wo_total_all, wo_ho_out_all = _totals_from_map(times_all, wo_map)
    w_total_all,  w_ho_out_all  = _totals_from_map(times_all, w_map)
    o_total_all,  o_ho_out_all  = _totals_from_map(times_all, onnx_map)

    total_prob_wo = (wo_ho_out_all / wo_total_all) if wo_total_all > 0 else 0.0
    total_prob_w  = (w_ho_out_all / w_total_all) if w_total_all > 0 else 0.0
    total_prob_o  = (o_ho_out_all / o_total_all) if o_total_all > 0 else 0.0

    avg_wo = _mean(probs_wo)
    avg_w  = _mean(probs_w)
    avg_o  = _mean(probs_onnx)

    # Improvements
    imp_w_vs_wo_avg   = pct_reduction(avg_wo, avg_w)
    imp_w_vs_wo_tot   = pct_reduction(total_prob_wo, total_prob_w)

    imp_o_vs_wo_avg   = pct_reduction(avg_wo, avg_o)
    imp_o_vs_wo_tot   = pct_reduction(total_prob_wo, total_prob_o)

    imp_o_vs_w_avg    = pct_reduction(avg_w,  avg_o)
    imp_o_vs_w_tot    = pct_reduction(total_prob_w, total_prob_o)

    print("Saved:")
    print(f"  {fig_path}")
    print(f"  {csv_path}")
    print(f"Time window used (window-first HO): [{tmin}, {tmax}]")

    print("\nTOTAL HO-outage count in window (HO only if both samples are inside window):")
    print(f"  Without secrecy (TN+NTN):         {wo_ho_out_all}")
    print(f"  With secrecy (TN+NTN):            {w_ho_out_all}")
    print(f"  With secrecy (ONNX, TN+NTN):      {o_ho_out_all}")

    print("\nTotal-count-based probabilities (window-first):")
    print(f"  Without secrecy (TN+NTN):      {total_prob_wo:.6f}   (UE rows={wo_total_all})")
    print(f"  With secrecy (TN+NTN):         {total_prob_w:.6f}   (UE rows={w_total_all})")
    print(f"  With secrecy (ONNX, TN+NTN):   {total_prob_o:.6f}   (UE rows={o_total_all})")

    print("\nAvg per-time probabilities (window-first):")
    print(f"  Without secrecy (TN+NTN):      {avg_wo:.6f}")
    print(f"  With secrecy (TN+NTN):         {avg_w:.6f}")
    print(f"  With secrecy (ONNX, TN+NTN):   {avg_o:.6f}")

    print("\nImprovement (lower is better):")

    if imp_w_vs_wo_avg is None:
        print("  With secrecy vs Without secrecy: N/A (baseline=0)")
    else:
        print("  With secrecy vs Without secrecy:")
        print(f"    Avg-per-time reduction: {imp_w_vs_wo_avg:.2f}%")
        print(f"    Total-count reduction:  {imp_w_vs_wo_tot:.2f}%")

    if imp_o_vs_wo_avg is None:
        print("  ONNX vs Without secrecy: N/A (baseline=0)")
    else:
        print("  ONNX vs Without secrecy:")
        print(f"    Avg-per-time reduction: {imp_o_vs_wo_avg:.2f}%")
        print(f"    Total-count reduction:  {imp_o_vs_wo_tot:.2f}%")

    if imp_o_vs_w_avg is None:
        print("  ONNX vs With secrecy (baseline): N/A (baseline=0)")
    else:
        print("  ONNX vs With secrecy (baseline):")
        print(f"    Avg-per-time reduction: {imp_o_vs_w_avg:.2f}%")
        print(f"    Total-count reduction:  {imp_o_vs_w_tot:.2f}%")


if __name__ == "__main__":
    main()
