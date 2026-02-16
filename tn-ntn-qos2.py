#!/usr/bin/env python3
"""
QoS mean±std vs time plots for TN/NTN with/without secrecy (LM).

Reads (relative to --base):
  withoutsecrecylm/tn/50_ues/qos-vs-time.txt
  withoutsecrecylm/ntn/50_ues/qos-vs-time.txt
  withsecrecylm/tn/50_ues/qos-vs-time.txt
  withsecrecylm/ntn/50_ues/qos-vs-time.txt

Generates mean vs time with shaded std for:
  Delay, Jitter, Throughput, PDR
Comparing: WithoutSecrecy vs WithSecrecy (TN+NTN combined)

IMPORTANT cleaning (for your “all-zero” rows):
- Drops rows where Delay=Jitter=Throughput=PDR=PLR are all 0 (default ON)
- Drops PDR==0 rows for PDR averaging/plotting (default ON)

Output:
  meanstd_delay_vs_time_combined.png
  meanstd_jitter_vs_time_combined.png
  meanstd_throughput_vs_time_combined.png
  meanstd_pdr_vs_time_combined.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Use your LaTeX style
plt.style.use("./latex_style.mplstyle")

plt.rcParams.update({
    "font.size": 20,
    "axes.titlesize": 20,
    "axes.labelsize": 20,
    "xtick.labelsize": 20,
    "ytick.labelsize": 20,
    "legend.fontsize": 20,
})

COLUMNS: List[str] = ["Time", "UE", "Delay", "Jitter", "Throughput", "PDR", "PLR"]
METRICS: List[str] = ["Delay", "Jitter", "Throughput", "PDR"]


def _first_nonempty_line(path: Path) -> str:
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if s:
                return s
    return ""


def has_header(path: Path) -> bool:
    first = _first_nonempty_line(path)
    return any(ch.isalpha() for ch in first)


def load_qos(path: Path, *, scenario: str, domain: str) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")

    if has_header(path):
        df = pd.read_csv(path, sep=",", skipinitialspace=True)
    else:
        df = pd.read_csv(path, sep=",", names=COLUMNS, header=None, skipinitialspace=True)

    df.columns = [str(c).strip() for c in df.columns]

    missing = [c for c in COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(
            f"{path} does not contain expected columns {missing}. "
            f"Found columns: {list(df.columns)}"
        )

    df = df[COLUMNS].copy()

    # numeric coercion
    for c in COLUMNS:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df["Scenario"] = scenario
    df["Domain"] = domain
    return df


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def drop_all_zero_rows(df: pd.DataFrame) -> pd.DataFrame:
    """
    Drops rows where ALL KPI fields are zero:
      Delay=0, Jitter=0, Throughput=0, PDR=0, PLR=0
    This matches your initial rows like:
      2,0,0,0,0,0,0
    """
    kpis = ["Delay", "Jitter", "Throughput", "PDR", "PLR"]
    mask_all_zero = (df[kpis].fillna(0.0) == 0.0).all(axis=1)
    return df.loc[~mask_all_zero].copy()


def metric_units(metric: str, latency_unit: str) -> Tuple[str, float]:
    """
    Returns (ylabel, scale_factor) for the metric.
    latency_unit: "s" or "ms"
    """
    if metric in ("Delay", "Jitter"):
        if latency_unit == "ms":
            return f"{metric} (ms)", 1000.0
        return f"{metric} (s)", 1.0
    if metric == "Throughput":
        return "Throughput (Mbps)", 1.0
    if metric == "PDR":
        return "PDR", 1.0
    return metric, 1.0


def compute_mean_std_by_time(
    df: pd.DataFrame,
    metric: str,
    scenario: str,
    *,
    drop_zero_pdr: bool,
) -> Tuple[pd.Series, pd.Series]:
    """
    Compute mean and std indexed by Time for given scenario and metric.
    For PDR, optionally drop PDR==0 rows so they don't affect average/std.
    """
    d = df[df["Scenario"] == scenario].copy()

    if metric == "PDR" and drop_zero_pdr:
        d = d[d["PDR"].notna() & (d["PDR"] > 30.0)]

    g = d.groupby("Time")[metric]
    mean = g.mean().sort_index()
    std = g.std().sort_index().fillna(0.0)  # std=0 when only one sample
    return mean, std


def apply_rolling(series: pd.Series, window: int) -> pd.Series:
    if window <= 1:
        return series
    return series.rolling(window=window, min_periods=1).mean()


def save_mean_std_plot(
    df: pd.DataFrame,
    metric: str,
    out_path: Path,
    *,
    window: int,
    latency_unit: str,
    drop_zero_pdr: bool,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.6))

    ylabel, scale = metric_units(metric, latency_unit)

    for scen in ["WithoutSecrecy", "WithSecrecy"]:
        mean, std = compute_mean_std_by_time(df, metric, scen, drop_zero_pdr=drop_zero_pdr)

        # scale if needed (ms conversion for Delay/Jitter)
        mean = mean * scale
        std = std * scale

        # optional smoothing (rolling over time samples)
        mean_s = apply_rolling(mean, window)
        std_s = apply_rolling(std, window)

        ax.plot(mean_s.index, mean_s.values, label=scen)
        ax.fill_between(
            mean_s.index,
            (mean_s - std_s).values,
            (mean_s + std_s).values,
            alpha=0.20,
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()

    # Nice bounds for percentage
    if metric == "PDR":
        ax.set_ylim(0, 105)

    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="QoS mean±std vs time plots with proper units + zero-row filtering.")
    parser.add_argument(
        "--base",
        type=Path,
        default=Path("results/nr/tn-ntn"),
        help="Base directory containing withoutsecrecylm/ and withsecrecylm/ (default: results/nr/tn-ntn)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("results/nr/tn-ntn/qos-meanstd"),
        help="Output directory for plots (default: results/nr/tn-ntn/qos-meanstd)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=1,
        help="Rolling window (number of time samples) to smooth mean/std (default: 1 = no smoothing)",
    )
    parser.add_argument(
        "--latency-unit",
        choices=["s", "ms"],
        default="s",
        help="Units for Delay/Jitter axis: s or ms (default: s)",
    )
    parser.add_argument(
        "--drop-allzero",
        action="store_true",
        default=True,
        help="Drop rows where Delay/Jitter/Throughput/PDR/PLR are ALL zero (default: ON)",
    )
    parser.add_argument(
        "--keep-allzero",
        action="store_true",
        help="Keep all-zero rows (overrides --drop-allzero)",
    )
    parser.add_argument(
        "--drop-zero-pdr",
        action="store_true",
        default=True,
        help="Exclude PDR==0 rows from PDR mean/std (default: ON)",
    )
    parser.add_argument(
        "--keep-zero-pdr",
        action="store_true",
        help="Include PDR==0 rows in PDR mean/std (overrides --drop-zero-pdr)",
    )
    args = parser.parse_args()

    ensure_dir(args.out)

    drop_allzero = args.drop_allzero and not args.keep_allzero
    drop_zero_pdr = args.drop_zero_pdr and not args.keep_zero_pdr

    files = [
        (args.base / "withoutsecrecylm" / "tn" / "50_ues" / "qos-vs-time.txt", "WithoutSecrecy", "TN"),
        (args.base / "withoutsecrecylm" / "ntn" / "50_ues" / "qos-vs-time.txt", "WithoutSecrecy", "NTN"),
        (args.base / "withsecrecylm" / "tn" / "50_ues" / "qos-vs-time.txt", "WithSecrecy", "TN"),
        (args.base / "withsecrecylm" / "ntn" / "50_ues" / "qos-vs-time.txt", "WithSecrecy", "NTN"),
    ]

    dfs = []
    for path, scenario, domain in files:
        try:
            dfs.append(load_qos(path, scenario=scenario, domain=domain))
        except Exception as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            return 2

    all_df = pd.concat(dfs, ignore_index=True)

    # Clean up initial “all zeros” lines
    if drop_allzero:
        all_df = drop_all_zero_rows(all_df)

    # Generate mean±std vs time plots for all metrics
    for metric in METRICS:
        out_path = args.out / f"meanstd_{metric.lower()}_vs_time_combined.png"
        save_mean_std_plot(
            all_df,
            metric,
            out_path,
            window=args.window,
            latency_unit=args.latency_unit,
            drop_zero_pdr=drop_zero_pdr,
        )

    print(f"[OK] Saved {len(METRICS)} plots in: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
