#!/usr/bin/env python3
"""Plot measured coverage from available 120 s ML UAV repositioning runs.

Uses local measured data only:
  - TN baseline / non-predictive reference, available to 120 s
  - GRU-predictive TN+NTN, available to 120 s
  - CNN-learning TN+NTN, available locally to about 72 s

Reactive K-means is intentionally omitted here if the full 120 s result is not
available yet.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


RUNS = {
    "TN baseline": Path(
        "results/nr/tn-ntn/"
        "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2"
    ),
    "GRU-predictive TN+NTN": Path(
        "results/nr/tn-ntn/ml_uav_final/"
        "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_gru"
    ),
    "CNN-learning TN+NTN": Path(
        "results/nr/tn-ntn/ml_uav_final/"
        "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_cnn-120"
    ),
}

OUT_DIR = Path("docs/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def load_serving_coverage(
    run_dir: Path,
    label: str,
    rsrp_threshold: float = -120.0,
    tn_only: bool = False,
    num_tn_cells: int = 8,
) -> pd.DataFrame:
    dataset = run_dir / "ml-ho-dataset.csv"
    if not dataset.exists() or dataset.stat().st_size == 0:
        return pd.DataFrame(columns=["time", "coverage_pct", "observed_ues", "scheme"])

    df = pd.read_csv(dataset, usecols=["time", "ueId", "servingCell", "servingRsrp"])
    if df.empty:
        return pd.DataFrame(columns=["time", "coverage_pct", "observed_ues", "scheme"])

    serving = df.drop_duplicates(["time", "ueId"]).copy()
    if tn_only:
        serving["covered"] = serving["servingCell"].between(1, num_tn_cells) & serving["servingRsrp"].ge(rsrp_threshold)
    else:
        serving["covered"] = serving["servingRsrp"].ge(rsrp_threshold)
    coverage = (
        serving.groupby("time", as_index=False)
        .agg(
            coverage_pct=("covered", lambda x: 100.0 * float(x.mean())),
            observed_ues=("ueId", "nunique"),
        )
    )
    coverage["scheme"] = label
    return coverage


def main() -> None:
    style = Path("latex_style.mplstyle")
    if style.exists():
        plt.style.use(str(style))

    frames = []
    for label, run_dir in RUNS.items():
        cov = load_serving_coverage(run_dir, label, tn_only=(label == "TN baseline"))
        if not cov.empty:
            frames.append(cov)
    if not frames:
        raise SystemExit("No measured coverage data found.")

    data = pd.concat(frames, ignore_index=True)
    data.to_csv(OUT_DIR / "uav_measured_coverage_120_tn_gru_cnn_partial.csv", index=False)

    fig, ax = plt.subplots(figsize=(9.6, 5.2), constrained_layout=False)
    fig.subplots_adjust(left=0.13, right=0.985, bottom=0.18, top=0.96)

    styles = {
        "TN baseline": dict(color="#1F77B4", linestyle="--", marker="o", linewidth=2.6),
        "GRU-predictive TN+NTN": dict(color="#2CA02C", linestyle="-", marker="s", linewidth=2.8),
        "CNN-learning TN+NTN": dict(color="#9467BD", linestyle="-.", marker="^", linewidth=2.8),
    }

    for label, kwargs in styles.items():
        sub = data[data["scheme"].eq(label)].sort_values("time")
        if sub.empty:
            continue
        ax.plot(sub["time"], sub["coverage_pct"], label=label, markevery=3, **kwargs)
        ax.scatter([sub["time"].iloc[-1]], [sub["coverage_pct"].iloc[-1]],
                   color=kwargs["color"], s=95, zorder=5)

    ax.axvline(6, color="#1F77B4", linestyle=":", linewidth=2.1, alpha=0.85, label="S2 attach starts")
    ax.axvline(120, color="#1F77B4", linestyle="-.", linewidth=2.1, alpha=0.9, label="Target 120 s")
    ax.set_xlabel("Time (s)", fontsize=24)
    ax.set_ylabel("Coverage (\\%)", fontsize=24)
    ax.set_xlim(0, 125)
    ax.set_ylim(0, 100)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.tick_params(axis="both", labelsize=17)
    ax.legend(
        loc="lower right",
        frameon=True,
        fontsize=12,
        framealpha=0.88,
        borderpad=0.45,
        labelspacing=0.35,
        handlelength=2.0,
    )

    for ext in ("png", "pdf"):
        fig.savefig(OUT_DIR / f"uav_measured_coverage_120_tn_gru_cnn_partial.{ext}", dpi=300)
    plt.close(fig)

    print("[saved]", OUT_DIR / "uav_measured_coverage_120_tn_gru_cnn_partial.png")
    print("[saved]", OUT_DIR / "uav_measured_coverage_120_tn_gru_cnn_partial.pdf")
    print("[saved]", OUT_DIR / "uav_measured_coverage_120_tn_gru_cnn_partial.csv")
    summary = data.sort_values("time").groupby("scheme").tail(1)
    print(summary[["scheme", "time", "coverage_pct", "observed_ues"]].to_string(index=False))


if __name__ == "__main__":
    main()
