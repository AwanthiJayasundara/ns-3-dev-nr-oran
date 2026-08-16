#!/usr/bin/env python3
"""Evaluate ISAC-assisted spatial identification of underserved UE clusters.

Communication measurements decide *which* UEs are underserved. ISAC sensing
provides noisy spatial estimates of *where* those UEs are. This script fuses
the two, forms grid heatmaps, clusters underserved locations, and reports how
sensing-localisation error affects hotspot-cell F1 and cluster-centroid error.

The ns-3 position trace is treated as ground truth only for evaluation. A
Gaussian error model produces the position estimates available to the
controller; those estimates, not the ground truth, are used for detection.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import KMeans


DEFAULT_RUN = Path(
    "results/nr/tn-ntn/ml_uav_final/"
    "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_gru"
)


@dataclass(frozen=True)
class Grid:
    east_min: float = -3000.0
    east_max: float = 3000.0
    north_min: float = -1500.0
    north_max: float = 1500.0
    nx: int = 24
    ny: int = 12

    @property
    def east_edges(self) -> np.ndarray:
        return np.linspace(self.east_min, self.east_max, self.nx + 1)

    @property
    def north_edges(self) -> np.ndarray:
        return np.linspace(self.north_min, self.north_max, self.ny + 1)


def parse_positions(path: Path, group: str) -> pd.DataFrame:
    rows = []
    for line in path.read_text(errors="ignore").splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        rows.append(
            {
                "time": float(parts[0]),
                "trace_name": parts[1],
                "lat": float(parts[2]),
                "lon": float(parts[3]),
                "alt": float(parts[4]),
                "group": group,
            }
        )
    return pd.DataFrame(rows)


def to_local(df: pd.DataFrame, ref_lat: float, ref_lon: float) -> pd.DataFrame:
    out = df.copy()
    out["east_m"] = (out["lon"] - ref_lon) * 111320.0 * np.cos(np.deg2rad(ref_lat))
    out["north_m"] = (out["lat"] - ref_lat) * 111320.0
    return out


def node_mapping(ml: pd.DataFrame, positions: pd.DataFrame) -> dict[int, str]:
    """Use the scenario's contiguous UE-node allocation used by existing tools."""
    first_node_id = int(ml["ueId"].min())
    s1_names = sorted(
        positions.loc[positions["group"].eq("S1"), "trace_name"].unique(),
        key=lambda value: int(value.removeprefix("UE")),
    )
    s2_names = sorted(
        positions.loc[positions["group"].eq("S2"), "trace_name"].unique(),
        key=lambda value: int(value.removeprefix("UE_S2_")),
    )
    names = s1_names + s2_names
    return {first_node_id + index: name for index, name in enumerate(names)}


def nearest_positions(positions: pd.DataFrame, target_time: float) -> pd.DataFrame:
    candidates = positions.copy()
    candidates["gap"] = (candidates["time"] - target_time).abs()
    nearest_idx = candidates.groupby("trace_name")["gap"].idxmin()
    return candidates.loc[nearest_idx].set_index("trace_name")


def heatmap(points: np.ndarray, grid: Grid) -> np.ndarray:
    if len(points) == 0:
        return np.zeros((grid.ny, grid.nx), dtype=float)
    hist, _, _ = np.histogram2d(
        points[:, 1], points[:, 0], bins=[grid.north_edges, grid.east_edges]
    )
    return hist


def cell_f1(truth: np.ndarray, estimate: np.ndarray) -> tuple[float, float, float]:
    y_true = truth.ravel() > 0
    y_est = estimate.ravel() > 0
    tp = int(np.sum(y_true & y_est))
    fp = int(np.sum(~y_true & y_est))
    fn = int(np.sum(y_true & ~y_est))
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    return precision, recall, f1


def centroids(points: np.ndarray, clusters: int, seed: int) -> np.ndarray:
    k = min(clusters, len(points))
    if k == 0:
        return np.empty((0, 2))
    return KMeans(n_clusters=k, n_init=20, random_state=seed).fit(points).cluster_centers_


def matched_centroid_error(truth: np.ndarray, estimate: np.ndarray) -> float:
    if len(truth) == 0 or len(truth) != len(estimate):
        return float("nan")
    distances = np.linalg.norm(truth[:, None, :] - estimate[None, :, :], axis=2)
    truth_idx, estimate_idx = linear_sum_assignment(distances)
    return float(np.mean(distances[truth_idx, estimate_idx]))


def decision_rows(ml: pd.DataFrame) -> pd.DataFrame:
    return (
        ml.groupby(["time", "ueId"], as_index=False)
        .agg(serving_rsrp_dbm=("servingRsrp", "first"), serving_cell=("servingCell", "first"))
        .sort_values(["time", "ueId"])
    )


def points_at_time(
    decisions: pd.DataFrame,
    positions: pd.DataFrame,
    mapping: dict[int, str],
    time_s: float,
    rsrp_threshold: float,
) -> tuple[np.ndarray, np.ndarray]:
    at_time = decisions[decisions["time"].eq(time_s)].copy()
    at_time["trace_name"] = at_time["ueId"].map(mapping)
    at_time = at_time.dropna(subset=["trace_name"])
    nearest = nearest_positions(positions, time_s)
    at_time = at_time[at_time["trace_name"].isin(nearest.index)]
    all_points = nearest.loc[at_time["trace_name"], ["east_m", "north_m"]].to_numpy(float)
    underserved_names = at_time.loc[
        at_time["serving_rsrp_dbm"].lt(rsrp_threshold), "trace_name"
    ]
    underserved = nearest.loc[underserved_names, ["east_m", "north_m"]].to_numpy(float)
    return all_points, underserved


def evaluate(
    decisions: pd.DataFrame,
    positions: pd.DataFrame,
    mapping: dict[int, str],
    errors_m: list[float],
    rsrp_threshold: float,
    clusters: int,
    repeats: int,
    seed: int,
    grid: Grid,
) -> pd.DataFrame:
    rows = []
    rng = np.random.default_rng(seed)
    for time_s in sorted(decisions["time"].unique()):
        _, truth_points = points_at_time(
            decisions, positions, mapping, float(time_s), rsrp_threshold
        )
        if len(truth_points) < clusters:
            continue
        truth_heatmap = heatmap(truth_points, grid)
        truth_centroids = centroids(truth_points, clusters, seed)
        for error_m in errors_m:
            for repeat in range(repeats):
                sensed_points = truth_points + rng.normal(0.0, error_m, truth_points.shape)
                sensed_heatmap = heatmap(sensed_points, grid)
                precision, recall, f1 = cell_f1(truth_heatmap, sensed_heatmap)
                # Use the same deterministic clustering initialisation for the
                # oracle and sensed point sets. At sigma=0 the two estimates
                # must therefore be identical; any non-zero error is due to
                # sensing uncertainty rather than K-means initialisation.
                sensed_centroids = centroids(sensed_points, clusters, seed)
                rows.append(
                    {
                        "time_s": float(time_s),
                        "localisation_error_std_m": error_m,
                        "repeat": repeat,
                        "underserved_ues": len(truth_points),
                        "hotspot_cell_precision": precision,
                        "hotspot_cell_recall": recall,
                        "hotspot_cell_f1": f1,
                        "centroid_error_m": matched_centroid_error(
                            truth_centroids, sensed_centroids
                        ),
                    }
                )
    if not rows:
        raise RuntimeError("No decision time contained enough mapped underserved UEs")
    return pd.DataFrame(rows)


def plot_sensitivity(summary: pd.DataFrame, output: Path) -> None:
    fig, left = plt.subplots(figsize=(8.8, 4.8), constrained_layout=True)
    right = left.twinx()
    f1_plot = left.errorbar(
        summary["localisation_error_std_m"],
        summary["f1_median"],
        yerr=np.vstack(
            [
                summary["f1_median"] - summary["f1_q25"],
                summary["f1_q75"] - summary["f1_median"],
            ]
        ),
        color="#2F73BF",
        marker="o",
        linewidth=2.3,
        capsize=3,
        label="Hotspot-cell F1",
    )
    centroid_plot = right.errorbar(
        summary["localisation_error_std_m"],
        summary["centroid_error_median_m"],
        yerr=np.vstack(
            [
                summary["centroid_error_median_m"] - summary["centroid_error_q25_m"],
                summary["centroid_error_q75_m"] - summary["centroid_error_median_m"],
            ]
        ),
        color="#D9534F",
        marker="s",
        linewidth=2.3,
        capsize=3,
        label="Centroid error",
    )
    left.set_xlabel("ISAC localisation error standard deviation (m)")
    left.set_ylabel("Underserved-hotspot cell F1", color="#2F73BF")
    right.set_ylabel("Matched cluster-centroid error (m)", color="#D9534F")
    left.set_ylim(0, 1.05)
    left.grid(True, linestyle="--", alpha=0.35)
    left.legend(
        [f1_plot, centroid_plot],
        ["Hotspot-cell F1 (median/IQR)", "Centroid error (median/IQR)"],
        loc="center right",
    )
    fig.savefig(output, dpi=300)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)


def plot_example(
    all_points: np.ndarray,
    truth_points: np.ndarray,
    sensed_points: np.ndarray,
    truth_centroids: np.ndarray,
    sensed_centroids: np.ndarray,
    grid: Grid,
    time_s: float,
    error_m: float,
    output: Path,
) -> None:
    truth_map = heatmap(truth_points, grid)
    sensed_map = heatmap(sensed_points, grid)
    extent = [grid.east_min, grid.east_max, grid.north_min, grid.north_max]
    fig, axes = plt.subplots(1, 3, figsize=(14.2, 4.5), constrained_layout=True)
    axes[0].scatter(all_points[:, 0], all_points[:, 1], s=8, color="#B8B8B8", label="Other UE")
    axes[0].scatter(truth_points[:, 0], truth_points[:, 1], s=18, color="#C0392B", label="Underserved")
    axes[0].set_title("(a) Ground-truth service state")
    axes[0].legend(fontsize=8)
    axes[1].imshow(truth_map, origin="lower", extent=extent, aspect="auto", cmap="YlOrRd")
    axes[1].scatter(truth_centroids[:, 0], truth_centroids[:, 1], marker="*", s=150, color="#1F4E79")
    axes[1].set_title("(b) Ideal spatial reference")
    axes[2].imshow(sensed_map, origin="lower", extent=extent, aspect="auto", cmap="YlOrRd")
    axes[2].scatter(sensed_centroids[:, 0], sensed_centroids[:, 1], marker="*", s=150, color="#1F4E79")
    axes[2].set_title(f"(c) ISAC estimate ($\\sigma_s$={error_m:g} m)")
    for axis in axes:
        axis.set_xlabel("East (m)")
        axis.set_ylabel("North (m)")
        axis.set_xlim(grid.east_min, grid.east_max)
        axis.set_ylim(grid.north_min, grid.north_max)
    fig.suptitle(f"ISAC-assisted underserved-region identification at t={time_s:g} s")
    fig.savefig(output, dpi=300)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--rsrp-threshold-dbm", type=float, default=-120.0)
    parser.add_argument("--localisation-errors-m", default="0,5,10,20,30")
    parser.add_argument("--visual-error-m", type=float, default=20.0)
    parser.add_argument("--clusters", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("docs/figures/isac-underserved-detection")
    )
    args = parser.parse_args()
    errors_m = [float(value) for value in args.localisation_errors_m.split(",")]
    if args.visual_error_m not in errors_m:
        errors_m.append(args.visual_error_m)
        errors_m.sort()

    ml = pd.read_csv(args.run_dir / "ml-ho-dataset.csv")
    positions = pd.concat(
        [
            parse_positions(args.run_dir / "ues1-position-trace.tr", "S1"),
            parse_positions(args.run_dir / "ues2-position-trace.tr", "S2"),
        ],
        ignore_index=True,
    )
    ref_lat = float(positions["lat"].mean())
    ref_lon = float(positions["lon"].mean())
    positions = to_local(positions, ref_lat, ref_lon)
    mapping = node_mapping(ml, positions)
    decisions = decision_rows(ml)
    grid = Grid()

    samples = evaluate(
        decisions,
        positions,
        mapping,
        errors_m,
        args.rsrp_threshold_dbm,
        args.clusters,
        args.repeats,
        args.seed,
        grid,
    )
    summary = (
        samples.groupby("localisation_error_std_m", as_index=False)
        .agg(
            evaluated_cases=("hotspot_cell_f1", "size"),
            f1_mean=("hotspot_cell_f1", "mean"),
            f1_std=("hotspot_cell_f1", "std"),
            f1_median=("hotspot_cell_f1", "median"),
            f1_q25=("hotspot_cell_f1", lambda values: values.quantile(0.25)),
            f1_q75=("hotspot_cell_f1", lambda values: values.quantile(0.75)),
            centroid_error_mean_m=("centroid_error_m", "mean"),
            centroid_error_std_m=("centroid_error_m", "std"),
            centroid_error_median_m=("centroid_error_m", "median"),
            centroid_error_q25_m=("centroid_error_m", lambda values: values.quantile(0.25)),
            centroid_error_q75_m=("centroid_error_m", lambda values: values.quantile(0.75)),
        )
        .fillna(0.0)
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    samples.to_csv(args.output_dir / "isac_localisation_sensitivity_samples.csv", index=False)
    summary.to_csv(args.output_dir / "isac_localisation_sensitivity_summary.csv", index=False)
    plot_sensitivity(summary, args.output_dir / "isac_localisation_sensitivity.png")

    counts = decisions.assign(
        underserved=decisions["serving_rsrp_dbm"].lt(args.rsrp_threshold_dbm)
    ).groupby("time")["underserved"].sum()
    example_time = float(counts.idxmax())
    all_points, truth_points = points_at_time(
        decisions, positions, mapping, example_time, args.rsrp_threshold_dbm
    )
    rng = np.random.default_rng(args.seed)
    sensed_points = truth_points + rng.normal(0.0, args.visual_error_m, truth_points.shape)
    plot_example(
        all_points,
        truth_points,
        sensed_points,
        centroids(truth_points, args.clusters, args.seed),
        centroids(sensed_points, args.clusters, args.seed),
        grid,
        example_time,
        args.visual_error_m,
        args.output_dir / "isac_underserved_detection_example.png",
    )
    print(summary.to_string(index=False))
    print(f"[saved] {args.output_dir}")


if __name__ == "__main__":
    main()
