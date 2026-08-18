#!/usr/bin/env python3
"""Generate literature-informed synthetic ISAC--O-RAN result figures.

The generated values are deterministic synthetic drafting data, not ns-3 output.
They mimic the variability and presentation expected from ten independent seeds
while remaining bounded by this experiment's configured traffic and geometry.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import pandas as pd


STATUS = "SYNTHETIC_LITERATURE_INFORMED_NOT_NS3_RESULTS"
METHODS = ["Static UAV", "Oracle reactive", "ISAC reactive", "ISAC + RF predictive"]
COLORS = {
    "Static UAV": "#4D4D4D",
    "Oracle reactive": "#0072B2",
    "ISAC reactive": "#D55E00",
    "ISAC + RF predictive": "#009E73",
}
MARKERS = {
    "Static UAV": "o",
    "Oracle reactive": "s",
    "ISAC reactive": "^",
    "ISAC + RF predictive": "D",
}
LOADS = [80, 90, 100]


def apply_style() -> None:
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.labelsize": 9,
            "axes.titlesize": 10,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.dpi": 120,
        }
    )


def save_figure(fig: plt.Figure, output_dir: Path, stem: str) -> None:
    fig.savefig(output_dir / f"{stem}.png", dpi=300, bbox_inches="tight")
    fig.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)


def bounded_logit_sample(
    rng: np.random.Generator, mean: float, spread: float, count: int
) -> np.ndarray:
    mean = float(np.clip(mean, 1e-5, 1 - 1e-5))
    center = np.log(mean / (1 - mean))
    return 1 / (1 + np.exp(-(center + rng.normal(0, spread, count))))


def synthetic_method_data(rng: np.random.Generator) -> pd.DataFrame:
    """Create correlated, heteroscedastic ten-seed controller outcomes."""
    means = {
        "Static UAV": {
            "p5": [0.044, 0.032, 0.021],
            "pdr": [82.0, 77.0, 71.0],
            "outage": [0.30, 0.38, 0.47],
            "demand": [0.25, 0.32, 0.40],
        },
        "Oracle reactive": {
            "p5": [0.116, 0.099, 0.080],
            "pdr": [95.0, 92.5, 88.5],
            "outage": [0.12, 0.16, 0.22],
            "demand": [0.075, 0.105, 0.155],
        },
        "ISAC reactive": {
            "p5": [0.093, 0.079, 0.062],
            "pdr": [91.5, 88.0, 83.0],
            "outage": [0.18, 0.23, 0.30],
            "demand": [0.125, 0.165, 0.225],
        },
        "ISAC + RF predictive": {
            "p5": [0.107, 0.092, 0.074],
            "pdr": [94.0, 91.0, 87.0],
            "outage": [0.145, 0.185, 0.245],
            "demand": [0.095, 0.125, 0.175],
        },
    }
    rows: list[dict[str, float | int | str]] = []
    for method in METHODS:
        for load_index, total_ues in enumerate(LOADS):
            congestion = 1 + 0.16 * load_index
            for seed in range(1, 11):
                radio = rng.normal(0, 1)
                mobility = rng.normal(0, 1)
                p5 = means[method]["p5"][load_index] * np.exp(
                    0.095 * congestion * radio + rng.normal(0, 0.045)
                )
                pdr = means[method]["pdr"][load_index] + 1.35 * radio + rng.normal(0, 0.8)
                outage = bounded_logit_sample(
                    rng, means[method]["outage"][load_index], 0.13 * congestion, 1
                )[0]
                demand = bounded_logit_sample(
                    rng, means[method]["demand"][load_index], 0.15 * congestion, 1
                )[0]
                outage = float(np.clip(outage - 0.012 * radio + 0.008 * mobility, 0, 1))
                demand = float(np.clip(demand - 0.010 * radio + 0.010 * mobility, 0, 1))
                rows.append(
                    {
                        "DataStatus": STATUS,
                        "Method": method,
                        "TotalUes": total_ues,
                        "SyntheticSeed": seed,
                        "P5ThroughputMbps": float(np.clip(p5, 0, 0.2)),
                        "PdrPercent": float(np.clip(pdr, 0, 100)),
                        "OutageFraction": outage,
                        "PersistentDemandFraction": demand,
                    }
                )
    return pd.DataFrame(rows)


def aggregate_method_data(data: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, float | int | str]] = []
    metrics = ["P5ThroughputMbps", "PdrPercent", "OutageFraction", "PersistentDemandFraction"]
    for (method, load), group in data.groupby(["Method", "TotalUes"], sort=False):
        row: dict[str, float | int | str] = {
            "DataStatus": STATUS,
            "Method": method,
            "TotalUes": int(load),
            "NumSyntheticSeeds": len(group),
        }
        for metric in metrics:
            values = group[metric].to_numpy(float)
            row[f"{metric}Mean"] = float(np.mean(values))
            row[f"{metric}Ci95"] = float(1.96 * np.std(values, ddof=1) / np.sqrt(len(values)))
        rows.append(row)
    return pd.DataFrame(rows)


def plot_method_comparison(
    replicates: pd.DataFrame, summary: pd.DataFrame, output_dir: Path
) -> None:
    panels = [
        ("P5ThroughputMbps", "5th-percentile DL throughput (Mbit/s)"),
        ("OutageFraction", "Outage-time fraction"),
        ("PersistentDemandFraction", "Persistent repositioning-demand fraction"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(10.2, 3.2), constrained_layout=True)
    offsets = np.linspace(-2.7, 2.7, len(METHODS))
    for method_index, method in enumerate(METHODS):
        selected_summary = summary[summary["Method"] == method].sort_values("TotalUes")
        for axis, (metric, ylabel) in zip(axes, panels):
            x_mean = selected_summary["TotalUes"].to_numpy(float) + offsets[method_index]
            for load_index, load in enumerate(LOADS):
                values = replicates[
                    (replicates["Method"] == method) & (replicates["TotalUes"] == load)
                ][metric].to_numpy(float)
                jitter = np.linspace(-0.42, 0.42, len(values))
                axis.scatter(
                    x_mean[load_index] + jitter,
                    values,
                    s=9,
                    color=COLORS[method],
                    alpha=0.26,
                    linewidths=0,
                    zorder=1,
                )
            axis.errorbar(
                x_mean,
                selected_summary[f"{metric}Mean"],
                yerr=selected_summary[f"{metric}Ci95"],
                color=COLORS[method],
                marker=MARKERS[method],
                linewidth=1.35,
                markersize=4.5,
                capsize=2.5,
                label=method,
                zorder=3,
            )
            axis.set_xlabel("Total ground UEs")
            axis.set_ylabel(ylabel)
            axis.set_xticks(LOADS)
            axis.grid(True, axis="y", linestyle=":", alpha=0.35)
    axes[0].set_ylim(0, 0.14)
    axes[1].set_ylim(0, 0.55)
    axes[2].set_ylim(0, 0.48)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.08))
    save_figure(fig, output_dir, "fig_assumed_method_comparison")


def sensing_trials(rng: np.random.Generator) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Sample the paper model using a 30 dB effective-loss drafting pilot."""
    ranges = np.arange(100, 1401, 100)
    trial_rows: list[dict[str, float | int | str]] = []
    power_w = 1.0
    gain_linear = 10 ** (30 / 10)
    wavelength_m = 299_792_458 / 4.0e9
    noise_w = 1.380649e-23 * 290 * 20.0e6 * 10 ** (7 / 10)
    effective_loss_linear = 10 ** (30 / 10)
    for seed in range(1, 11):
        for range_m in ranges:
            for observation in range(40):
                rcs_db = rng.normal(0.0, 4.0)
                clutter_db = rng.normal(0.0, 2.5)
                rcs_m2 = 10 ** (rcs_db / 10)
                snr_linear = (
                    power_w * gain_linear**2 * wavelength_m**2 * rcs_m2
                    / ((4 * np.pi) ** 3 * range_m**4 * effective_loss_linear * noise_w)
                )
                snr_db = 10 * np.log10(snr_linear) - clutter_db
                probability = 1 / (1 + np.exp(-0.5 * (snr_db + 15)))
                detected = int(rng.random() < probability)
                sigma_m = float(np.clip(5 * 10 ** (-(snr_db + 10) / 20), 1, 50))
                radial_error_m = np.nan
                if detected:
                    radial_error_m = float(np.linalg.norm(rng.normal(0, sigma_m, 2)))
                trial_rows.append(
                    {
                        "DataStatus": STATUS,
                        "SyntheticSeed": seed,
                        "RangeM": range_m,
                        "Observation": observation,
                        "SensingSnrDb": snr_db,
                        "Detected": detected,
                        "RadialErrorM": radial_error_m,
                        "AssumedEffectiveLossDb": 30.0,
                    }
                )
    trials = pd.DataFrame(trial_rows)
    rows = []
    for range_m, group in trials.groupby("RangeM"):
        detections = group["Detected"].to_numpy(int)
        n = len(detections)
        p = detections.mean()
        z = 1.96
        denominator = 1 + z**2 / n
        center = (p + z**2 / (2 * n)) / denominator
        half = z * np.sqrt(p * (1 - p) / n + z**2 / (4 * n**2)) / denominator
        errors = group["RadialErrorM"].dropna().to_numpy(float)
        # Suppress an unstable conditional RMSE when almost no targets are
        # detected; the final analysis should report this as insufficient data.
        rmse = float(np.sqrt(np.mean(errors**2))) if len(errors) >= 20 else np.nan
        bootstrap = []
        if len(errors) >= 20:
            for _ in range(500):
                sample = rng.choice(errors, len(errors), replace=True)
                bootstrap.append(np.sqrt(np.mean(sample**2)))
        rows.append(
            {
                "DataStatus": STATUS,
                "RangeM": int(range_m),
                "NumTrials": n,
                "NumDetections": int(detections.sum()),
                "DetectionProbability": p,
                "DetectionCi95Low": max(0, center - half),
                "DetectionCi95High": min(1, center + half),
                "LocalisationRmseM": rmse,
                "RmseCi95Low": float(np.quantile(bootstrap, 0.025)) if bootstrap else np.nan,
                "RmseCi95High": float(np.quantile(bootstrap, 0.975)) if bootstrap else np.nan,
                "AssumedEffectiveLossDb": 30.0,
            }
        )
    return trials, pd.DataFrame(rows)


def plot_sensing(data: pd.DataFrame, output_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0), constrained_layout=True)
    x = data["RangeM"].to_numpy(float)
    detection = data["DetectionProbability"].to_numpy(float)
    axes[0].errorbar(
        x,
        detection,
        yerr=np.vstack(
            [detection - data["DetectionCi95Low"], data["DetectionCi95High"] - detection]
        ),
        fmt="o-",
        color="#0072B2",
        markersize=3.5,
        linewidth=1.4,
        capsize=2,
    )
    axes[0].axhline(0.5, color="0.45", linestyle="--", linewidth=0.9)
    axes[0].set_ylabel("Empirical detection probability")
    axes[0].set_ylim(-0.03, 1.03)
    rmse = data["LocalisationRmseM"].to_numpy(float)
    axes[1].errorbar(
        x,
        rmse,
        yerr=np.vstack([rmse - data["RmseCi95Low"], data["RmseCi95High"] - rmse]),
        fmt="s-",
        color="#D55E00",
        markersize=3.5,
        linewidth=1.4,
        capsize=2,
    )
    axes[1].set_ylabel("Conditional localisation RMSE (m)")
    for axis in axes:
        axis.set_xlabel("Sensing range (m)")
        axis.grid(True, linestyle=":", alpha=0.35)
        axis.set_xlim(60, 1440)
    save_figure(fig, output_dir, "fig_assumed_isac_sensing")


def binary_metrics(y: np.ndarray, pred: np.ndarray) -> tuple[float, float, float]:
    tp = int(np.sum((y == 1) & (pred == 1)))
    fp = int(np.sum((y == 0) & (pred == 1)))
    fn = int(np.sum((y == 1) & (pred == 0)))
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return precision, recall, f1


def precision_recall_points(y: np.ndarray, score: np.ndarray) -> pd.DataFrame:
    rows = []
    for threshold in np.linspace(0, 1, 201):
        precision, recall, f1 = binary_metrics(y, score >= threshold)
        rows.append({"Threshold": threshold, "Precision": precision, "Recall": recall, "F1": f1})
    return pd.DataFrame(rows)


def synthetic_rf_data(rng: np.random.Generator) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    score_rows = []
    metric_rows = []
    curve_rows = []
    distributions = {
        2: ((7.0, 2.1), (1.7, 7.2)),
        5: ((5.2, 2.6), (2.0, 6.2)),
        10: ((4.4, 2.8), (2.0, 5.3)),
    }
    thresholds = {2: 0.55, 5: 0.50, 10: 0.46}
    for horizon, (positive_beta, negative_beta) in distributions.items():
        pooled_y = []
        pooled_score = []
        for seed in range(1, 11):
            n = 700
            prevalence = float(np.clip(rng.normal(0.16, 0.018), 0.10, 0.23))
            y = (rng.random(n) < prevalence).astype(int)
            score = np.empty(n)
            score[y == 1] = rng.beta(*positive_beta, size=np.sum(y == 1))
            score[y == 0] = rng.beta(*negative_beta, size=np.sum(y == 0))
            score = np.clip(score + rng.normal(0, 0.025), 0, 1)
            pred = score >= thresholds[horizon]
            precision, recall, f1 = binary_metrics(y, pred)
            metric_rows.append(
                {
                    "DataStatus": STATUS,
                    "PredictionHorizonS": horizon,
                    "SyntheticSeed": seed,
                    "Threshold": thresholds[horizon],
                    "Precision": precision,
                    "Recall": recall,
                    "F1": f1,
                    "PositivePrevalence": y.mean(),
                }
            )
            for index in range(n):
                score_rows.append(
                    {
                        "DataStatus": STATUS,
                        "PredictionHorizonS": horizon,
                        "SyntheticSeed": seed,
                        "CellSample": index,
                        "TrueLabel": int(y[index]),
                        "PredictedProbability": float(score[index]),
                    }
                )
            pooled_y.append(y)
            pooled_score.append(score)
        curve = precision_recall_points(np.concatenate(pooled_y), np.concatenate(pooled_score))
        curve["DataStatus"] = STATUS
        curve["PredictionHorizonS"] = horizon
        curve_rows.extend(curve.to_dict("records"))
    return pd.DataFrame(score_rows), pd.DataFrame(metric_rows), pd.DataFrame(curve_rows)


def plot_rf(metrics: pd.DataFrame, curves: pd.DataFrame, output_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0), constrained_layout=True)
    horizon_colors = {2: "#0072B2", 5: "#E69F00", 10: "#CC79A7"}
    for horizon in [2, 5, 10]:
        curve = curves[curves["PredictionHorizonS"] == horizon].sort_values("Recall")
        axes[0].plot(
            curve["Recall"], curve["Precision"], color=horizon_colors[horizon],
            linewidth=1.5, label=f"{horizon} s"
        )
    axes[0].set_xlabel("Recall")
    axes[0].set_ylabel("Precision")
    axes[0].set_xlim(0, 1.01)
    axes[0].set_ylim(0, 1.01)
    axes[0].legend(title="Horizon", frameon=False)
    metric_names = ["Precision", "Recall", "F1"]
    metric_colors = ["#0072B2", "#D55E00", "#009E73"]
    offsets = [-0.22, 0, 0.22]
    for metric, color, offset in zip(metric_names, metric_colors, offsets):
        means = metrics.groupby("PredictionHorizonS")[metric].mean().reindex([2, 5, 10])
        cis = metrics.groupby("PredictionHorizonS")[metric].std(ddof=1).reindex([2, 5, 10]) / np.sqrt(10) * 1.96
        axes[1].errorbar(
            np.arange(3) + offset, means, yerr=cis, fmt="o", color=color,
            markersize=4, capsize=2.5, label=metric
        )
    axes[1].set_xticks(np.arange(3), ["2", "5", "10"])
    axes[1].set_xlabel("Prediction horizon (s)")
    axes[1].set_ylabel("Score at validation threshold")
    axes[1].set_ylim(0.45, 1.0)
    axes[1].legend(frameon=False)
    for axis in axes:
        axis.grid(True, linestyle=":", alpha=0.35)
    save_figure(fig, output_dir, "fig_assumed_rf_prediction")


def synthetic_controller_dynamics(rng: np.random.Generator) -> pd.DataFrame:
    times = np.arange(0, 91, 2)
    rows = []
    for method in METHODS:
        for seed in range(1, 11):
            phase = rng.normal(0, 1.0)
            seed_level = rng.normal(0, 0.012)
            for time_s in times:
                # The first assessment occurs at 12 s. A connected weak UE can
                # become MOVE_REQUIRED only after the 6 s persistence timer;
                # initially UNKNOWN UEs can take roughly another 6 s.
                if time_s < 18:
                    rise = 0.0
                else:
                    rise = 0.31 / (1 + np.exp(-(time_s - 20.5 - phase) / 2.2))
                if method == "Static UAV":
                    relief = 0.0
                elif method == "Oracle reactive":
                    relief = 0.235 / (1 + np.exp(-(time_s - 39 - phase) / 5.5))
                elif method == "ISAC reactive":
                    relief = 0.185 / (1 + np.exp(-(time_s - 44 - phase) / 6.5))
                else:
                    # A predictive controller may start travelling from its
                    # future label before reactive demand fully develops.
                    relief = 0.215 / (1 + np.exp(-(time_s - 35 - phase) / 6.0))
                if time_s < 18:
                    value = 0.0
                else:
                    fluctuation = 0.009 * np.sin(time_s / 5 + seed) + rng.normal(0, 0.004)
                    value = np.clip(rise - relief + seed_level + fluctuation, 0, 1)
                rows.append(
                    {
                        "DataStatus": STATUS,
                        "Method": method,
                        "SyntheticSeed": seed,
                        "TimeS": time_s,
                        "PersistentDemandFraction": value,
                    }
                )
    return pd.DataFrame(rows)


def plot_controller_dynamics(data: pd.DataFrame, output_dir: Path) -> None:
    fig, axis = plt.subplots(figsize=(7.2, 3.3), constrained_layout=True)
    for method in METHODS:
        selected = data[data["Method"] == method]
        grouped = selected.groupby("TimeS")["PersistentDemandFraction"]
        mean = grouped.mean()
        ci = 1.96 * grouped.std(ddof=1) / np.sqrt(10)
        axis.plot(mean.index, mean, color=COLORS[method], linewidth=1.55, label=method)
        axis.fill_between(mean.index, mean - ci, mean + ci, color=COLORS[method], alpha=0.13)
    axis.axvline(12, color="0.35", linestyle="--", linewidth=0.9, label="Controller start")
    axis.set_xlabel("Simulation time (s)")
    axis.set_ylabel("Persistent repositioning-demand fraction")
    axis.set_xlim(0, 90)
    axis.set_ylim(0, 0.36)
    axis.grid(True, linestyle=":", alpha=0.35)
    axis.legend(frameon=False, ncol=2)
    save_figure(fig, output_dir, "fig_assumed_controller_dynamics")


def spatial_data(
    rng: np.random.Generator,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    central = rng.normal([0, 0], [115, 105], size=(60, 2))
    outer_centers = np.array([[-540.0, 420.0], [500.0, 360.0], [360.0, -525.0]])
    outer_sizes = [8, 12, 10]
    outer = np.vstack(
        [center + rng.multivariate_normal([0, 0], [[4200, 900], [900, 2900]], size=size)
         for center, size in zip(outer_centers, outer_sizes)]
    )
    all_ues = np.vstack([central, outer])
    demand_outer_indices = np.array([0, 1, 3, 4, 8, 9, 10, 12, 14, 15, 19, 20, 22, 25, 27, 28])
    demand_points = outer[demand_outer_indices]
    detected = rng.random(len(demand_points)) < 0.84
    fused_points = demand_points[detected] + rng.normal(0, [11, 14], size=(detected.sum(), 2))
    point_rows = []
    demand_global = set((60 + demand_outer_indices).tolist())
    for index, point in enumerate(all_ues):
        point_rows.append(
            {
                "DataStatus": STATUS,
                "PointType": "PersistentDemand" if index in demand_global else "OtherUE",
                "Index": index,
                "EastM": point[0],
                "NorthM": point[1],
            }
        )
    for index, point in enumerate(fused_points):
        point_rows.append(
            {
                "DataStatus": STATUS,
                "PointType": "FusedDetection",
                "Index": index,
                "EastM": point[0],
                "NorthM": point[1],
            }
        )
    grid_values = []
    grid_centers = np.arange(-950, 951, 100)
    predicted_centers = outer_centers + np.array([[35, -18], [42, 25], [-25, 38]])
    for east in grid_centers:
        for north in grid_centers:
            probability = 0.025
            for center, amplitude, scale in zip(predicted_centers, [0.83, 0.91, 0.79], [155, 170, 150]):
                probability += amplitude * np.exp(
                    -np.sum((np.array([east, north]) - center) ** 2) / (2 * scale**2)
                )
            probability = float(np.clip(probability + rng.normal(0, 0.018), 0, 0.98))
            grid_values.append(
                {
                    "DataStatus": STATUS,
                    "EastM": east,
                    "NorthM": north,
                    "PredictedProbability": probability,
                }
            )
    starts = np.array([[-85.0, 55.0], [35.0, 90.0], [65.0, -75.0]])
    routes = []
    for uav_index, (start, target) in enumerate(zip(starts, predicted_centers)):
        distance = np.linalg.norm(target - start)
        steps = int(np.ceil(distance / 30)) + 1
        for step, weight in enumerate(np.linspace(0, 1, steps)):
            curve = np.array([18 * np.sin(np.pi * weight + uav_index), 12 * np.sin(2 * np.pi * weight)])
            point = start * (1 - weight) + target * weight + curve
            routes.append(
                {
                    "DataStatus": STATUS,
                    "UavIndex": uav_index + 1,
                    "TimeS": 12 + 2 * step,
                    "EastM": point[0],
                    "NorthM": point[1],
                }
            )
    centroid_data = pd.DataFrame(
        {
            "DataStatus": STATUS,
            "CentroidIndex": [1, 2, 3],
            "EastM": predicted_centers[:, 0],
            "NorthM": predicted_centers[:, 1],
            "PredictedProbability": [0.83, 0.91, 0.79],
        }
    )
    return pd.DataFrame(point_rows), pd.DataFrame(routes), centroid_data, pd.DataFrame(grid_values)


def plot_spatial(
    points: pd.DataFrame,
    routes: pd.DataFrame,
    centroids: pd.DataFrame,
    grid: pd.DataFrame,
    output_dir: Path,
) -> None:
    fig, axis = plt.subplots(figsize=(6.0, 5.0), constrained_layout=True)
    pivot = grid.pivot(index="NorthM", columns="EastM", values="PredictedProbability")
    image = axis.imshow(
        pivot.to_numpy(), origin="lower", extent=[-1000, 1000, -1000, 1000],
        cmap="YlOrRd", vmin=0, vmax=1, alpha=0.46, interpolation="bilinear", aspect="equal"
    )
    all_ues = points[points["PointType"].isin(["OtherUE", "PersistentDemand"])]
    demand = points[points["PointType"] == "PersistentDemand"]
    fused = points[points["PointType"] == "FusedDetection"]
    axis.scatter(
        all_ues["EastM"], all_ues["NorthM"], s=9, color="0.42", alpha=0.5,
        label="All UE positions (evaluation)"
    )
    axis.scatter(
        demand["EastM"], demand["NorthM"], marker="x", color="#B2182B", s=24,
        label=r"Movement targets ($I_u^{\mathrm{move}}=1$)"
    )
    axis.scatter(
        fused["EastM"], fused["NorthM"], facecolors="none", edgecolors="#2166AC",
        linewidths=0.8, s=28, label="Fused estimates of detected targets"
    )
    for uav_index, route in routes.groupby("UavIndex"):
        axis.plot(route["EastM"], route["NorthM"], linewidth=1.6, label=f"UAV {uav_index} path")
        axis.scatter(route.iloc[0]["EastM"], route.iloc[0]["NorthM"], marker="^", color="black", s=32, zorder=5)
    axis.scatter(
        centroids["EastM"], centroids["NorthM"], marker="*", color="#009E73",
        edgecolor="black", linewidth=0.5, s=105, label="Predicted centroids", zorder=6
    )
    axis.scatter(
        [-100, 100, -100, 100], [100, 100, -100, -100], marker="s",
        color="black", s=25, label="TN gNBs", zorder=5
    )
    axis.add_patch(Rectangle((-1000, -1000), 2000, 2000, fill=False, color="0.2", linewidth=0.8))
    axis.set_xlim(-1020, 1020)
    axis.set_ylim(-1020, 1020)
    axis.set_xlabel("East offset (m)")
    axis.set_ylabel("North offset (m)")
    axis.legend(loc="upper left", fontsize=6.8, framealpha=0.9, ncol=2)
    colorbar = fig.colorbar(image, ax=axis, fraction=0.046, pad=0.03)
    colorbar.set_label("Predicted demand probability")
    save_figure(fig, output_dir, "fig_assumed_spatial_control")


def write_notice(output_dir: Path) -> None:
    (output_dir / "README-ASSUMED-DATA.txt").write_text(
        "ALL FILES IN THIS DIRECTORY ARE SYNTHETIC DRAFTING ARTIFACTS.\n"
        "They are literature-informed but were not produced by ns-3 or a trained RF.\n"
        "No visible watermark is added; provenance is retained in CSV DataStatus fields.\n"
        "The sensing pilot assumes 30 dB effective aggregate loss because the current\n"
        "configured L_s=1 produces near-perfect sensing throughout the compact area.\n"
        "Replace all values with multi-seed ns-3/RF results before making claims.\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("results/plots/illustrative-assumed")
    )
    parser.add_argument("--assumption-seed", type=int, default=20260817)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    apply_style()
    rng = np.random.default_rng(args.assumption_seed)

    method_replicates = synthetic_method_data(rng)
    method_summary = aggregate_method_data(method_replicates)
    sensing_trials_data, sensing_summary = sensing_trials(rng)
    rf_scores, rf_metrics, rf_curves = synthetic_rf_data(rng)
    dynamics = synthetic_controller_dynamics(rng)
    points, routes, centroids, grid = spatial_data(rng)

    method_replicates.to_csv(args.output_dir / "assumed_method_replicates.csv", index=False)
    method_summary.to_csv(args.output_dir / "assumed_method_summary.csv", index=False)
    sensing_trials_data.to_csv(args.output_dir / "assumed_sensing_trials.csv", index=False)
    sensing_summary.to_csv(args.output_dir / "assumed_sensing_by_range.csv", index=False)
    rf_scores.to_csv(args.output_dir / "assumed_rf_cell_scores.csv", index=False)
    rf_metrics.to_csv(args.output_dir / "assumed_rf_by_horizon.csv", index=False)
    rf_curves.to_csv(args.output_dir / "assumed_rf_pr_curves.csv", index=False)
    dynamics.to_csv(args.output_dir / "assumed_controller_dynamics.csv", index=False)
    points.to_csv(args.output_dir / "assumed_spatial_points.csv", index=False)
    routes.to_csv(args.output_dir / "assumed_uav_routes.csv", index=False)
    centroids.to_csv(args.output_dir / "assumed_predictive_centroids.csv", index=False)
    grid.to_csv(args.output_dir / "assumed_probability_grid.csv", index=False)

    plot_method_comparison(method_replicates, method_summary, args.output_dir)
    plot_sensing(sensing_summary, args.output_dir)
    plot_rf(rf_metrics, rf_curves, args.output_dir)
    plot_controller_dynamics(dynamics, args.output_dir)
    plot_spatial(points, routes, centroids, grid, args.output_dir)
    write_notice(args.output_dir)
    print(f"Synthetic literature-informed package written to {args.output_dir}")


if __name__ == "__main__":
    main()
