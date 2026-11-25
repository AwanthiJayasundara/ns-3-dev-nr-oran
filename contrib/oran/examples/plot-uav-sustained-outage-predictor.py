#!/usr/bin/env python3
"""Plot the frozen v1 sustained-donor-loss predictor evaluation.

The figure is intentionally limited to offline, seed-disjoint forecasting
evidence.  It does not read raw ns-3 traces and does not depict or imply a
closed-loop network-policy gain.  Inputs are verified against the versioned v1
artifact manifest before plotting.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.metrics import (
    average_precision_score,
    confusion_matrix,
    f1_score,
    precision_recall_curve,
)


FIGURE_BASENAME = "sustained_outage_predictor_grouped_evaluation"
INPUT_FILES = [
    "frozen_protocol_v1.json",
    "grouped_oof_predictions_v1.csv",
    "grouped_seed_metrics_v1.csv",
    "grouped_oof_metrics_v1.json",
    "input_output_schema_v1.json",
]
LABEL_NAMES = {
    0: "No sustained\nloss",
    1: "Sustained\nloss",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def verify_inputs(artifact_dir: Path) -> dict[str, str]:
    manifest_path = artifact_dir / "artifact_manifest_v1.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Missing versioned artifact manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    recorded = manifest.get("files", {})
    hashes: dict[str, str] = {}
    for filename in INPUT_FILES:
        path = artifact_dir / filename
        if not path.is_file():
            raise FileNotFoundError(f"Missing required v1 plotting input: {path}")
        actual = sha256_file(path)
        expected = recorded.get(filename, {}).get("sha256")
        if expected is None:
            raise ValueError(f"No recorded SHA-256 for {filename} in {manifest_path}")
        if actual != expected:
            raise ValueError(
                f"Input hash mismatch for {filename}: actual {actual}, expected {expected}"
            )
        hashes[filename] = actual
    hashes[manifest_path.name] = sha256_file(manifest_path)
    return hashes


def load_and_validate(
    artifact_dir: Path,
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any], dict[str, Any]]:
    protocol = json.loads(
        (artifact_dir / "frozen_protocol_v1.json").read_text(encoding="utf-8")
    )
    if protocol.get("protocol_version") != "uav-sustained-outage-v1":
        raise ValueError("This plotter accepts only uav-sustained-outage-v1 artifacts")
    expected_target = (
        "1 iff RawXhaulState is UNREACHABLE at t+2 s, t+4 s, and t+6 s"
    )
    if protocol.get("target_definition") != expected_target:
        raise ValueError(f"Unexpected target definition: {protocol.get('target_definition')}")

    oof = pd.read_csv(artifact_dir / "grouped_oof_predictions_v1.csv")
    seeds = pd.read_csv(artifact_dir / "grouped_seed_metrics_v1.csv")
    metrics = json.loads(
        (artifact_dir / "grouped_oof_metrics_v1.json").read_text(encoding="utf-8")
    )
    pooled = metrics["pooled_oof_metrics"]

    required_oof = {
        "RowId",
        "SeedGroup",
        "true_label_id",
        "predicted_label_id",
        "sustained_loss_probability",
    }
    if not required_oof.issubset(oof.columns):
        raise ValueError(f"OOF input is missing columns: {required_oof.difference(oof.columns)}")
    if len(oof) != int(pooled["samples"]) or not oof["RowId"].is_unique:
        raise ValueError("OOF rows must be unique and equal the recorded pooled sample count")
    if not oof["true_label_id"].isin([0, 1]).all():
        raise ValueError("OOF true labels must be binary 0/1")
    if not oof["predicted_label_id"].isin([0, 1]).all():
        raise ValueError("OOF predictions must be binary 0/1")
    probability = oof["sustained_loss_probability"].to_numpy(dtype=float)
    if not np.isfinite(probability).all() or np.any((probability < 0.0) | (probability > 1.0)):
        raise ValueError("OOF probabilities must be finite and lie in [0,1]")

    expected_seeds = [f"seed{seed}" for seed in range(1, 6)]
    if sorted(seeds["held_out_seed"].astype(str).tolist()) != expected_seeds:
        raise ValueError("Per-seed input must contain exactly seed1 through seed5")

    truth = oof["true_label_id"].to_numpy(dtype=int)
    prediction = oof["predicted_label_id"].to_numpy(dtype=int)
    matrix = confusion_matrix(truth, prediction, labels=[0, 1])
    recorded_matrix = np.asarray(pooled["confusion_matrix_label_order_0_1"], dtype=int)
    if not np.array_equal(matrix, recorded_matrix):
        raise ValueError(f"Confusion matrix mismatch: {matrix} != {recorded_matrix}")
    actual_ap = average_precision_score(truth, probability)
    if not np.isclose(actual_ap, float(pooled["average_precision"]), atol=1e-12):
        raise ValueError("Average precision does not reproduce the recorded pooled metric")

    for row in seeds.itertuples(index=False):
        mask = oof["SeedGroup"].astype(str) == str(row.held_out_seed)
        actual_f1 = f1_score(
            truth[mask], prediction[mask], labels=[0, 1], average="macro"
        )
        if not np.isclose(actual_f1, float(row.macro_f1), atol=1e-12):
            raise ValueError(f"Macro-F1 mismatch for {row.held_out_seed}")

    return oof, seeds.sort_values("held_out_seed"), metrics, protocol


def figure_data_rows(
    oof: pd.DataFrame, seeds: pd.DataFrame, pooled: dict[str, Any]
) -> tuple[list[dict[str, Any]], np.ndarray, np.ndarray, np.ndarray]:
    truth = oof["true_label_id"].to_numpy(dtype=int)
    prediction = oof["predicted_label_id"].to_numpy(dtype=int)
    probability = oof["sustained_loss_probability"].to_numpy(dtype=float)
    matrix = confusion_matrix(truth, prediction, labels=[0, 1])
    row_fraction = matrix / matrix.sum(axis=1, keepdims=True)
    precision, recall, thresholds = precision_recall_curve(truth, probability)

    rows: list[dict[str, Any]] = []
    for position, row in enumerate(seeds.itertuples(index=False), start=1):
        rows.append(
            {
                "panel": "seed_macro_f1",
                "series": "held_out_seed",
                "seed": row.held_out_seed,
                "x": position,
                "value": row.macro_f1,
                "total_support": row.test_samples,
                "positive_support": row.positive_support,
            }
        )
    rows.extend(
        [
            {
                "panel": "seed_macro_f1",
                "series": "mean_seed_macro_f1",
                "value": pooled["mean_seed_macro_f1"],
            },
            {
                "panel": "seed_macro_f1",
                "series": "pooled_macro_f1",
                "value": pooled["macro_f1"],
            },
        ]
    )

    for actual in (0, 1):
        for predicted in (0, 1):
            rows.append(
                {
                    "panel": "pooled_confusion",
                    "series": "cell",
                    "actual_label_id": actual,
                    "actual_label": LABEL_NAMES[actual].replace("\n", " "),
                    "predicted_label_id": predicted,
                    "predicted_label": LABEL_NAMES[predicted].replace("\n", " "),
                    "count": int(matrix[actual, predicted]),
                    "row_fraction": float(row_fraction[actual, predicted]),
                    "total_support": int(matrix[actual].sum()),
                }
            )

    for index, (recall_value, precision_value) in enumerate(zip(recall, precision)):
        rows.append(
            {
                "panel": "precision_recall",
                "series": "oof_curve",
                "point_index": index,
                "recall": float(recall_value),
                "precision": float(precision_value),
                "threshold": (
                    float(thresholds[index]) if index < len(thresholds) else ""
                ),
            }
        )
    rows.extend(
        [
            {
                "panel": "precision_recall",
                "series": "prevalence_baseline",
                "value": float(np.mean(truth == 1)),
            },
            {
                "panel": "precision_recall",
                "series": "threshold_0.5_operating_point",
                "recall": pooled["positive_recall"],
                "precision": pooled["positive_precision"],
                "threshold": 0.5,
            },
            {
                "panel": "precision_recall",
                "series": "average_precision",
                "value": pooled["average_precision"],
            },
        ]
    )
    return rows, matrix, precision, recall


def write_figure_data(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "panel",
        "series",
        "seed",
        "point_index",
        "x",
        "value",
        "actual_label_id",
        "actual_label",
        "predicted_label_id",
        "predicted_label",
        "recall",
        "precision",
        "threshold",
        "count",
        "row_fraction",
        "total_support",
        "positive_support",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9.0,
            "axes.titlesize": 10.0,
            "axes.labelsize": 9.0,
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "legend.fontsize": 7.7,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def make_figure(
    oof: pd.DataFrame,
    seeds: pd.DataFrame,
    pooled: dict[str, Any],
    matrix: np.ndarray,
    precision: np.ndarray,
    recall: np.ndarray,
) -> plt.Figure:
    configure_style()
    blue = "#0072B2"
    orange = "#D55E00"
    green = "#009E73"
    gray = "#666666"

    figure, axes = plt.subplots(1, 3, figsize=(12.2, 4.15))
    figure.subplots_adjust(left=0.065, right=0.965, bottom=0.19, top=0.76, wspace=0.48)

    # Panel A: one independent test group per bar.
    axis = axes[0]
    positions = np.arange(len(seeds))
    values = seeds["macro_f1"].to_numpy(dtype=float)
    bars = axis.bar(
        positions,
        values,
        width=0.68,
        color=blue,
        edgecolor="white",
        linewidth=0.7,
        zorder=3,
    )
    axis.axhline(
        float(pooled["mean_seed_macro_f1"]),
        color=orange,
        linewidth=1.6,
        linestyle="--",
        label=f"mean seed = {pooled['mean_seed_macro_f1']:.3f}",
        zorder=4,
    )
    axis.axhline(
        float(pooled["macro_f1"]),
        color=green,
        linewidth=1.5,
        linestyle=":",
        label=f"pooled = {pooled['macro_f1']:.3f}",
        zorder=4,
    )
    for bar, value in zip(bars, values):
        axis.text(
            bar.get_x() + bar.get_width() / 2,
            value + 0.025,
            f"{value:.3f}",
            ha="center",
            va="bottom",
            fontsize=7.7,
        )
    axis.set_xticks(positions, seeds["held_out_seed"].str.replace("seed", "Seed "))
    axis.set_ylim(0.0, 1.06)
    axis.set_yticks(np.linspace(0.0, 1.0, 6))
    axis.set_ylabel("Macro-F1")
    axis.set_title("(a) Leave-one-seed-out performance", loc="left", fontweight="bold")
    axis.grid(axis="y", color="#d9d9d9", linewidth=0.7, zorder=0)
    axis.legend(loc="lower left", frameon=False)

    # Panel B: pooled counts with row-normalized percentages.
    axis = axes[1]
    row_fraction = matrix / matrix.sum(axis=1, keepdims=True)
    image = axis.imshow(row_fraction, cmap="Blues", vmin=0.0, vmax=1.0, aspect="equal")
    for actual in (0, 1):
        for predicted in (0, 1):
            value = row_fraction[actual, predicted]
            color = "white" if value >= 0.55 else "#222222"
            axis.text(
                predicted,
                actual,
                f"{matrix[actual, predicted]}\n({100.0 * value:.1f}%)",
                ha="center",
                va="center",
                fontsize=9.2,
                color=color,
                fontweight="bold" if actual == predicted else "normal",
            )
    axis.set_xticks([0, 1], [LABEL_NAMES[0], LABEL_NAMES[1]])
    axis.set_yticks([0, 1], [LABEL_NAMES[0], LABEL_NAMES[1]])
    axis.set_xlabel("Predicted class")
    axis.set_ylabel("Observed future class")
    axis.set_title("(b) Pooled OOF confusion", loc="left", fontweight="bold")
    for spine in axis.spines.values():
        spine.set_visible(True)
        spine.set_linewidth(0.7)
        spine.set_color("#777777")
    colorbar = figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
    colorbar.set_label("Fraction within observed class", fontsize=7.8)
    colorbar.ax.tick_params(labelsize=7.5)

    # Panel C: pooled OOF precision-recall curve.
    axis = axes[2]
    prevalence = float(np.mean(oof["true_label_id"].to_numpy(dtype=int) == 1))
    axis.plot(recall, precision, color=blue, linewidth=2.0, label=f"AP = {pooled['average_precision']:.3f}")
    axis.axhline(
        prevalence,
        color=gray,
        linewidth=1.2,
        linestyle="--",
        label=f"prevalence = {prevalence:.3f}",
    )
    axis.scatter(
        [float(pooled["positive_recall"])],
        [float(pooled["positive_precision"])],
        color=orange,
        edgecolor="white",
        linewidth=0.8,
        s=44,
        zorder=5,
        label="0.5 decision threshold",
    )
    axis.set_xlim(0.0, 1.01)
    axis.set_ylim(0.0, 1.03)
    axis.set_xlabel("Recall for sustained loss")
    axis.set_ylabel("Precision for sustained loss")
    axis.set_title("(c) Pooled OOF precision–recall", loc="left", fontweight="bold")
    axis.grid(color="#d9d9d9", linewidth=0.7)
    axis.legend(loc="lower left", frameon=False)

    figure.suptitle(
        "Seed-disjoint prediction of future sustained raw donor loss",
        fontsize=13.0,
        fontweight="bold",
        y=0.95,
    )
    figure.text(
        0.5,
        0.855,
        "Target: raw donor state is unreachable at t+2, t+4, and t+6 s; offline forecasting only",
        ha="center",
        va="center",
        fontsize=9.3,
        color="#333333",
    )
    figure.text(
        0.5,
        0.045,
        "Five grouped test folds (seeds 1–5); time samples within a run are not independent replications.",
        ha="center",
        va="center",
        fontsize=8.1,
        color="#444444",
    )
    return figure


def write_readme(
    path: Path,
    artifact_dir: Path,
    input_hashes: dict[str, str],
    output_hashes: dict[str, str],
    pooled: dict[str, Any],
) -> None:
    caption = (
        "Seed-disjoint offline evaluation of the fixed Random Forest predictor for "
        "future sustained raw donor loss. (a) Macro-F1 for each leave-one-seed-out "
        "fold, with the unweighted seed mean and pooled OOF macro-F1. (b) Pooled "
        "OOF confusion counts and percentages normalized within each observed class. "
        "(c) Pooled OOF precision–recall curve; the marker shows the 0.5 classifier "
        "threshold. The target requires an unreachable raw donor state at t+2, t+4, "
        "and t+6 s. These panels quantify offline forecasting, not switching latency, "
        "packet delivery, handover performance, or network-policy gain."
    )
    input_lines = "\n".join(
        f"- `{name}`: `{digest}`" for name, digest in sorted(input_hashes.items())
    )
    output_lines = "\n".join(
        f"- `{name}`: `{digest}`" for name, digest in sorted(output_hashes.items())
    )
    path.write_text(
        f"""# Sustained-outage predictor evaluation figure

## Caption

{caption}

## Scope

The figure reads only versioned files in `{artifact_dir}`. It does not read raw
ns-3 traces. The five folds are independent at the seed level; the 1,590 time
rows are not independent simulation replications. Pooled macro-F1 is
`{pooled['macro_f1']:.6f}` and mean seed macro-F1 is
`{pooled['mean_seed_macro_f1']:.6f}`. Seed-level dispersion, especially the
weaker seed-4 fold, must remain visible when reporting the result.

This predictor forecasts a future physical donor condition. It has not, by
this offline figure, demonstrated faster switching or any network-policy gain.

## Verified input hashes

{input_lines}

## Generated output hashes

{output_lines}

The accompanying long-format CSV contains every plotted seed value, confusion
cell, precision–recall point, baseline, and operating point.
""",
        encoding="utf-8",
    )


def main() -> None:
    script_path = Path(__file__).resolve()
    default_artifact_dir = script_path.parents[3] / "results/ai/uav-sustained-outage-v1"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-dir", type=Path, default=default_artifact_dir)
    args = parser.parse_args()
    artifact_dir = args.artifact_dir.resolve()
    input_hashes = verify_inputs(artifact_dir)
    oof, seeds, metrics, protocol = load_and_validate(artifact_dir)
    pooled = metrics["pooled_oof_metrics"]
    rows, matrix, precision, recall = figure_data_rows(oof, seeds, pooled)

    figure_dir = artifact_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    data_path = figure_dir / f"{FIGURE_BASENAME}_data.csv"
    pdf_path = figure_dir / f"{FIGURE_BASENAME}.pdf"
    png_path = figure_dir / f"{FIGURE_BASENAME}.png"
    readme_path = figure_dir / "README.md"
    manifest_path = figure_dir / "figure_manifest_v1.json"

    write_figure_data(data_path, rows)
    figure = make_figure(oof, seeds, pooled, matrix, precision, recall)
    figure.savefig(
        pdf_path,
        format="pdf",
        metadata={
            "Title": "Seed-disjoint prediction of future sustained raw donor loss",
            "Author": "Reproducible ns-3 evidence audit",
            "Subject": "Offline grouped predictor evaluation; no network-policy claim",
            "Creator": script_path.name,
            "CreationDate": None,
            "ModDate": None,
        },
    )
    figure.savefig(
        png_path,
        format="png",
        dpi=300,
        metadata={
            "Title": "Seed-disjoint prediction of future sustained raw donor loss",
            "Software": script_path.name,
        },
    )
    plt.close(figure)

    output_hashes = {
        data_path.name: sha256_file(data_path),
        pdf_path.name: sha256_file(pdf_path),
        png_path.name: sha256_file(png_path),
    }
    write_readme(readme_path, artifact_dir, input_hashes, output_hashes, pooled)
    output_hashes[readme_path.name] = sha256_file(readme_path)
    figure_manifest = {
        "figure_version": "uav-sustained-outage-predictor-figure-v1",
        "scope": (
            "Offline prediction of future sustained raw donor loss; not evidence "
            "of closed-loop switching or network-policy gain"
        ),
        "target_definition": protocol["target_definition"],
        "artifact_dir": str(artifact_dir),
        "script_path": str(script_path),
        "script_sha256": sha256_file(script_path),
        "inputs": input_hashes,
        "outputs": output_hashes,
        "png_dpi": 300,
        "pdf_is_vector": True,
        "plotted_seed_groups": seeds["held_out_seed"].astype(str).tolist(),
        "oof_rows": int(len(oof)),
        "plotted_data_rows": len(rows),
    }
    write_json(manifest_path, figure_manifest)

    print(
        json.dumps(
            {
                "status": "FIGURE_GENERATED_AND_INPUTS_VERIFIED",
                "pdf": str(pdf_path),
                "pdf_sha256": output_hashes[pdf_path.name],
                "png": str(png_path),
                "png_sha256": output_hashes[png_path.name],
                "data_csv": str(data_path),
                "data_csv_sha256": output_hashes[data_path.name],
                "readme_sha256": output_hashes[readme_path.name],
                "manifest": str(manifest_path),
                "manifest_sha256": sha256_file(manifest_path),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
