#!/usr/bin/env python3
"""Train and export a causal UAV donor-backhaul sustained-loss predictor.

This pipeline is deliberately separate from the three-class BackhaulMode
imitation model.  It predicts a physical future condition, not the action of
the existing switching rule:

    SUSTAINED_LOSS(t) = 1 iff RawXhaulState is UNREACHABLE at
    t + 2 s, t + 4 s, and t + 6 s.

Only measurements available at or before t are used as features.  The first
two samples of each run/UAV are excluded because four seconds of history are
required, and the last three are excluded because the target is right-censored.

The script reads raw traces from an explicitly supplied external result tree;
it does not copy those traces into the output.  It freezes a protocol and
dataset hash before fitting, evaluates one fixed Random Forest in leave-one-
seed-out folds, refits on seeds 1--5, exports ONNX, and checks exact sklearn /
ONNX label parity on both the dataset and deterministic numeric probes.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import joblib
import numpy as np
import onnx
import onnxruntime as ort
import pandas as pd
import sklearn
from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (
    accuracy_score,
    average_precision_score,
    balanced_accuracy_score,
    brier_score_loss,
    confusion_matrix,
    f1_score,
    precision_recall_fscore_support,
    roc_auc_score,
)
from sklearn.model_selection import LeaveOneGroupOut


PROTOCOL_VERSION = "uav-sustained-outage-v1"
CONTROL_CADENCE_S = 2.0
PREDICTION_HORIZON_S = 6.0
SOURCE_SEEDS = (1, 2, 3, 4, 5)
LABEL_NAME = "SustainedLossNext6s"
LABELS = ("NO_SUSTAINED_LOSS", "SUSTAINED_LOSS")

FEATURES = [
    "BestDonorDistanceM",
    "XhaulConnected",
    "XhaulChannelVariationDb",
    "XhaulRsrpDbm",
    "RawXhaulStateAgeSec",
    "delta_distance_2s",
    "delta_rsrp_2s",
    "delta_rsrp_4s",
]

FEATURE_SCHEMA = {
    "BestDonorDistanceM": {
        "unit": "m",
        "description": "Current distance to the strongest eligible TN donor",
        "disconnected_default": -1.0,
    },
    "XhaulConnected": {
        "unit": "binary",
        "description": "1 when an eligible terrestrial donor exists, else 0",
        "default": 0.0,
    },
    "XhaulChannelVariationDb": {
        "unit": "dB",
        "description": "Current donor shadowing-minus-fading realization",
        "disconnected_default": 0.0,
    },
    "XhaulRsrpDbm": {
        "unit": "dBm",
        "description": "Current RSRP proxy for the strongest eligible TN donor",
        "disconnected_default": -999.0,
    },
    "RawXhaulStateAgeSec": {
        "unit": "s",
        "description": "Elapsed time in the current instantaneous donor state",
        "default": 0.0,
    },
    "delta_distance_2s": {
        "unit": "m_per_2s",
        "description": "Current donor distance minus its value two seconds earlier",
        "invalid_or_disconnected_default": 0.0,
    },
    "delta_rsrp_2s": {
        "unit": "dB_per_2s",
        "description": "Current donor RSRP minus its value two seconds earlier",
        "invalid_or_disconnected_default": 0.0,
    },
    "delta_rsrp_4s": {
        "unit": "dB_per_4s",
        "description": "Current donor RSRP minus its value four seconds earlier",
        "invalid_or_disconnected_default": 0.0,
    },
}

RAW_COLUMNS = [
    "Time",
    "UavIndex",
    "BestDonorDistanceM",
    "XhaulConnected",
    "XhaulChannelVariationDb",
    "XhaulRsrpDbm",
    "RawXhaulState",
    "RawXhaulStateAgeSec",
]

MODEL_PARAMETERS = {
    "n_estimators": 300,
    "max_depth": 8,
    "min_samples_leaf": 3,
    "class_weight": "balanced",
    "random_state": 7,
    "n_jobs": 1,
}

ARTIFACT_FILENAMES = {
    "dataset": "sustained_outage_dataset_v1.csv",
    "protocol": "frozen_protocol_v1.json",
    "oof_predictions": "grouped_oof_predictions_v1.csv",
    "seed_metrics": "grouped_seed_metrics_v1.csv",
    "metrics": "grouped_oof_metrics_v1.json",
    "joblib": "sustained_outage_rf_v1.joblib",
    "onnx": "sustained_outage_rf_v1.onnx",
    "schema": "input_output_schema_v1.json",
    "onnx_validation": "onnx_validation_v1.json",
    "run_record": "training_run_v1.json",
    "readme": "README.md",
    "manifest": "artifact_manifest_v1.json",
}


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def repository_head(repo: Path) -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True
    ).strip()


def parse_run_identity(run_dir: Path) -> tuple[int, str]:
    seed_match = re.search(r"seed(\d+)", run_dir.name)
    if seed_match is None:
        raise ValueError(f"No seed token in source run directory: {run_dir}")
    seed = int(seed_match.group(1))
    scenario = (
        "satellite" if run_dir.name.startswith("tn-uav-satellite_") else "no_satellite"
    )
    return seed, scenario


def discover_source_runs(base_dir: Path) -> list[Path]:
    if not base_dir.is_dir():
        raise FileNotFoundError(f"Raw result base directory not found: {base_dir}")
    candidates = sorted(
        run_dir
        for run_dir in base_dir.iterdir()
        if run_dir.is_dir()
        and "ai-dataset-v2-logdist-fading-seed" in run_dir.name
        and (run_dir / "xhaul-autonomy-trace.csv").is_file()
    )
    identities = [parse_run_identity(run_dir) for run_dir in candidates]
    expected = {
        (seed, scenario)
        for seed in SOURCE_SEEDS
        for scenario in ("satellite", "no_satellite")
    }
    if set(identities) != expected or len(identities) != len(expected):
        raise ValueError(
            "Expected exactly one satellite and one no-satellite dataset-v2 "
            f"run for seeds {SOURCE_SEEDS}; found {identities} under {base_dir}"
        )
    return candidates


def numeric_column(frame: pd.DataFrame, name: str, source: Path) -> pd.Series:
    values = pd.to_numeric(frame[name], errors="coerce")
    bad = ~np.isfinite(values.to_numpy(dtype=float))
    if bad.any():
        rows = np.flatnonzero(bad)[:10].tolist()
        raise ValueError(f"Non-finite {name} in {source}; row indices {rows}")
    return values.astype(float)


def normalized_link_columns(frame: pd.DataFrame, source: Path) -> pd.DataFrame:
    normalized = pd.DataFrame(index=frame.index)
    connected_numeric = numeric_column(frame, "XhaulConnected", source)
    if not connected_numeric.isin([0.0, 1.0]).all():
        raise ValueError(f"XhaulConnected must be binary in {source}")
    connected = connected_numeric.astype(int)

    distance = numeric_column(frame, "BestDonorDistanceM", source)
    variation = numeric_column(frame, "XhaulChannelVariationDb", source)
    rsrp = numeric_column(frame, "XhaulRsrpDbm", source)
    state_age = numeric_column(frame, "RawXhaulStateAgeSec", source)
    if (state_age < 0.0).any():
        raise ValueError(f"Negative RawXhaulStateAgeSec in {source}")

    normalized["BestDonorDistanceM"] = np.where(connected == 1, distance, -1.0)
    normalized["XhaulConnected"] = connected.astype(float)
    normalized["XhaulChannelVariationDb"] = np.where(
        connected == 1, variation, 0.0
    )
    normalized["XhaulRsrpDbm"] = np.where(connected == 1, rsrp, -999.0)
    normalized["RawXhaulStateAgeSec"] = state_age
    return normalized


def build_run_rows(run_dir: Path) -> tuple[pd.DataFrame, dict[str, Any]]:
    trace_path = run_dir / "xhaul-autonomy-trace.csv"
    source = pd.read_csv(trace_path, usecols=RAW_COLUMNS)
    if source.empty:
        raise ValueError(f"Empty xHaul trace: {trace_path}")
    source["Time"] = numeric_column(source, "Time", trace_path)
    source["UavIndex"] = numeric_column(source, "UavIndex", trace_path).astype(int)
    allowed_states = {"HEALTHY", "UNREACHABLE"}
    observed_states = set(source["RawXhaulState"].astype(str))
    if not observed_states.issubset(allowed_states):
        raise ValueError(f"Unexpected raw states in {trace_path}: {observed_states}")

    seed, scenario = parse_run_identity(run_dir)
    trace_hash = sha256_file(trace_path)
    run_frames: list[pd.DataFrame] = []
    uav_counts: dict[str, int] = {}

    for uav_index, raw_group in source.groupby("UavIndex", sort=True):
        group = raw_group.sort_values("Time").reset_index(drop=True)
        times = group["Time"].to_numpy(dtype=float)
        if len(times) < 6:
            raise ValueError(f"Too few samples for UAV {uav_index} in {trace_path}")
        deltas = np.diff(times)
        if not np.allclose(deltas, CONTROL_CADENCE_S, atol=1e-9, rtol=0.0):
            raise ValueError(
                f"Expected a {CONTROL_CADENCE_S:g}-s cadence for UAV {uav_index} "
                f"in {trace_path}; observed {sorted(set(deltas.tolist()))}"
            )

        current = normalized_link_columns(group, trace_path)
        connected = current["XhaulConnected"] == 1.0
        connected_lag1 = connected.shift(1, fill_value=False)
        connected_lag2 = connected.shift(2, fill_value=False)

        distance_lag1 = current["BestDonorDistanceM"].shift(1)
        rsrp_lag1 = current["XhaulRsrpDbm"].shift(1)
        rsrp_lag2 = current["XhaulRsrpDbm"].shift(2)

        current["delta_distance_2s"] = np.where(
            connected & connected_lag1,
            current["BestDonorDistanceM"] - distance_lag1,
            0.0,
        )
        current["delta_rsrp_2s"] = np.where(
            connected & connected_lag1,
            current["XhaulRsrpDbm"] - rsrp_lag1,
            0.0,
        )
        current["delta_rsrp_4s"] = np.where(
            connected & connected_lag2,
            current["XhaulRsrpDbm"] - rsrp_lag2,
            0.0,
        )

        raw_state = group["RawXhaulState"].astype(str)
        future_2 = raw_state.shift(-1)
        future_4 = raw_state.shift(-2)
        future_6 = raw_state.shift(-3)
        valid_history = pd.Series(np.arange(len(group)) >= 2, index=group.index)
        valid_target = future_6.notna()
        keep = valid_history & valid_target

        rows = pd.DataFrame(
            {
                "SourceRun": run_dir.name,
                "SourceTraceSha256": trace_hash,
                "Seed": seed,
                "SeedGroup": f"seed{seed}",
                "ScenarioKind": scenario,
                "Time": group["Time"],
                "UavIndex": int(uav_index),
                "TargetHorizonEndTime": group["Time"] + PREDICTION_HORIZON_S,
            }
        )
        for feature in FEATURES:
            rows[feature] = current[feature].astype(float)
        rows[LABEL_NAME] = (
            (future_2 == "UNREACHABLE")
            & (future_4 == "UNREACHABLE")
            & (future_6 == "UNREACHABLE")
        ).astype(int)
        rows = rows.loc[keep].copy()
        if not np.isfinite(rows[FEATURES].to_numpy(dtype=float)).all():
            raise ValueError(f"Non-finite engineered feature in {trace_path}")
        uav_counts[str(int(uav_index))] = len(rows)
        run_frames.append(rows)

    result = pd.concat(run_frames, ignore_index=True)
    manifest = {
        "run_name": run_dir.name,
        "run_path": str(run_dir.resolve()),
        "seed": seed,
        "scenario": scenario,
        "source_trace_path": str(trace_path.resolve()),
        "source_trace_sha256": trace_hash,
        "source_trace_rows": int(len(source)),
        "derived_rows": int(len(result)),
        "derived_rows_by_uav": uav_counts,
        "raw_time_min_s": float(source["Time"].min()),
        "raw_time_max_s": float(source["Time"].max()),
    }
    return result, manifest


def build_dataset(run_dirs: list[Path]) -> tuple[pd.DataFrame, list[dict[str, Any]]]:
    frames: list[pd.DataFrame] = []
    manifests: list[dict[str, Any]] = []
    for run_dir in run_dirs:
        frame, manifest = build_run_rows(run_dir)
        frames.append(frame)
        manifests.append(manifest)
    dataset = pd.concat(frames, ignore_index=True)
    scenario_order = pd.Categorical(
        dataset["ScenarioKind"], categories=["no_satellite", "satellite"], ordered=True
    )
    dataset = (
        dataset.assign(_scenario_order=scenario_order)
        .sort_values(["Seed", "_scenario_order", "UavIndex", "Time"])
        .drop(columns="_scenario_order")
        .reset_index(drop=True)
    )
    dataset.insert(0, "RowId", np.arange(len(dataset), dtype=int))
    if set(dataset["Seed"].unique()) != set(SOURCE_SEEDS):
        raise ValueError("Derived dataset does not contain exactly seeds 1--5")
    if set(dataset[LABEL_NAME].unique()) != {0, 1}:
        raise ValueError("Derived dataset does not contain both sustained-loss classes")
    return dataset, manifests


def make_model() -> RandomForestClassifier:
    return RandomForestClassifier(**MODEL_PARAMETERS)


def classification_metrics(y_true: np.ndarray, y_pred: np.ndarray, probability: np.ndarray) -> dict[str, Any]:
    matrix = confusion_matrix(y_true, y_pred, labels=[0, 1])
    precision, recall, f1, support = precision_recall_fscore_support(
        y_true, y_pred, labels=[0, 1], zero_division=0
    )
    return {
        "samples": int(len(y_true)),
        "class_support": {LABELS[index]: int(support[index]) for index in (0, 1)},
        "accuracy": float(accuracy_score(y_true, y_pred)),
        "balanced_accuracy": float(balanced_accuracy_score(y_true, y_pred)),
        "macro_f1": float(f1_score(y_true, y_pred, labels=[0, 1], average="macro")),
        "positive_precision": float(precision[1]),
        "positive_recall": float(recall[1]),
        "positive_f1": float(f1[1]),
        "roc_auc": float(roc_auc_score(y_true, probability)),
        "average_precision": float(average_precision_score(y_true, probability)),
        "brier_score": float(brier_score_loss(y_true, probability)),
        "confusion_matrix_label_order_0_1": matrix.tolist(),
    }


def grouped_oof(
    features: np.ndarray, labels: np.ndarray, groups: np.ndarray
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]]]:
    predictions = np.full(len(labels), -1, dtype=np.int64)
    probabilities = np.full(len(labels), np.nan, dtype=np.float64)
    seed_metrics: list[dict[str, Any]] = []
    splitter = LeaveOneGroupOut()

    for train_indices, test_indices in splitter.split(features, labels, groups):
        test_groups = sorted(set(groups[test_indices]))
        if len(test_groups) != 1:
            raise AssertionError(f"Expected one held-out seed, got {test_groups}")
        held_out = test_groups[0]
        if set(labels[train_indices]) != {0, 1} or set(labels[test_indices]) != {0, 1}:
            raise ValueError(f"Both classes must occur in train and test for {held_out}")
        model = make_model()
        model.fit(features[train_indices], labels[train_indices])
        fold_prediction = model.predict(features[test_indices]).astype(np.int64)
        fold_probability = model.predict_proba(features[test_indices])[:, 1]
        predictions[test_indices] = fold_prediction
        probabilities[test_indices] = fold_probability
        metrics = classification_metrics(
            labels[test_indices], fold_prediction, fold_probability
        )
        metrics.update(
            {
                "held_out_seed": held_out,
                "training_seeds": sorted(set(groups[train_indices])),
                "train_samples": int(len(train_indices)),
                "test_samples": int(len(test_indices)),
            }
        )
        seed_metrics.append(metrics)

    if (predictions < 0).any() or not np.isfinite(probabilities).all():
        raise AssertionError("Every row must receive exactly one finite OOF prediction")
    return predictions, probabilities, sorted(
        seed_metrics, key=lambda row: row["held_out_seed"]
    )


def flattened_seed_metric_rows(seed_metrics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for metric in seed_metrics:
        matrix = metric["confusion_matrix_label_order_0_1"]
        rows.append(
            {
                "held_out_seed": metric["held_out_seed"],
                "training_seeds": " ".join(metric["training_seeds"]),
                "train_samples": metric["train_samples"],
                "test_samples": metric["test_samples"],
                "negative_support": metric["class_support"][LABELS[0]],
                "positive_support": metric["class_support"][LABELS[1]],
                "accuracy": metric["accuracy"],
                "balanced_accuracy": metric["balanced_accuracy"],
                "macro_f1": metric["macro_f1"],
                "positive_precision": metric["positive_precision"],
                "positive_recall": metric["positive_recall"],
                "positive_f1": metric["positive_f1"],
                "roc_auc": metric["roc_auc"],
                "average_precision": metric["average_precision"],
                "brier_score": metric["brier_score"],
                "true_negative": matrix[0][0],
                "false_positive": matrix[0][1],
                "false_negative": matrix[1][0],
                "true_positive": matrix[1][1],
            }
        )
    return rows


def add_onnx_metadata(model: onnx.ModelProto, metadata: dict[str, str]) -> None:
    del model.metadata_props[:]
    for key, value in sorted(metadata.items()):
        item = model.metadata_props.add()
        item.key = key
        item.value = value


def onnx_interface(session: ort.InferenceSession) -> dict[str, Any]:
    return {
        "inputs": [
            {"name": item.name, "shape": item.shape, "type": item.type}
            for item in session.get_inputs()
        ],
        "outputs": [
            {"name": item.name, "shape": item.shape, "type": item.type}
            for item in session.get_outputs()
        ],
    }


def run_onnx(
    session: ort.InferenceSession, values: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or inputs[0].type != "tensor(float)":
        raise ValueError(f"Unexpected ONNX input interface: {onnx_interface(session)}")
    if len(outputs) != 2:
        raise ValueError(f"Expected label and probability outputs: {onnx_interface(session)}")
    result = session.run(None, {inputs[0].name: values.astype(np.float32, copy=False)})
    label_index = next(
        (index for index, item in enumerate(outputs) if item.type == "tensor(int64)"), None
    )
    probability_index = next(
        (index for index, item in enumerate(outputs) if item.type == "tensor(float)"), None
    )
    if label_index is None or probability_index is None:
        raise ValueError(f"Unexpected ONNX output types: {onnx_interface(session)}")
    labels = np.asarray(result[label_index], dtype=np.int64).reshape(-1)
    probabilities = np.asarray(result[probability_index], dtype=np.float32)
    if probabilities.ndim != 2 or probabilities.shape[1] != 2:
        raise ValueError(f"Expected [N,2] probabilities, got {probabilities.shape}")
    return labels, probabilities


def synthetic_probes(features: np.ndarray, count: int = 2000) -> np.ndarray:
    rng = np.random.default_rng(7)
    minimum = features.min(axis=0).astype(np.float64)
    maximum = features.max(axis=0).astype(np.float64)
    probes = rng.uniform(minimum, maximum, size=(count, len(FEATURES)))
    connected_index = FEATURES.index("XhaulConnected")
    probes[:, connected_index] = rng.integers(0, 2, size=count)
    disconnected = probes[:, connected_index] == 0.0
    probes[disconnected, FEATURES.index("BestDonorDistanceM")] = -1.0
    probes[disconnected, FEATURES.index("XhaulChannelVariationDb")] = 0.0
    probes[disconnected, FEATURES.index("XhaulRsrpDbm")] = -999.0
    probes[disconnected, FEATURES.index("delta_distance_2s")] = 0.0
    probes[disconnected, FEATURES.index("delta_rsrp_2s")] = 0.0
    probes[disconnected, FEATURES.index("delta_rsrp_4s")] = 0.0
    return probes.astype(np.float32)


def validate_onnx(
    model: RandomForestClassifier,
    onnx_path: Path,
    dataset_features: np.ndarray,
) -> dict[str, Any]:
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 1
    session_options.inter_op_num_threads = 1
    session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(
        str(onnx_path),
        sess_options=session_options,
        providers=["CPUExecutionProvider"],
    )

    def compare(values: np.ndarray, name: str) -> dict[str, Any]:
        sklearn_labels = model.predict(values).astype(np.int64)
        sklearn_probabilities = model.predict_proba(values).astype(np.float64)
        onnx_labels, onnx_probabilities = run_onnx(session, values)
        label_matches = int(np.sum(sklearn_labels == onnx_labels))
        maximum_error = float(
            np.max(np.abs(sklearn_probabilities - onnx_probabilities.astype(np.float64)))
        )
        if label_matches != len(values):
            mismatch = np.flatnonzero(sklearn_labels != onnx_labels)[:10].tolist()
            raise ValueError(f"ONNX label mismatch on {name}; indices {mismatch}")
        if maximum_error > 1e-5:
            raise ValueError(
                f"ONNX probability error on {name} exceeds tolerance: {maximum_error}"
            )
        return {
            "rows": int(len(values)),
            "exact_label_matches": label_matches,
            "exact_label_match_rate_pct": 100.0 * label_matches / len(values),
            "maximum_absolute_probability_error": maximum_error,
        }

    probes = synthetic_probes(dataset_features)
    return {
        "interface": onnx_interface(session),
        "providers": session.get_providers(),
        "dataset_check": compare(dataset_features, "dataset"),
        "synthetic_range_probe_check": compare(probes, "synthetic probes"),
        "probability_tolerance": 1e-5,
        "synthetic_probe_seed": 7,
        "synthetic_probe_note": (
            "Probes remain within observed per-feature numeric ranges and preserve "
            "the disconnected-sentinel contract; combinations need not be physical."
        ),
    }


def environment_record() -> dict[str, Any]:
    versions = {
        "python": platform.python_version(),
        "numpy": np.__version__,
        "pandas": pd.__version__,
        "scikit_learn": sklearn.__version__,
        "joblib": joblib.__version__,
        "onnx": onnx.__version__,
        "onnxruntime": ort.__version__,
    }
    try:
        import skl2onnx

        versions["skl2onnx"] = skl2onnx.__version__
    except Exception:
        versions["skl2onnx"] = "unknown"
    return {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "cpu_count": os.cpu_count(),
        "versions": versions,
    }


def write_readme(path: Path, base_dir: Path, dataset_rows: int) -> None:
    path.write_text(
        f"""# UAV sustained donor-loss predictor v1

This directory was generated from raw `xhaul-autonomy-trace.csv` files under
`{base_dir.resolve()}`. Raw traces were read in place and were not copied here.

The binary target is one only when `RawXhaulState` is `UNREACHABLE` at all
three future controller epochs: t+2 s, t+4 s, and t+6 s. Features use only the
current and preceding four seconds. The first two and final three samples per
run/UAV are therefore excluded. The resulting dataset contains {dataset_rows}
rows from seeds 1--5, both satellite and no-satellite scenarios, and all three
UAVs.

`grouped_oof_predictions_v1.csv` contains leave-one-seed-out predictions from
the fixed 300-tree Random Forest. `sustained_outage_rf_v1.onnx` is the final
all-training-seed refit, not an independently tested artifact. Its intended
network-test seeds are fresh seeds outside 1--5.

The model predicts future donor loss only. Satellite feasibility, probability
persistence, dwell time, and route selection remain external policy guards.
It must not be interpreted as the earlier three-class BackhaulMode imitation
model or as evidence of network benefit without a fresh closed-loop test.
""",
        encoding="utf-8",
    )


def main() -> None:
    script_path = Path(__file__).resolve()
    repo = script_path.parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-dir",
        type=Path,
        required=True,
        help="Directory containing the ten raw dataset-v2 seed1--5 result folders",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo / "results/ai/uav-sustained-outage-v1",
    )
    args = parser.parse_args()
    base_dir = args.base_dir.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(
            f"Refusing to overwrite non-empty output directory: {output_dir}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()

    run_dirs = discover_source_runs(base_dir)
    dataset, source_manifest = build_dataset(run_dirs)
    dataset_path = output_dir / ARTIFACT_FILENAMES["dataset"]
    dataset.to_csv(dataset_path, index=False)
    dataset_sha256 = sha256_file(dataset_path)

    protocol = {
        "protocol_version": PROTOCOL_VERSION,
        "status": "FROZEN_BEFORE_MODEL_FITS",
        "created_utc": utc_now(),
        "repository_head": repository_head(repo),
        "script_path": str(script_path),
        "script_sha256": sha256_file(script_path),
        "raw_base_dir": str(base_dir),
        "raw_traces_copied": False,
        "source_runs": source_manifest,
        "dataset_path": str(dataset_path),
        "dataset_sha256": dataset_sha256,
        "dataset_rows": int(len(dataset)),
        "source_seeds": list(SOURCE_SEEDS),
        "feature_order": FEATURES,
        "feature_schema": FEATURE_SCHEMA,
        "feature_causality": (
            "Every feature is evaluated at t or from t-2/t-4. No future QoS, "
            "scenario flag, seed, rule action, or effective BackhaulMode is an input."
        ),
        "target_name": LABEL_NAME,
        "target_definition": (
            "1 iff RawXhaulState is UNREACHABLE at t+2 s, t+4 s, and t+6 s"
        ),
        "control_cadence_s": CONTROL_CADENCE_S,
        "prediction_horizon_s": PREDICTION_HORIZON_S,
        "left_exclusion": "First two samples per run/UAV lack four seconds of history",
        "right_exclusion": "Last three samples per run/UAV lack the complete future target",
        "model_family": "RandomForestClassifier",
        "fixed_model_parameters": MODEL_PARAMETERS,
        "offline_evaluation": (
            "LeaveOneGroupOut by SeedGroup; both scenarios, every UAV, and every "
            "time from the held-out seed remain together"
        ),
        "final_refit": "One fixed model refit on all seed1--5 rows after OOF evaluation",
        "network_test_seeds": "Must be outside source seeds 1--5 and fixed before testing",
        "operational_policy_not_selected_here": (
            "Probability threshold, consecutive-positive requirement, satellite "
            "feasibility, and dwell guards are outside this offline predictor."
        ),
    }
    protocol_path = output_dir / ARTIFACT_FILENAMES["protocol"]
    write_json(protocol_path, protocol)
    protocol_sha256 = sha256_file(protocol_path)

    feature_matrix = dataset[FEATURES].to_numpy(dtype=np.float32)
    labels = dataset[LABEL_NAME].to_numpy(dtype=np.int64)
    groups = dataset["SeedGroup"].astype(str).to_numpy()
    if not np.isfinite(feature_matrix).all():
        raise ValueError("Non-finite value in final feature matrix")

    oof_prediction, oof_probability, seed_metrics = grouped_oof(
        feature_matrix, labels, groups
    )
    pooled_metrics = classification_metrics(labels, oof_prediction, oof_probability)
    pooled_metrics.update(
        {
            "evaluation": "pooled leave-one-seed-out OOF",
            "seed_count": len(SOURCE_SEEDS),
            "all_rows_tested_once": True,
            "mean_seed_macro_f1": float(
                np.mean([metric["macro_f1"] for metric in seed_metrics])
            ),
            "mean_seed_balanced_accuracy": float(
                np.mean([metric["balanced_accuracy"] for metric in seed_metrics])
            ),
            "uncertainty_note": (
                "Only five seed groups are available; time rows are not independent "
                "replications and no row-level confidence interval is reported."
            ),
        }
    )

    oof = dataset[
        ["RowId", "SourceRun", "Seed", "SeedGroup", "ScenarioKind", "Time", "UavIndex"]
    ].copy()
    oof["true_label_id"] = labels
    oof["true_label"] = [LABELS[value] for value in labels]
    oof["predicted_label_id"] = oof_prediction
    oof["predicted_label"] = [LABELS[value] for value in oof_prediction]
    oof["sustained_loss_probability"] = oof_probability
    oof.to_csv(output_dir / ARTIFACT_FILENAMES["oof_predictions"], index=False)
    pd.DataFrame(flattened_seed_metric_rows(seed_metrics)).to_csv(
        output_dir / ARTIFACT_FILENAMES["seed_metrics"], index=False
    )
    metrics_record = {
        "protocol_sha256": protocol_sha256,
        "dataset_sha256": dataset_sha256,
        "pooled_oof_metrics": pooled_metrics,
        "per_seed_metrics": seed_metrics,
    }
    metrics_path = output_dir / ARTIFACT_FILENAMES["metrics"]
    write_json(metrics_path, metrics_record)

    final_model = make_model()
    final_model.fit(feature_matrix, labels)
    joblib_bundle = {
        "model": final_model,
        "protocol_version": PROTOCOL_VERSION,
        "features": FEATURES,
        "feature_schema": FEATURE_SCHEMA,
        "labels": list(LABELS),
        "label_mapping": {0: LABELS[0], 1: LABELS[1]},
        "target_name": LABEL_NAME,
        "target_definition": protocol["target_definition"],
        "prediction_horizon_s": PREDICTION_HORIZON_S,
        "control_cadence_s": CONTROL_CADENCE_S,
        "source_seeds": list(SOURCE_SEEDS),
        "dataset_sha256": dataset_sha256,
        "protocol_sha256": protocol_sha256,
        "evaluation_scope": (
            "Final refit uses all seed1--5 rows and has no independent offline test; "
            "use grouped OOF metrics for offline generalization."
        ),
    }
    joblib_path = output_dir / ARTIFACT_FILENAMES["joblib"]
    joblib.dump(joblib_bundle, joblib_path, compress=3)
    joblib_sha256 = sha256_file(joblib_path)

    onnx_model = convert_sklearn(
        final_model,
        initial_types=[("features", FloatTensorType([None, len(FEATURES)]))],
        target_opset=12,
        options={id(final_model): {"zipmap": False}},
    )
    add_onnx_metadata(
        onnx_model,
        {
            "protocol_version": PROTOCOL_VERSION,
            "dataset_sha256": dataset_sha256,
            "protocol_sha256": protocol_sha256,
            "joblib_sha256": joblib_sha256,
            "feature_order": json.dumps(FEATURES, separators=(",", ":")),
            "label_order": json.dumps(LABELS, separators=(",", ":")),
            "target_definition": protocol["target_definition"],
        },
    )
    onnx.checker.check_model(onnx_model)
    onnx_path = output_dir / ARTIFACT_FILENAMES["onnx"]
    onnx_path.write_bytes(onnx_model.SerializeToString())
    onnx_sha256 = sha256_file(onnx_path)
    onnx_validation = validate_onnx(final_model, onnx_path, feature_matrix)
    onnx_validation.update(
        {
            "onnx_path": str(onnx_path),
            "onnx_sha256": onnx_sha256,
            "source_joblib_path": str(joblib_path),
            "source_joblib_sha256": joblib_sha256,
            "dataset_sha256": dataset_sha256,
            "protocol_sha256": protocol_sha256,
            "conversion": {
                "target_opset": 12,
                "zipmap": False,
                "input_dtype": "float32",
            },
        }
    )
    validation_path = output_dir / ARTIFACT_FILENAMES["onnx_validation"]
    write_json(validation_path, onnx_validation)

    schema = {
        "schema_version": PROTOCOL_VERSION,
        "feature_order": FEATURES,
        "features": FEATURE_SCHEMA,
        "preprocessing": {
            "dtype": "float32",
            "nonfinite_values": "invalid; do not invoke the model",
            "history_requirement_s": 4.0,
            "invalid_history_behavior": "FeatureValid=0 and use the rule policy",
            "disconnected_normalization": {
                "BestDonorDistanceM": -1.0,
                "XhaulChannelVariationDb": 0.0,
                "XhaulRsrpDbm": -999.0,
                "delta_distance_2s": 0.0,
                "delta_rsrp_2s": 0.0,
                "delta_rsrp_4s": 0.0,
            },
        },
        "target": {
            "name": LABEL_NAME,
            "definition": protocol["target_definition"],
            "horizon_s": PREDICTION_HORIZON_S,
            "cadence_s": CONTROL_CADENCE_S,
        },
        "labels": {"0": LABELS[0], "1": LABELS[1]},
        "onnx_interface": onnx_validation["interface"],
        "positive_class_probability_column": 1,
        "policy_note": (
            "The ONNX output is a donor-loss forecast. Satellite feasibility and "
            "the applied backhaul action are external guarded policy decisions."
        ),
    }
    schema_path = output_dir / ARTIFACT_FILENAMES["schema"]
    write_json(schema_path, schema)
    write_readme(output_dir / ARTIFACT_FILENAMES["readme"], base_dir, len(dataset))

    run_record = {
        "status": "COMPLETED_OFFLINE_TRAINING_AND_ONNX_VALIDATION",
        "completed_utc": utc_now(),
        "elapsed_wall_s": time.perf_counter() - started,
        "command": " ".join([sys.executable, str(script_path), *sys.argv[1:]]),
        "repository_head": repository_head(repo),
        "script_sha256": sha256_file(script_path),
        "environment": environment_record(),
        "raw_base_dir": str(base_dir),
        "raw_traces_copied": False,
        "dataset_rows": int(len(dataset)),
        "negative_rows": int(np.sum(labels == 0)),
        "positive_rows": int(np.sum(labels == 1)),
        "positive_prevalence": float(np.mean(labels == 1)),
        "pooled_oof_metrics": pooled_metrics,
        "model_parameters": MODEL_PARAMETERS,
        "dataset_sha256": dataset_sha256,
        "protocol_sha256": protocol_sha256,
        "joblib_sha256": joblib_sha256,
        "onnx_sha256": onnx_sha256,
        "onnx_validation_passed": True,
        "network_simulations_performed": False,
    }
    run_record_path = output_dir / ARTIFACT_FILENAMES["run_record"]
    write_json(run_record_path, run_record)

    manifest_files: dict[str, Any] = {}
    for path in sorted(output_dir.iterdir()):
        if path.is_file() and path.name != ARTIFACT_FILENAMES["manifest"]:
            manifest_files[path.name] = {
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
    manifest = {
        "manifest_version": PROTOCOL_VERSION,
        "created_utc": utc_now(),
        "output_dir": str(output_dir),
        "raw_base_dir": str(base_dir),
        "raw_traces_copied": False,
        "files": manifest_files,
    }
    manifest_path = output_dir / ARTIFACT_FILENAMES["manifest"]
    write_json(manifest_path, manifest)

    print(
        json.dumps(
            {
                "status": run_record["status"],
                "output_dir": str(output_dir),
                "dataset_rows": len(dataset),
                "positive_rows": run_record["positive_rows"],
                "pooled_macro_f1": pooled_metrics["macro_f1"],
                "pooled_roc_auc": pooled_metrics["roc_auc"],
                "onnx_dataset_exact_labels": onnx_validation["dataset_check"][
                    "exact_label_matches"
                ],
                "onnx_probe_exact_labels": onnx_validation[
                    "synthetic_range_probe_check"
                ]["exact_label_matches"],
                "dataset_sha256": dataset_sha256,
                "protocol_sha256": protocol_sha256,
                "joblib_sha256": joblib_sha256,
                "onnx_sha256": onnx_sha256,
                "manifest_sha256": sha256_file(manifest_path),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
