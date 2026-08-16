#!/usr/bin/env python3
"""Build and evaluate a proactive UAV handover predictor from ns-3 traces.

The target at decision time t is one when the O-RAN logic module selects a
handover target in the future-only interval (t, t + horizon].  Candidate rows
in ``ml-ho-dataset.csv`` are collapsed to one row per UE and decision time.

This script is an offline prediction benchmark.  It does not replace the
runtime handover logic or, by itself, demonstrate an improvement in network
KPIs.  Use independent simulation runs (normally random-number seeds) for the
train, validation, and test partitions.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, precision_recall_fscore_support
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.tree import DecisionTreeClassifier


REQUIRED_COLUMNS = {
    "time",
    "ueId",
    "servingCell",
    "servingRsrp",
    "candidateCell",
    "candidateRsrp",
    "candidateLoad",
    "candidateCap",
    "candidateIsNtn",
    "finalChosenCell",
}

FEATURES = [
    "serving_rsrp_dbm",
    "best_neighbor_rsrp_dbm",
    "rsrp_margin_db",
    "serving_rsrp_slope_dbps",
    "neighbor_rsrp_slope_dbps",
    "best_neighbor_load_ratio",
    "best_neighbor_is_ntn",
]


def seed_from_path(path: Path) -> str:
    match = re.search(r"(?:^|[-_])seed(\d+)(?:[-_]|$)", path.parent.name)
    return f"seed{match.group(1)}" if match else path.parent.name


def discover_inputs(base_dir: Path, pattern: str) -> list[Path]:
    return sorted(path for path in base_dir.glob(pattern) if path.stat().st_size > 0)


def aggregate_trace(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(path)
    missing = REQUIRED_COLUMNS.difference(raw.columns)
    if missing:
        raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")

    numeric = list(REQUIRED_COLUMNS)
    for column in numeric:
        raw[column] = pd.to_numeric(raw[column], errors="coerce")
    raw = raw.dropna(subset=["time", "ueId", "servingRsrp", "candidateRsrp"])
    if raw.empty:
        return pd.DataFrame()

    # One candidate row per (run, UE, reporting time): use the strongest cell
    # other than the serving cell.  finalChosenCell is retained only for labels.
    candidates = raw[raw["candidateCell"] != raw["servingCell"]].copy()
    if candidates.empty:
        return pd.DataFrame()
    best_idx = candidates.groupby(["ueId", "time"])["candidateRsrp"].idxmax()
    best = candidates.loc[best_idx].copy()

    chosen = (
        raw.groupby(["ueId", "time"], as_index=False)["finalChosenCell"]
        .max()
        .rename(columns={"finalChosenCell": "chosen_cell"})
    )
    best = best.merge(chosen, on=["ueId", "time"], how="left")
    best = best.sort_values(["ueId", "time"]).reset_index(drop=True)
    best["serving_rsrp_dbm"] = best["servingRsrp"]
    best["best_neighbor_rsrp_dbm"] = best["candidateRsrp"]
    best["rsrp_margin_db"] = best["candidateRsrp"] - best["servingRsrp"]
    capacity = best["candidateCap"].replace(0, np.nan)
    best["best_neighbor_load_ratio"] = (best["candidateLoad"] / capacity).fillna(1.0)
    best["best_neighbor_is_ntn"] = best["candidateIsNtn"].astype(float)

    for source, destination in [
        ("serving_rsrp_dbm", "serving_rsrp_slope_dbps"),
        ("best_neighbor_rsrp_dbm", "neighbor_rsrp_slope_dbps"),
    ]:
        delta_value = best.groupby("ueId")[source].diff()
        delta_time = best.groupby("ueId")["time"].diff()
        best[destination] = (delta_value / delta_time.replace(0, np.nan)).fillna(0.0)

    best["run"] = path.parent.name
    best["seed_group"] = seed_from_path(path)
    return best


def add_future_labels(frame: pd.DataFrame, horizon_s: float) -> pd.DataFrame:
    labelled = []
    for _, ue in frame.groupby(["run", "ueId"], sort=False):
        ue = ue.sort_values("time").copy()
        times = ue["time"].to_numpy(dtype=float)
        events = (ue["chosen_cell"].fillna(0).to_numpy(dtype=float) > 0).astype(np.int64)
        cumulative = np.concatenate(([0], np.cumsum(events)))
        right = np.searchsorted(times, times + horizon_s, side="right")
        left = np.arange(len(times)) + 1  # strictly after t; no current-decision leakage
        ue["ho_within_horizon"] = (cumulative[right] - cumulative[left] > 0).astype(int)
        labelled.append(ue)
    return pd.concat(labelled, ignore_index=True) if labelled else pd.DataFrame()


def split_groups(
    data: pd.DataFrame, group_column: str, seed: int
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    groups = np.array(sorted(data[group_column].astype(str).unique()))
    if len(groups) < 5:
        raise ValueError(
            f"Grouped 60/20/20 splitting needs at least 5 independent groups; found {len(groups)}. "
            "Generate more random-seed runs or use --group-by run when appropriate."
        )
    rng = np.random.default_rng(seed)
    rng.shuffle(groups)
    train_end = max(1, int(np.floor(0.60 * len(groups))))
    validation_end = max(train_end + 1, int(np.floor(0.80 * len(groups))))
    validation_end = min(validation_end, len(groups) - 1)
    train_groups = set(groups[:train_end])
    validation_groups = set(groups[train_end:validation_end])
    test_groups = set(groups[validation_end:])
    return (
        data[data[group_column].astype(str).isin(train_groups)].copy(),
        data[data[group_column].astype(str).isin(validation_groups)].copy(),
        data[data[group_column].astype(str).isin(test_groups)].copy(),
    )


def metrics(model, x: pd.DataFrame, y: pd.Series) -> dict[str, float | int]:
    start = time.perf_counter()
    prediction = model.predict(x)
    elapsed = time.perf_counter() - start
    precision, recall, f1, _ = precision_recall_fscore_support(
        y, prediction, average="binary", zero_division=0
    )
    return {
        "samples": int(len(y)),
        "positive_samples": int(y.sum()),
        "accuracy": float(accuracy_score(y, prediction)),
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "inference_ms_per_sample": float(1000.0 * elapsed / max(len(y), 1)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", type=Path, default=Path("results/nr/tn-ntn"))
    parser.add_argument("--glob", default="**/ml-ho-dataset.csv")
    parser.add_argument("--horizon-s", type=float, default=5.0)
    parser.add_argument("--group-by", choices=["seed", "run"], default="seed")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("results/ai/uav-proactive-handover")
    )
    args = parser.parse_args()
    if args.horizon_s <= 0:
        parser.error("--horizon-s must be positive")

    inputs = discover_inputs(args.base_dir, args.glob)
    if not inputs:
        raise SystemExit(f"No non-empty inputs matching {args.base_dir / args.glob}")
    frames = []
    for path in inputs:
        try:
            frame = aggregate_trace(path)
        except ValueError as error:
            print(f"[skip] {error}")
            continue
        if not frame.empty:
            frames.append(frame)
            print(f"[input] {path} decision_rows={len(frame)}")
    if not frames:
        raise SystemExit("No usable handover decision traces were found")

    data = add_future_labels(pd.concat(frames, ignore_index=True), args.horizon_s)
    data = data.replace([np.inf, -np.inf], np.nan).dropna(subset=FEATURES)
    group_column = "seed_group" if args.group_by == "seed" else "run"
    train, validation, test = split_groups(data, group_column, args.seed)
    for name, partition in [("train", train), ("validation", validation), ("test", test)]:
        if partition.empty or partition["ho_within_horizon"].nunique() < 2:
            raise ValueError(f"The {name} partition does not contain both target classes")

    x_train, y_train = train[FEATURES], train["ho_within_horizon"]
    models = {
        "logistic_regression": make_pipeline(
            StandardScaler(),
            LogisticRegression(max_iter=1000, class_weight="balanced", random_state=args.seed),
        ),
        "decision_tree": DecisionTreeClassifier(
            max_depth=6, min_samples_leaf=5, class_weight="balanced", random_state=args.seed
        ),
        "random_forest": RandomForestClassifier(
            n_estimators=300,
            max_depth=10,
            min_samples_leaf=3,
            class_weight="balanced",
            random_state=args.seed,
            n_jobs=-1,
        ),
    }

    rows = []
    for model_name, model in models.items():
        model.fit(x_train, y_train)
        for partition_name, partition in [("validation", validation), ("test", test)]:
            row = {
                "model": model_name,
                "partition": partition_name,
                **metrics(model, partition[FEATURES], partition["ho_within_horizon"]),
            }
            rows.append(row)
        print(f"[model] {model_name} test_f1={rows[-1]['f1']:.4f}")

    comparison = pd.DataFrame(rows)
    validation_comparison = comparison[comparison["partition"] == "validation"]
    test_comparison = comparison[comparison["partition"] == "test"]
    # Select on validation data so the held-out test partition remains an
    # unbiased final estimate.
    best_name = str(
        validation_comparison.sort_values(["f1", "recall"], ascending=False).iloc[0]["model"]
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    dataset_columns = ["run", "seed_group", "time", "ueId", *FEATURES, "ho_within_horizon"]
    data[dataset_columns].to_csv(args.output_dir / "proactive_handover_dataset.csv", index=False)
    comparison.to_csv(args.output_dir / "proactive_handover_model_comparison.csv", index=False)
    joblib.dump(
        {"model": models[best_name], "features": FEATURES, "horizon_s": args.horizon_s},
        args.output_dir / "proactive_handover_model.joblib",
    )
    report = {
        "scope": "offline future-handover prediction; not a closed-loop network-KPI result",
        "horizon_s": args.horizon_s,
        "group_by": args.group_by,
        "input_files": len(inputs),
        "decision_samples": len(data),
        "positive_samples": int(data["ho_within_horizon"].sum()),
        "features": FEATURES,
        "partitions": {
            name: {
                "samples": len(partition),
                "groups": sorted(partition[group_column].astype(str).unique().tolist()),
            }
            for name, partition in [("train", train), ("validation", validation), ("test", test)]
        },
        "selected_model": best_name,
    }
    (args.output_dir / "proactive_handover_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(f"[saved] {args.output_dir}")
    print(test_comparison.to_string(index=False))


if __name__ == "__main__":
    main()
