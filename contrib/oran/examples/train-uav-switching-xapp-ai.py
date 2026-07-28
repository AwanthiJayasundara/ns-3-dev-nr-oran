#!/usr/bin/env python3
"""Train an AI model for UAV TN/NTN backhaul switching.

The current labels are learned from the existing switching trace
(`BackhaulMode`). This is an imitation-learning baseline for replacing the
rule block with a model. To prove QoS improvement, generate more scenario
variation and relabel samples using future-window QoS rewards.
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
from sklearn.ensemble import HistGradientBoostingClassifier, RandomForestClassifier
from sklearn.inspection import permutation_importance
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import GroupShuffleSplit, train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.tree import DecisionTreeClassifier

try:
    from xgboost import XGBClassifier

    HAVE_XGBOOST = True
except Exception:
    XGBClassifier = None
    HAVE_XGBOOST = False


FEATURES = [
    "BestDonorDistanceM",
    "XhaulConnected",
    "XhaulChannelVariationDb",
    "XhaulRsrpDbm",
    "SatBackhaulDlSnrDb",
    "SatBackhaulUlSnrDb",
    "SatBackhaulHealthy",
    "UavSwitchingXappAvailable",
    "recent_dl_pdr",
    "recent_dl_delay_ms",
    "recent_dl_throughput_mbps",
    "recent_ul_pdr",
    "recent_ul_delay_ms",
    "recent_ul_throughput_mbps",
]
SCENARIO_FLAG_FEATURES = ["DonorUnavailableActive"]

LABEL = "BackhaulMode"
VALID_LABELS = {"TN_DIRECT", "SATELLITE_FALLBACK", "NO_BACKHAUL_AVAILABLE"}


def seed_group_from_name(name: str) -> str:
    match = re.search(r"seed(\d+)", name)
    if match:
        return f"seed{match.group(1)}"
    return name


def scenario_dirs(base_dir: Path) -> list[Path]:
    patterns = [
        "tn-uav_*donor-unavailable-no-sat-seed*",
        "tn-uav-satellite_*donor-unavailable-sat-seed*",
        "tn-uav_*distance-loss*no-sat*",
        "tn-uav-satellite_*distance-loss*sat*",
        "tn-uav_*ai-*no-sat*",
        "tn-uav-satellite_*ai-*sat*",
    ]
    dirs: list[Path] = []
    for pattern in patterns:
        dirs.extend(sorted(base_dir.glob(pattern)))
        dirs.extend(sorted(base_dir.glob(f"**/{pattern}")))
    unique_dirs = sorted(set(dirs))
    return [d for d in unique_dirs if (d / "xhaul-autonomy-trace.csv").exists()]


def read_qos_features(qos_file: Path) -> pd.DataFrame:
    if not qos_file.exists():
        return pd.DataFrame()

    qos = pd.read_csv(qos_file)
    if qos.empty:
        return pd.DataFrame()
    qos["Time"] = pd.to_numeric(qos["Time"], errors="coerce")
    for col in ["Delay", "Throughput", "PDR"]:
        qos[col] = pd.to_numeric(qos[col], errors="coerce")
    qos["DelayMs"] = qos["Delay"] * 1000.0

    grouped = []
    for direction, prefix in [("DL", "recent_dl"), ("UL", "recent_ul")]:
        sub = qos[qos["Dir"].str.upper() == direction].copy()
        if sub.empty:
            continue
        agg = (
            sub.groupby("Time")
            .agg(
                **{
                    f"{prefix}_pdr": ("PDR", "mean"),
                    f"{prefix}_delay_ms": ("DelayMs", lambda x: x[x > 0].mean()),
                    f"{prefix}_throughput_mbps": ("Throughput", "mean"),
                }
            )
            .reset_index()
        )
        grouped.append(agg)

    if not grouped:
        return pd.DataFrame()

    out = grouped[0]
    for g in grouped[1:]:
        out = out.merge(g, on="Time", how="outer")
    return out.sort_values("Time")


def nearest_qos_rows(xhaul: pd.DataFrame, qos: pd.DataFrame) -> pd.DataFrame:
    if qos.empty:
        for col in FEATURES:
            if col.startswith("recent_"):
                xhaul[col] = 0.0
        return xhaul

    x = xhaul.sort_values("Time")
    q = qos.sort_values("Time")
    x["Time"] = x["Time"].astype(float)
    q["Time"] = q["Time"].astype(float)
    merged = pd.merge_asof(x, q, on="Time", direction="nearest", tolerance=1.1)
    for col in FEATURES:
        if col.startswith("recent_") and col not in merged.columns:
            merged[col] = 0.0
    return merged


def load_dataset(base_dir: Path) -> pd.DataFrame:
    frames = []
    for d in scenario_dirs(base_dir):
        xhaul_file = d / "xhaul-autonomy-trace.csv"
        qos_file = d / "qos-vs-time.txt"
        xhaul = pd.read_csv(xhaul_file)
        if xhaul.empty or LABEL not in xhaul:
            print(f"[skip] unusable xHaul trace: {xhaul_file}")
            continue

        xhaul["Time"] = pd.to_numeric(xhaul["Time"], errors="coerce")
        xhaul = xhaul[xhaul[LABEL].isin(VALID_LABELS)].copy()
        if xhaul.empty:
            continue

        qos = read_qos_features(qos_file)
        merged = nearest_qos_rows(xhaul, qos)
        merged["ResultDir"] = d.name
        merged["SeedGroup"] = seed_group_from_name(d.name)
        merged["ScenarioKind"] = "satellite" if "tn-uav-satellite" in d.name else "no_satellite"
        frames.append(merged)
        print(f"[input] {d} rows={len(merged)}")

    if not frames:
        raise RuntimeError("No usable result folders found")

    data = pd.concat(frames, ignore_index=True)

    numeric_defaults = {
        "BestDonorDistanceM": -1.0,
        "XhaulConnected": 0.0,
        "XhaulChannelVariationDb": 0.0,
        "XhaulRsrpDbm": -999.0,
        "DonorUnavailableActive": 0.0,
        "SatBackhaulDlSnrDb": -999.0,
        "SatBackhaulUlSnrDb": -999.0,
        "SatBackhaulHealthy": 0.0,
        "UavSwitchingXappAvailable": 0.0,
        "recent_dl_pdr": 0.0,
        "recent_dl_delay_ms": 0.0,
        "recent_dl_throughput_mbps": 0.0,
        "recent_ul_pdr": 0.0,
        "recent_ul_delay_ms": 0.0,
        "recent_ul_throughput_mbps": 0.0,
    }
    for col, default in numeric_defaults.items():
        if col not in data:
            data[col] = default
        data[col] = pd.to_numeric(data[col], errors="coerce").fillna(default)

    return data


def split_data(data: pd.DataFrame, seed: int, split_mode: str, features: list[str]):
    y_raw = data[LABEL].astype(str)
    label_encoder = LabelEncoder()
    y = label_encoder.fit_transform(y_raw)
    x = data[features].copy()

    groups = data["SeedGroup"].astype(str) if "SeedGroup" in data else data["ResultDir"].astype(str)
    if split_mode == "group" and groups.nunique() >= 3:
        test_size = 1 if groups.nunique() <= 4 else 0.35
        splitter = GroupShuffleSplit(n_splits=50, test_size=test_size, random_state=seed)
        all_labels = set(np.unique(y))
        for train_idx, test_idx in splitter.split(x, y, groups=groups):
            if set(np.unique(y[train_idx])) == all_labels:
                return x.iloc[train_idx], x.iloc[test_idx], y[train_idx], y[test_idx], label_encoder
        print("[warn] no grouped split preserves all labels in training; using stratified row split")
    elif split_mode == "group":
        print(f"[warn] grouped split needs at least 3 seed groups; found {groups.nunique()}; using stratified row split")

    return (*train_test_split(x, y, test_size=0.30, random_state=seed, stratify=y), label_encoder)


def train_models(x_train, y_train, seed: int):
    models = {
        "decision_tree": DecisionTreeClassifier(
            max_depth=5,
            min_samples_leaf=5,
            class_weight="balanced",
            random_state=seed,
        ),
        "gradient_boosting": HistGradientBoostingClassifier(
            learning_rate=0.05,
            max_iter=200,
            max_leaf_nodes=15,
            l2_regularization=0.05,
            random_state=seed,
        ),
        "random_forest": RandomForestClassifier(
            n_estimators=300,
            max_depth=8,
            min_samples_leaf=3,
            class_weight="balanced",
            random_state=seed,
            n_jobs=-1,
        ),
    }
    if HAVE_XGBOOST:
        models["xgboost"] = XGBClassifier(
            n_estimators=250,
            max_depth=4,
            learning_rate=0.05,
            subsample=0.9,
            colsample_bytree=0.9,
            objective="multi:softprob",
            eval_metric="mlogloss",
            random_state=seed,
            n_jobs=-1,
        )
    return models


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", type=Path, default=Path("results/nr/tn-ntn"))
    parser.add_argument("--output-dir", type=Path, default=Path("results/ai/uav-switching-xapp"))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--model",
        choices=["gradient_boosting", "random_forest", "xgboost"],
        default="xgboost",
        help="Model to save as the active AI switching model.",
    )
    parser.add_argument(
        "--split-mode",
        choices=["stratified", "group"],
        default="stratified",
        help="Use stratified row split for small current data; use group split for many independent runs.",
    )
    parser.add_argument(
        "--include-outage-flag",
        action="store_true",
        help="Include DonorUnavailableActive as an explicit scenario flag. Use only for ablation/debugging.",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    features = FEATURES + SCENARIO_FLAG_FEATURES if args.include_outage_flag else FEATURES

    data = load_dataset(args.base_dir)
    dataset_csv = args.output_dir / "uav_switching_ai_dataset.csv"
    data.to_csv(dataset_csv, index=False)
    print(f"[saved] {dataset_csv}")
    print("[labels]", data[LABEL].value_counts().to_dict())

    x_train, x_test, y_train, y_test, label_encoder = split_data(
        data, args.seed, args.split_mode, features
    )
    models = train_models(x_train, y_train, args.seed)
    if args.model == "xgboost" and "xgboost" not in models:
        print("[warn] xgboost is not installed; saving gradient_boosting as the active model")
        args.model = "gradient_boosting"

    reports = {}
    confusion_matrices = {}
    comparison_rows = []
    for name, model in models.items():
        model.fit(x_train, y_train)
        start = time.perf_counter()
        pred = model.predict(x_test)
        elapsed_s = time.perf_counter() - start
        inference_ms_per_sample = (elapsed_s / max(len(x_test), 1)) * 1000.0
        label_ids = list(range(len(label_encoder.classes_)))
        report = classification_report(
            y_test,
            pred,
            labels=label_ids,
            target_names=label_encoder.classes_,
            output_dict=True,
            zero_division=0,
        )
        matrix = confusion_matrix(y_test, pred, labels=label_ids)
        reports[name] = report
        confusion_matrices[name] = matrix.tolist()
        comparison_rows.append(
            {
                "model": name,
                "accuracy": report["accuracy"],
                "macro_f1": report["macro avg"]["f1-score"],
                "weighted_f1": report["weighted avg"]["f1-score"],
                "inference_ms_per_sample": inference_ms_per_sample,
            }
        )
        print(f"\n=== {name} ===")
        print(
            classification_report(
                y_test,
                pred,
                labels=label_ids,
                target_names=label_encoder.classes_,
                zero_division=0,
            )
        )
        print(matrix)
        print(f"inference_ms_per_sample={inference_ms_per_sample:.6f}")

    active = models[args.model]
    model_path = args.output_dir / "uav_switching_xapp_model.joblib"
    label_path = args.output_dir / "uav_switching_xapp_labels.json"
    report_path = args.output_dir / "uav_switching_xapp_training_report.json"
    feature_path = args.output_dir / "uav_switching_xapp_feature_importance.csv"
    comparison_path = args.output_dir / "uav_switching_xapp_model_comparison.csv"

    joblib.dump({"model": active, "features": features, "label_encoder": label_encoder}, model_path)
    label_path.write_text(json.dumps(list(label_encoder.classes_), indent=2), encoding="utf-8")

    result = permutation_importance(active, x_test, y_test, n_repeats=10, random_state=args.seed)
    feature_importance = pd.DataFrame(
        {
            "feature": features,
            "importance_mean": result.importances_mean,
            "importance_std": result.importances_std,
        }
    ).sort_values("importance_mean", ascending=False)
    feature_importance.to_csv(feature_path, index=False)
    comparison = pd.DataFrame(comparison_rows).sort_values("accuracy", ascending=False)
    comparison.to_csv(comparison_path, index=False)

    report = {
        "active_model": args.model,
        "split_mode": args.split_mode,
        "num_samples": int(len(data)),
        "num_train": int(len(x_train)),
        "num_test": int(len(x_test)),
        "features": features,
        "include_outage_flag": bool(args.include_outage_flag),
        "labels": list(label_encoder.classes_),
        "reports": reports,
        "confusion_matrices": confusion_matrices,
        "note": (
            "This model is trained from current BackhaulMode decisions. It is an "
            "AI imitation baseline for the switching xApp, not yet a QoS-optimal "
            "policy. For PDR improvement, relabel samples using future-window QoS reward."
        ),
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"[saved] {model_path}")
    print(f"[saved] {label_path}")
    print(f"[saved] {report_path}")
    print(f"[saved] {feature_path}")
    print(f"[saved] {comparison_path}")
    print("\nModel comparison:")
    print(comparison.to_string(index=False))
    print(feature_importance.head(10).to_string(index=False))


if __name__ == "__main__":
    main()
