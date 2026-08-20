#!/usr/bin/env python3
"""Prepare a reproducible Random-Forest surrogate study for the RF-only paper.

This script deliberately does not import or train the DQN.  It reads completed
ns-3 runs, defines normalisers from that design dataset, fits a surrogate for a
corrected energy-aware utility, searches the declared parameter space, and
writes one frozen candidate for subsequent equal-seed ns-3 evaluation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
from sklearn.dummy import DummyRegressor
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.model_selection import KFold, cross_val_predict

from offline_rf_benchmark import BOUNDS, FEATURES, sample_candidates


METRIC_COLUMNS = {
    "throughput": "avgThrMbps",
    "rmse": "RMSEm",
    "delay": "avgDelayMs",
    "energy": "EtotJ",
}
UAV_ENERGY_COLUMNS = [f"UAV{i}_Energy_J" for i in range(4)]
WEIGHTS = {
    "pdet": 3.0,
    "throughput": 2.0,
    "rmse": 2.0,
    "delay": 1.0,
    "energy": 1.0,
}


def robust_limits(frame: pd.DataFrame) -> dict[str, dict[str, float]]:
    """Freeze 5th/95th-percentile limits from the design dataset."""
    limits: dict[str, dict[str, float]] = {}
    for name, column in METRIC_COLUMNS.items():
        lo, hi = frame[column].quantile([0.05, 0.95]).astype(float)
        if not hi > lo:
            raise ValueError(f"Cannot normalise constant metric {column}")
        limits[name] = {"lower": float(lo), "upper": float(hi)}
    return limits


def scale(values: pd.Series, limits: dict[str, float]) -> pd.Series:
    denominator = limits["upper"] - limits["lower"]
    return ((values - limits["lower"]) / denominator).clip(0.0, 1.0)


def corrected_utility(
    frame: pd.DataFrame, limits: dict[str, dict[str, float]]
) -> pd.Series:
    """Higher is better; RMSE, delay, and energy enter as penalties."""
    return (
        WEIGHTS["pdet"] * frame["Pdet"].clip(0.0, 1.0)
        + WEIGHTS["throughput"]
        * scale(frame[METRIC_COLUMNS["throughput"]], limits["throughput"])
        - WEIGHTS["rmse"] * scale(frame[METRIC_COLUMNS["rmse"]], limits["rmse"])
        - WEIGHTS["delay"] * scale(frame[METRIC_COLUMNS["delay"]], limits["delay"])
        - WEIGHTS["energy"] * scale(frame[METRIC_COLUMNS["energy"]], limits["energy"])
    )


def make_model(seed: int) -> RandomForestRegressor:
    return RandomForestRegressor(
        n_estimators=500,
        max_depth=10,
        min_samples_leaf=2,
        max_features="sqrt",
        random_state=seed,
        n_jobs=-1,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--candidate-samples", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=20260819)
    parser.add_argument(
        "--candidate-energy-limit",
        type=float,
        default=95_000.0,
        help="Conservative predicted per-UAV limit; final simulations use 100 kJ.",
    )
    args = parser.parse_args()

    frame = pd.read_csv(args.csv).dropna(
        subset=FEATURES + list(METRIC_COLUMNS.values()) + UAV_ENERGY_COLUMNS + ["Pdet"]
    ).copy()
    if len(frame) < 50:
        raise ValueError("At least 50 complete simulator runs are required")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    limits = robust_limits(frame)
    frame["Jcorrected"] = corrected_utility(frame, limits)
    frame["battery_feasible_100kJ"] = (
        frame[UAV_ENERGY_COLUMNS].max(axis=1) < 100_000.0
    )
    frame.to_csv(args.output_dir / "design_runs_with_corrected_utility.csv", index=False)

    X = frame[FEATURES]
    y = frame["Jcorrected"]
    cv = KFold(n_splits=5, shuffle=True, random_state=args.seed)

    model = make_model(args.seed)
    cv_prediction = cross_val_predict(model, X, y, cv=cv, n_jobs=1)
    dummy_prediction = cross_val_predict(DummyRegressor(strategy="mean"), X, y, cv=cv)
    model.fit(X, y)
    train_prediction = model.predict(X)

    energy_models: dict[str, RandomForestRegressor] = {}
    for index, column in enumerate(UAV_ENERGY_COLUMNS):
        energy_model = make_model(args.seed + index + 1)
        energy_model.fit(X, frame[column])
        energy_models[column] = energy_model

    candidates = sample_candidates(n=args.candidate_samples, seed=args.seed)
    candidates["predicted_Jcorrected"] = model.predict(candidates[FEATURES])
    for column, energy_model in energy_models.items():
        candidates[f"predicted_{column}"] = energy_model.predict(candidates[FEATURES])

    predicted_energy_columns = [f"predicted_{column}" for column in UAV_ENERGY_COLUMNS]
    candidates["predicted_max_uav_energy_j"] = candidates[predicted_energy_columns].max(axis=1)
    feasible = candidates[
        candidates["predicted_max_uav_energy_j"] <= args.candidate_energy_limit
    ].sort_values("predicted_Jcorrected", ascending=False)
    if feasible.empty:
        raise RuntimeError("No candidate passed the conservative energy screen")

    top = feasible.head(20).copy()
    selected = top.head(1).copy()
    top.to_csv(args.output_dir / "top_candidates.csv", index=False)
    selected.to_csv(args.output_dir / "selected-candidate.csv", index=False)

    importance = pd.DataFrame(
        {"feature": FEATURES, "importance": model.feature_importances_}
    ).sort_values("importance", ascending=False)
    importance.to_csv(args.output_dir / "feature_importance.csv", index=False)

    metrics = {
        "method": "Random-Forest surrogate-assisted configuration search (not RL)",
        "design_runs": int(len(frame)),
        "features": len(FEATURES),
        "candidate_samples": args.candidate_samples,
        "candidate_search_seed": args.seed,
        "cross_validation": "shuffled 5-fold; fixed random seed",
        "train_mae": float(mean_absolute_error(y, train_prediction)),
        "train_r2": float(r2_score(y, train_prediction)),
        "cv_mae": float(mean_absolute_error(y, cv_prediction)),
        "cv_r2": float(r2_score(y, cv_prediction)),
        "dummy_cv_mae": float(mean_absolute_error(y, dummy_prediction)),
        "feasible_design_runs_100kJ": int(frame["battery_feasible_100kJ"].sum()),
        "infeasible_design_runs_100kJ": int((~frame["battery_feasible_100kJ"]).sum()),
        "candidate_predicted_utility": float(selected.iloc[0]["predicted_Jcorrected"]),
        "candidate_predicted_max_uav_energy_j": float(
            selected.iloc[0]["predicted_max_uav_energy_j"]
        ),
        "candidate_energy_screen_j": args.candidate_energy_limit,
        "final_equal_seed_evaluation_complete": False,
    }
    (args.output_dir / "model_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "normalisation.json").write_text(
        json.dumps(
            {
                "source": "5th and 95th percentiles of the 300-run design dataset",
                "limits": limits,
                "weights": WEIGHTS,
                "packet_loss": "reported separately; constant zero in all design runs",
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    joblib.dump(
        {
            "utility_model": model,
            "energy_models": energy_models,
            "features": FEATURES,
            "limits": limits,
            "weights": WEIGHTS,
        },
        args.output_dir / "rf-paper-model.joblib",
    )

    print(json.dumps(metrics, indent=2))
    print("\nSelected candidate:")
    print(selected[["predicted_Jcorrected", "predicted_max_uav_energy_j"] + FEATURES].to_string(index=False))


if __name__ == "__main__":
    main()
