#!/usr/bin/env python3
"""Generate prediction-accuracy figures for UAV hotspot forecasting."""

from __future__ import annotations

from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


RESULT_DIR = Path("results/nr/tn-ntn/ml_uav_final")
OUT_DIR = Path("docs/figures/cnn-paper-plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def pretty_model(name: str) -> str:
    return {
        "persistence": "Persistence",
        "mlp": "MLP",
        "cnn": "CNN",
        "gru": "GRU",
    }.get(name, name.upper())


def main() -> None:
    style = Path("latex_style.mplstyle")
    if style.exists():
        plt.style.use(str(style))

    src = RESULT_DIR / "predictor_comparison.csv"
    df = pd.read_csv(src)
    df["Predictor"] = df["model"].map(pretty_model)

    # Keep metrics that directly answer "is the prediction accurate?"
    out = df[
        [
            "Predictor",
            "rmse",
            "mae",
            "hit_at_k",
            "precision_at_k",
            "recall_at_k",
            "centroid_error_m",
            "count_mae",
        ]
    ].copy()
    out.to_csv(OUT_DIR / "uav_prediction_accuracy_summary.csv", index=False)

    order = ["Persistence", "MLP", "GRU", "CNN"]
    plot_df = out.set_index("Predictor").loc[order].reset_index()

    colors = {
        "Persistence": "#7f7f7f",
        "MLP": "#9467bd",
        "GRU": "#2ca02c",
        "CNN": "#1f77b4",
    }
    bar_colors = [colors[p] for p in plot_df["Predictor"]]

    fig, axes = plt.subplots(1, 3, figsize=(12.0, 4.2), constrained_layout=False)
    fig.subplots_adjust(left=0.07, right=0.99, bottom=0.20, top=0.92, wspace=0.33)

    ax = axes[0]
    ax.bar(plot_df["Predictor"], plot_df["rmse"], color=bar_colors, edgecolor="black", linewidth=0.5)
    ax.set_ylabel("RMSE", fontsize=18)
    ax.set_xlabel("Predictor", fontsize=16)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.tick_params(axis="x", labelsize=13, rotation=18)
    ax.tick_params(axis="y", labelsize=13)
    for i, v in enumerate(plot_df["rmse"]):
        ax.text(i, v + 0.015, f"{v:.3f}", ha="center", va="bottom", fontsize=10)
    ax.text(0.5, -0.33, "(a)", transform=ax.transAxes, ha="center", va="center", fontsize=17)

    ax = axes[1]
    ax.bar(plot_df["Predictor"], 100 * plot_df["hit_at_k"], color=bar_colors, edgecolor="black", linewidth=0.5)
    ax.set_ylabel(r"Hit@5 (\%)", fontsize=18)
    ax.set_xlabel("Predictor", fontsize=16)
    ax.set_ylim(0, 110)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.tick_params(axis="x", labelsize=13, rotation=18)
    ax.tick_params(axis="y", labelsize=13)
    for i, v in enumerate(100 * plot_df["hit_at_k"]):
        ax.text(i, v + 2, f"{v:.1f}", ha="center", va="bottom", fontsize=10)
    ax.text(0.5, -0.33, "(b)", transform=ax.transAxes, ha="center", va="center", fontsize=17)

    ax = axes[2]
    ax.bar(
        plot_df["Predictor"],
        plot_df["centroid_error_m"],
        color=bar_colors,
        edgecolor="black",
        linewidth=0.5,
    )
    ax.set_ylabel("Centroid error (m)", fontsize=18)
    ax.set_xlabel("Predictor", fontsize=16)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.tick_params(axis="x", labelsize=13, rotation=18)
    ax.tick_params(axis="y", labelsize=13)
    for i, v in enumerate(plot_df["centroid_error_m"]):
        ax.text(i, v + 18, f"{v:.0f}", ha="center", va="bottom", fontsize=10)
    ax.text(0.5, -0.33, "(c)", transform=ax.transAxes, ha="center", va="center", fontsize=17)

    for ext in ("png", "pdf"):
        fig.savefig(
            OUT_DIR / f"uav_prediction_accuracy_comparison.{ext}",
            dpi=300,
            bbox_inches="tight",
            pad_inches=0.03,
        )
    plt.close(fig)

    # A second compact figure showing training/validation loss variance over epochs.
    history_path = RESULT_DIR / "training_history.csv"
    if history_path.exists():
        hist = pd.read_csv(history_path)
        hist = hist[hist["epoch"] > 0].copy()
        hist["Predictor"] = hist["model"].map(pretty_model)
        fig, ax = plt.subplots(figsize=(8.8, 4.4), constrained_layout=False)
        fig.subplots_adjust(left=0.095, right=0.985, bottom=0.16, top=0.96)
        for predictor, sub in hist.groupby("Predictor"):
            if predictor not in colors:
                continue
            sub = sub.sort_values("epoch")
            ax.plot(sub["epoch"], sub["val_loss"], label=f"{predictor} validation", color=colors[predictor], linewidth=2.4)
            ax.plot(sub["epoch"], sub["train_loss"], color=colors[predictor], linestyle="--", linewidth=1.7, alpha=0.6)
        ax.set_xlabel("Epoch", fontsize=18)
        ax.set_ylabel("Weighted MSE loss", fontsize=18)
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.legend(loc="upper right", fontsize=11, frameon=True)
        ax.tick_params(axis="both", labelsize=13)
        for ext in ("png", "pdf"):
            fig.savefig(
                OUT_DIR / f"uav_prediction_training_validation_loss.{ext}",
                dpi=300,
                bbox_inches="tight",
                pad_inches=0.03,
            )
        plt.close(fig)

    print("[saved]", OUT_DIR / "uav_prediction_accuracy_comparison.png")
    print("[saved]", OUT_DIR / "uav_prediction_accuracy_summary.csv")
    if (OUT_DIR / "uav_prediction_training_validation_loss.png").exists():
        print("[saved]", OUT_DIR / "uav_prediction_training_validation_loss.png")

    print(out.to_string(index=False))


if __name__ == "__main__":
    main()
