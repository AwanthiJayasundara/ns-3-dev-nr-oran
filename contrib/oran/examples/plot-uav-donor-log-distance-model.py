#!/usr/bin/env python3
"""Plot the UAV-to-TN donor log-distance path-loss proxy.

The figure is explanatory: it shows how the donor RSRP used by the switching
xApp can naturally decrease as the UAV moves away from the terrestrial donor.
It uses the same default values as the final distance-loss scripts.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT_DIR = Path("docs/figures")
OUT_PREFIX = OUT_DIR / "uav_donor_log_distance_pathloss_model"


def fspl_db(frequency_hz: float, distance_m: np.ndarray) -> np.ndarray:
    f_ghz = frequency_hz / 1e9
    d = np.maximum(distance_m, 1.0)
    return 32.4 + 20.0 * np.log10(f_ghz) + 20.0 * np.log10(d)


def log_distance_pathloss_db(
    frequency_hz: float,
    distance_m: np.ndarray,
    exponent: float,
    reference_distance_m: float,
) -> np.ndarray:
    d0 = max(reference_distance_m, 1.0)
    d = np.maximum(distance_m, 1.0)
    ref_loss = fspl_db(frequency_hz, np.array([d0]))[0]
    return np.where(d <= d0, ref_loss, ref_loss + 10.0 * exponent * np.log10(d / d0))


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        plt.style.use("./latex_style.mplstyle")
    except OSError:
        pass

    rng = np.random.default_rng(7)
    tx_power_dbm = 46.0
    frequency_hz = 4.0e9
    exponent = 4.8
    reference_distance_m = 100.0
    threshold_dbm = -110.0
    shadowing_std_db = 6.0
    fading_std_db = 4.0

    distance_m = np.linspace(100.0, 20000.0, 240)
    fspl = fspl_db(frequency_hz, distance_m)
    log_pl = log_distance_pathloss_db(
        frequency_hz, distance_m, exponent, reference_distance_m
    )
    shadowing_db = rng.normal(0.0, shadowing_std_db, size=distance_m.size)
    fading_loss_db = np.abs(rng.normal(0.0, fading_std_db, size=distance_m.size))
    variation_db = shadowing_db - fading_loss_db
    log_pl_with_variation = log_pl - variation_db
    rsrp_dbm = tx_power_dbm - log_pl_with_variation

    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.2), sharex=True)
    ax0, ax1 = axes

    ax0.plot(distance_m / 1000.0, fspl, label="Free-space path loss", lw=1.8)
    ax0.plot(
        distance_m / 1000.0,
        log_pl,
        label=fr"Log-distance path loss, $n={exponent}$",
        lw=1.8,
    )
    ax0.plot(
        distance_m / 1000.0,
        log_pl_with_variation,
        label="Log-distance + shadowing/fading",
        lw=1.2,
        alpha=0.85,
    )
    ax0.set_ylabel("Path loss (dB)")
    ax0.grid(True, alpha=0.35)
    ax0.legend(loc="upper left", fontsize=8)

    ax1.plot(distance_m / 1000.0, rsrp_dbm, color="#2f6f9f", lw=1.3)
    ax1.axhline(
        threshold_dbm,
        color="#b33a3a",
        ls="--",
        lw=1.3,
        label=fr"TN donor usable threshold ({threshold_dbm:.0f} dBm)",
    )
    ax1.fill_between(
        distance_m / 1000.0,
        rsrp_dbm,
        threshold_dbm,
        where=rsrp_dbm < threshold_dbm,
        color="#b33a3a",
        alpha=0.12,
        interpolate=True,
        label="Satellite fallback region",
    )
    ax1.set_xlabel("UAV-to-TN donor distance (km)")
    ax1.set_ylabel("Estimated donor RSRP (dBm)")
    ax1.grid(True, alpha=0.35)
    ax1.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    for ext in ("png", "pdf"):
        out = OUT_PREFIX.with_suffix(f".{ext}")
        fig.savefig(out, dpi=300, bbox_inches="tight")
        print(f"[saved] {out}")

    csv_out = OUT_PREFIX.with_suffix(".csv")
    data = np.column_stack(
        [distance_m, fspl, log_pl, log_pl_with_variation, variation_db, rsrp_dbm]
    )
    np.savetxt(
        csv_out,
        data,
        delimiter=",",
        header=(
            "distance_m,fspl_db,log_distance_pathloss_db,"
            "log_distance_pathloss_with_variation_db,variation_db,rsrp_dbm"
        ),
        comments="",
    )
    print(f"[saved] {csv_out}")


if __name__ == "__main__":
    main()
