#!/usr/bin/env python3
"""Apply the trained UAV switching xApp AI model to result folders."""

from __future__ import annotations

import argparse
from pathlib import Path

import joblib
import pandas as pd

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


def read_qos_features(qos_file: Path) -> pd.DataFrame:
    if not qos_file.exists():
        return pd.DataFrame()
    qos = pd.read_csv(qos_file)
    if qos.empty:
        return pd.DataFrame()
    qos["Time"] = pd.to_numeric(qos["Time"], errors="coerce").astype(float)
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
    x = xhaul.sort_values("Time").copy()
    q = qos.sort_values("Time").copy()
    x["Time"] = x["Time"].astype(float)
    q["Time"] = q["Time"].astype(float)
    merged = pd.merge_asof(x, q, on="Time", direction="nearest", tolerance=1.1)
    for col in FEATURES:
        if col.startswith("recent_") and col not in merged:
            merged[col] = 0.0
    return merged


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dirs", nargs="+", type=Path)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("results/ai/uav-switching-xapp/uav_switching_xapp_model.joblib"),
    )
    parser.add_argument("--output-name", default="ai-switching-decisions.csv")
    args = parser.parse_args()

    bundle = joblib.load(args.model)
    model = bundle["model"]
    label_encoder = bundle["label_encoder"]
    features = bundle.get("features", FEATURES)

    for result_dir in args.result_dirs:
        xhaul_file = result_dir / "xhaul-autonomy-trace.csv"
        qos_file = result_dir / "qos-vs-time.txt"
        if not xhaul_file.exists():
            print(f"[skip] missing {xhaul_file}")
            continue
        xhaul = pd.read_csv(xhaul_file)
        if xhaul.empty:
            print(f"[skip] empty {xhaul_file}")
            continue
        xhaul["Time"] = pd.to_numeric(xhaul["Time"], errors="coerce")
        qos = read_qos_features(qos_file)
        data = nearest_qos_rows(xhaul, qos)
        for col in features:
            if col not in data:
                data[col] = 0.0
            data[col] = pd.to_numeric(data[col], errors="coerce").fillna(0.0)

        pred = model.predict(data[features])
        data["AiBackhaulMode"] = label_encoder.inverse_transform(pred)
        data["AiMatchesRule"] = data["AiBackhaulMode"].astype(str) == data["BackhaulMode"].astype(str)
        keep = [
            "Time",
            "DeploymentMode",
            "UavIndex",
            "UavCellId",
            "XhaulRsrpDbm",
            "RawXhaulState",
            "RawXhaulStateAgeSec",
            "XhaulState",
            "DonorUnavailableActive",
            "SatBackhaulDlSnrDb",
            "SatBackhaulUlSnrDb",
            "SatBackhaulHealthy",
            "BackhaulMode",
            "AiBackhaulMode",
            "AiMatchesRule",
            "NormalUeHandoverAllowed",
        ]
        keep = [col for col in keep if col in data]
        out = result_dir / args.output_name
        data[keep].to_csv(out, index=False)
        match_rate = data["AiMatchesRule"].mean() * 100.0 if "BackhaulMode" in data else float("nan")
        print(f"[saved] {out} match_rate={match_rate:.2f}%")


if __name__ == "__main__":
    main()
