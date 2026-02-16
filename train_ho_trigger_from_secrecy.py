from pathlib import Path
import math
import argparse
import pandas as pd
import numpy as np

from sklearn.model_selection import GroupShuffleSplit
from sklearn.metrics import roc_auc_score, classification_report
from sklearn.ensemble import HistGradientBoostingClassifier

import joblib
from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType


def parse_secrecy_kv(path: Path) -> pd.DataFrame:
    """
    Parses lines like:
    2,UE,0,cell,2,ueSinrLin,228.483,ueSinrDb,23.5885,...,secrecy,7.84,outage,0
    into a normal table with numeric columns.
    """
    rows = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("time,type,id"):
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 7:
                continue

            try:
                t = float(parts[0])
                typ = parts[1]
                ueId = int(parts[2])
            except:
                continue

            d = {"time": t, "type": typ, "ueId": ueId}

            i = 3
            while i + 1 < len(parts):
                k = parts[i]
                v = parts[i + 1]
                i += 2

                if k in {"cell","ueSinrDb","eveSinrDb","secrecy","outage","ueSinrLin","eveSinrLin"}:
                    if isinstance(v, str) and v.lower() == "-inf":
                        d[k] = -math.inf
                    else:
                        try:
                            if k == "cell":
                                d[k] = int(float(v))
                            elif k == "outage":
                                d[k] = int(float(v))
                            else:
                                d[k] = float(v)
                        except:
                            pass

            if d.get("type") == "UE" and "cell" in d and "secrecy" in d:
                rows.append(d)

    df = pd.DataFrame(rows).sort_values(["ueId", "time"]).reset_index(drop=True)

    # Make model-friendly: replace -inf eveSinrDb with a floor
    if "eveSinrDb" in df.columns:
        df["eveSinrDb"] = df["eveSinrDb"].replace([-math.inf], -100.0)

    return df


def build_dataset(df: pd.DataFrame, mgmt: float) -> tuple[pd.DataFrame, list[str]]:
    """
    Build HO_next label from cell changes between consecutive management ticks.
    Adds lag and delta features so LM can reproduce them online.
    """

    # Quantize to decision grid (2s) in case of float noise
    df = df.copy()
    df["t_dec"] = (np.floor(df["time"] / mgmt) * mgmt).astype(float)

    # Keep last row per UE per decision time (if duplicates exist)
    df = df.sort_values(["ueId", "t_dec", "time"]).groupby(["ueId", "t_dec"], as_index=False).tail(1)
    df = df.sort_values(["ueId", "t_dec"]).reset_index(drop=True)

    # Label: HO in next tick if cell changes
    df["cell_next"] = df.groupby("ueId")["cell"].shift(-1)
    df["HO_next"] = (df["cell_next"] != df["cell"]).astype(int)

    # Lag features
    for col in ["ueSinrDb", "eveSinrDb", "secrecy", "outage"]:
        if col in df.columns:
            df[f"{col}_prev"] = df.groupby("ueId")[col].shift(1)

    # Delta features
    df["d_ueSinrDb"] = df["ueSinrDb"] - df["ueSinrDb_prev"]
    df["d_secrecy"]  = df["secrecy"]  - df["secrecy_prev"]

    # Drop rows with missing prev/next
    df = df.dropna().reset_index(drop=True)

    feature_cols = [
        "ueSinrDb", "eveSinrDb", "secrecy", "outage",
        "ueSinrDb_prev", "secrecy_prev", "outage_prev",
        "d_ueSinrDb", "d_secrecy",
    ]
    feature_cols = [c for c in feature_cols if c in df.columns]

    return df, feature_cols


def train_and_export(df: pd.DataFrame, feature_cols: list[str], out_prefix: str):
    X = df[feature_cols].astype(np.float32).values
    y = df["HO_next"].astype(int).values
    groups = df["ueId"].values  # prevent UE leakage between train/test

    gss = GroupShuffleSplit(n_splits=1, test_size=0.25, random_state=7)
    train_idx, test_idx = next(gss.split(X, y, groups=groups))

    clf = HistGradientBoostingClassifier(max_depth=4, learning_rate=0.1)
    clf.fit(X[train_idx], y[train_idx])

    proba = clf.predict_proba(X[test_idx])[:, 1]
    pred = (proba > 0.5).astype(int)

    print("HO_next rate:", y.mean())
    print("AUC:", roc_auc_score(y[test_idx], proba))
    print(classification_report(y[test_idx], pred, digits=4))

    # Save joblib bundle (handy for debugging)
    joblib.dump({"model": clf, "features": feature_cols}, f"{out_prefix}.joblib")
    print("Saved:", f"{out_prefix}.joblib")

    # Export ONNX (what LM needs)
    initial_type = [("input", FloatTensorType([None, len(feature_cols)]))]

    options = {id(clf): {"zipmap": False}}  
    onnx_model = convert_sklearn(clf, initial_types=initial_type, options=options,target_opset=18)


    onnx_path = f"{out_prefix}.onnx"
    with open(onnx_path, "wb") as f:
        f.write(onnx_model.SerializeToString())

    print("Saved:", onnx_path)
    print("IMPORTANT: feature order for runtime LM input vector:")
    for i, name in enumerate(feature_cols):
        print(f"  {i}: {name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secrecy", type=Path, required=True)
    ap.add_argument("--mgmt", type=float, default=2.0)
    ap.add_argument("--out_prefix", type=str, default="model_tn")
    args = ap.parse_args()

    df = parse_secrecy_kv(args.secrecy)
    print("Parsed rows:", len(df), "UEs:", df["ueId"].nunique(), "times:", df["time"].nunique())

    df_ds, feature_cols = build_dataset(df, args.mgmt)
    df_ds[["t_dec","ueId","cell","HO_next"] + feature_cols].to_csv(f"{args.out_prefix}_dataset.csv", index=False)
    print("Saved dataset:", f"{args.out_prefix}_dataset.csv")

    train_and_export(df_ds, feature_cols, args.out_prefix)


if __name__ == "__main__":
    main()
