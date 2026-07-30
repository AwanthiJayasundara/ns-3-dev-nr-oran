#!/usr/bin/env python3
"""Export the trained UAV TN/NTN switching xApp model to ONNX.

The ns-3 example cannot load a Python joblib model directly. This script
converts the trained scikit-learn bundle produced by
train-uav-switching-xapp-ai.py into an ONNX model that can be evaluated from
C++ using ONNX Runtime.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import joblib
import numpy as np
import onnxruntime as ort
import pandas as pd
from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("results/ai/uav-switching-xapp-ai-dataset-v2/uav_switching_xapp_model.joblib"),
        help="Input joblib bundle from train-uav-switching-xapp-ai.py",
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("results/ai/uav-switching-xapp-ai-dataset-v2/uav_switching_ai_dataset.csv"),
        help="Dataset CSV used only for an export sanity check",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/ai/uav-switching-xapp-ai-dataset-v2/uav_switching_xapp_model.onnx"),
        help="Output ONNX model path",
    )
    parser.add_argument(
        "--metadata",
        type=Path,
        default=Path("results/ai/uav-switching-xapp-ai-dataset-v2/uav_switching_xapp_onnx_metadata.json"),
        help="Output JSON metadata path",
    )
    args = parser.parse_args()

    bundle = joblib.load(args.model)
    model = bundle["model"]
    features = list(bundle["features"])
    label_encoder = bundle["label_encoder"]

    initial_types = [("float_input", FloatTensorType([None, len(features)]))]
    options = {id(model): {"zipmap": False}}
    onnx_model = convert_sklearn(
        model,
        initial_types=initial_types,
        target_opset=12,
        options=options,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(onnx_model.SerializeToString())

    metadata = {
        "features": features,
        "labels": list(label_encoder.classes_),
        "input_name": "float_input",
        "output_note": (
            "The model returns a predicted label tensor and a probability tensor. "
            "The label IDs map through the labels array in this metadata file."
        ),
    }
    args.metadata.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print(f"[saved] {args.output}")
    print(f"[saved] {args.metadata}")
    print(f"[features] {len(features)}")
    print(f"[labels] {metadata['labels']}")

    if args.dataset.exists():
        data = pd.read_csv(args.dataset)
        for col in features:
            data[col] = pd.to_numeric(data[col], errors="coerce").fillna(0.0)
        sample = data[features].head(256).astype(np.float32).to_numpy()
        sklearn_pred = model.predict(sample)

        session = ort.InferenceSession(str(args.output), providers=["CPUExecutionProvider"])
        input_name = session.get_inputs()[0].name
        outputs = session.run(None, {input_name: sample})
        onnx_pred = outputs[0].astype(np.int64).reshape(-1)
        match = float(np.mean(onnx_pred == sklearn_pred) * 100.0)
        print(f"[check] ONNX/sklearn prediction match on first {len(sample)} samples: {match:.2f}%")


if __name__ == "__main__":
    main()
