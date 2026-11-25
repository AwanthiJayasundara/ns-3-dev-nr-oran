# UAV backhaul predictor and ns-3 integration

This workflow adds an eight-input Random Forest that predicts sustained loss of the terrestrial donor link. The in-process ns-O-RAN Near-RT RIC uses the prediction alongside satellite availability, persistence, and dwell-time checks. The target is loss at all three future control epochs, 2, 4, and 6 seconds after inference. See the [predictor methodology](predictor_methodology.md) and [simulation and ML parameter table](simulation_ml_parameters.tex).

The historical 200-second clustered missions compared rule-based backhaul switching with a legacy 14-input backhaul-mode classifier. Both arms used the same RSRP-based UE handover controller. The mission traces and original paper figures retain their own provenance. The eight-input predictor was evaluated in the separate v10 integration scenario.

## Rebuild the offline evaluation and plots

From the repository root, use Python 3.12. The ten input xHaul trace files are already tracked under `results/nr/tn-ntn/ai-dataset-v2`. The training script builds 1,590 labelled rows and evaluates five leave-one-seed-out folds with grouping by simulation seed. It then exports the all-seed refit.

```bash
python3.12 -m venv .venv-uav-ai
source .venv-uav-ai/bin/activate
python -m pip install numpy==2.4.2 pandas==3.0.0 scikit-learn==1.8.0 \
    joblib==1.5.3 onnx==1.20.1 onnxruntime==1.24.1 skl2onnx==1.20.0 \
    matplotlib==3.10.8

reviewer_artifacts=$(mktemp -d)
python contrib/oran/examples/train-uav-sustained-outage-predictor.py \
    --base-dir results/nr/tn-ntn/ai-dataset-v2 \
    --output-dir "$reviewer_artifacts"
python contrib/oran/examples/plot-uav-sustained-outage-predictor.py \
    --artifact-dir "$reviewer_artifacts"
```

The temporary directory receives the dataset, grouped scores, ONNX export, and plots. The published [ONNX model](../../results/ai/uav-sustained-outage-v1/sustained_outage_rf_v1.onnx) and [schema](../../results/ai/uav-sustained-outage-v1/input_output_schema_v1.json) are the frozen artifacts used by the v10 predictor arm.

## Build the optimized ns-3 example

Use a Linux ns-3 build environment with CMake and SQLite development headers. Follow the [ns-O-RAN ONNX setup](../../contrib/oran/README.md#onnx) to install the native ONNX Runtime 1.14.1 and export `LIBONNXPATH` before configuring. The Python ONNX Runtime package above serves offline analysis.

```bash
./ns3 configure --build-profile=optimized --enable-logs --enable-examples
./ns3 build oran-nr-uav-xhaul-autonomy-example
```

## Route and packet-delivery check

The [v21 route contract](../../contrib/oran/examples/run-hybrid-sat-route-contract-v21.sh) exercises route actuation and packet delivery with one UE, one terrestrial cell, and one UAV cell. Run it from a build that has no existing `results/nr/tn-ntn/route-contract-v21` directory. The runner refuses to overwrite that directory.

```bash
bash contrib/oran/examples/run-hybrid-sat-route-contract-v21.sh
```

The historical v21 contract passed. A new execution writes its own report to `results/nr/tn-ntn/route-contract-v21/route-contract.json`.

## V10 fixed-association integration

Print the original matched rule and predictor command lines without starting ns-3.

```bash
DRY_RUN=1 bash contrib/oran/examples/run-uav-reviewer-integration-pair-v10.sh pair
```

The sealed v10 runner checks the original executable, shared library, source, route-contract report, and path-bearing manifest by SHA-256. Execution through this runner requires the original frozen environment. A fresh build can inspect the command lines through the dry run. The [frozen v10 source revision](https://github.com/AwanthiJayasundara/ns-3-dev-nr-oran/tree/5756d23c18ff71f17db6dbda715634e9cee87de9) predates the mobility instrumentation.

Both 100-UE arms completed under `RngSeed=1` and `RngRun=6`. The six matched-input checks passed, and offline replay reproduced all 42 online predictor probabilities and threshold decisions. The original overall validator verdict was **FAIL**. The audit corrected a stale control-horizon expectation separately. The anticipatory switch inside the baseline window and the additional late switch remained in the action accounting. The late switch had no future observations available to determine its target. With fixed UE association, handover success and UE ping-pong were inapplicable.

## Active RSRP mobility diagnostic

The separate rule-only mobility harness keeps backhaul fixed to healthy terrestrial service. It enables the ns-O-RAN RSRP handover controller and uses the [v2 mobility paths](mobility_corridors_v2.csv) and [random stream map](mobility_stream_map_v2.json).

```bash
bash contrib/oran/examples/run-uav-reviewer-mobility.sh dry-run

# Optional 100-UE run. Use a fresh absolute output path.
MOBILITY_OUTPUT_ROOT="$PWD/results/nr/tn-ntn/reviewer-mobility-reproduction" \
    bash contrib/oran/examples/run-uav-reviewer-mobility.sh integration-rule
```

The completed Gate 2 run finished in 6,601 seconds on the recorded host and passed all 22 integration checks. It reconciled five RRC starts with five successful completions, including four intended-target completions and one 10-second A-to-B-to-A reversal. These counts validate the event collection and reconciliation workflow in one rule-only run. The validator writes `gate2-validation.json` under the configured output directory.
