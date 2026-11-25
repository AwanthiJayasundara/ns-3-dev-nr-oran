# Sustained donor-loss predictor methodology

The eight-input model predicts future terrestrial donor loss. The historical
14-input model learned current switching labels. Their datasets, ONNX schemas,
and validation results apply to their respective models.

## Dataset and target

The [training script](../../contrib/oran/examples/train-uav-sustained-outage-predictor.py)
uses ten archived xHaul traces from simulation seeds 1 through 5. Each seed
has a satellite and a no-satellite scenario. Each trace has 58 observations
per UAV for three UAVs, sampled every 2 seconds. A positive label requires
raw donor state `UNREACHABLE` at all three future observations at
`t+2`, `t+4`, and `t+6` seconds. The first two records of each UAV lack
complete history and the final three lack complete target observations.
The remaining dataset has 1,590 rows, of which 648 are positive.

## Inputs and preprocessing

The feature matrix is float32 and the target is int64. The trainer validates
finite inputs, binary connectivity, nonnegative state age, and exact
per-UAV cadence. It applies fixed disconnected sentinels and computes
differences from the preceding observations. The fitted model uses no feature
scaling or resampling.

| Order | Input | Definition |
|---:|---|---|
| 1 | `BestDonorDistanceM` | Current distance to the strongest eligible TN donor, in metres. Disconnected value −1. |
| 2 | `XhaulConnected` | 1 when an eligible terrestrial donor is present, otherwise 0. |
| 3 | `XhaulChannelVariationDb` | Current shadowing-minus-fading contribution, in dB. Disconnected value 0. |
| 4 | `XhaulRsrpDbm` | Current donor RSRP, in dBm. Disconnected value −999. |
| 5 | `RawXhaulStateAgeSec` | Time in the current raw donor state, in seconds. |
| 6 | `delta_distance_2s` | Distance at `t` minus distance at `t−2`, in metres. |
| 7 | `delta_rsrp_2s` | RSRP at `t` minus RSRP at `t−2`, in dB. |
| 8 | `delta_rsrp_4s` | RSRP at `t` minus RSRP at `t−4`, in dB. |

A difference is zero when either endpoint is disconnected. Online inference
requires both preceding observations and valid history. If history is
unavailable, the switching policy falls back to the rule.

## Training and evaluation

The Random Forest uses 300 trees, maximum depth 8, minimum leaf size 3,
balanced class weights, random state 7, and one worker. Five
leave-one-seed-out folds keep all observations from a seed together,
including both scenarios and all UAVs. Each fold uses 1,272 training rows and
318 test rows. Each record contributes once to the out-of-fold results. The
reported experiment used fixed hyperparameters and no separate validation
split.

The [pooled scores](../../results/ai/uav-sustained-outage-v1/grouped_oof_metrics_v1.json)
are 96.8553% accuracy and 96.7517% macro-F1. Mean seed macro-F1 is
94.1240%. The [per-seed scores](../../results/ai/uav-sustained-outage-v1/grouped_seed_metrics_v1.csv)
record the variation across held-out seeds.

After evaluation, the trainer refits the same model on all 1,590 rows for
deployment. The [schema](../../results/ai/uav-sustained-outage-v1/input_output_schema_v1.json)
specifies a float32 input of shape `[N,8]`, an int64 label vector, and float32
probabilities of shape `[N,2]`. Positive probability is column 1. The
[published ONNX](../../results/ai/uav-sustained-outage-v1/sustained_outage_rf_v1.onnx)
has SHA-256 `2f9bb76fe6d161cf8d4ab6ff5031647f45fa7cb6916d60463b4cffadeca1207f`.
The network runners retain this deployment identity. See the [workflow
README](README.md) for verified integration status and reproducibility commands.
