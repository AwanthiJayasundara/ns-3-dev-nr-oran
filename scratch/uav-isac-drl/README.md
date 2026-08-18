# UAV–ISAC DRL implementation

This directory preserves the released random-forest method as a benchmark and
adds a genuine sequential Double-DQN environment and agent.

## What was recoverable

The upstream repository was audited at commit
`8f5ce9977b37533e4fabffcc505751cb52998713`. It has one branch, no tags, and no
CSV/model artifacts. No `run_summary*.csv`, `xapp_kpi_log.csv`, or
`offline_rf_model.pkl` matching the thesis was present in the supplied
workspace or Downloads. Consequently, the exact 300+ thesis results and their
per-run commands are not recoverable from the available artifacts. Newly
generated datasets record every configuration and command.

## Setup and tests

```bash
.venv/bin/python -m pip install -r scratch/uav-isac-drl/requirements.txt
./ns3 build uav-isac-drl -j 4
.venv/bin/python -m unittest discover \
  -s scratch/uav-isac-drl/tests -p 'test_*.py' -v
```

## Random-policy ns-3 smoke test

```bash
.venv/bin/python scratch/uav-isac-drl/random_policy_smoke.py \
  --episodes 1 --seed 1001 --simulation-time 40 \
  --output results/uav-isac-drl/random-smoke-001
```

Episode directories are immutable: the wrapper refuses to overwrite a prior
run. Use a new output directory for a new campaign.

## RF benchmark

Generate a new, reproducible 40-run baseline:

```bash
.venv/bin/python scratch/uav-isac-drl/generate_rf_dataset.py \
  --runs 40 --sampling-seed 20260818 --simulation-time 300
```

Fit the released RF method and save its model/candidates:

```bash
.venv/bin/python scratch/uav-isac-drl/offline_rf_benchmark.py \
  --csv results/uav-isac-drl/rf-baseline/run_summary.csv \
  --output-dir results/uav-isac-drl/rf-model
```

## DQN verification and training

Fast mock training:

```bash
.venv/bin/python scratch/uav-isac-drl/train_dqn.py \
  --environment mock --episodes 100 --learning-starts 64 --batch-size 32 \
  --output results/uav-isac-drl/mock-training
```

Short ns-3 training campaign:

```bash
.venv/bin/python scratch/uav-isac-drl/train_dqn.py \
  --environment ns3 --episodes 20 --simulation-time 60 \
  --learning-starts 100 --batch-size 64 \
  --output results/uav-isac-drl/short-training
```

Full training should use disjoint, predeclared training/validation/test seeds
and multiple independent agent seeds. Do not treat the short campaign as a
paper result.

An example full candidate (1,000 episodes x 30 decisions) is:

```bash
.venv/bin/python scratch/uav-isac-drl/train_dqn.py \
  --environment ns3 --episodes 1000 --seed 31 --simulation-time 300 \
  --learning-starts 2000 --batch-size 128 --epsilon-decay-steps 30000 \
  --target-update-interval 1000 --checkpoint-every 25 \
  --output results/uav-isac-drl/full/agent-seed-31
```

Here the training simulator seeds are deterministically `3100000` through
`3100999`. Train other predeclared hyperparameter candidates into separate
directories. This debug build is slow, so measure one episode and schedule the
full campaign as a batch job; never replace it with the smoke run in a paper.

Select only among preserved candidate checkpoints using validation seeds:

```bash
.venv/bin/python scratch/uav-isac-drl/select_checkpoint.py \
  results/uav-isac-drl/short-training/checkpoint-episode-*.pt \
  --validation-seeds 7001,7002,7003,7004,7005 \
  --output results/uav-isac-drl/validation/selection.csv
```

The resulting `selection.json` records the frozen checkpoint. Do not select a
model by `best-training-return.pt`; that file exists only for diagnostics.

## Equal-seed evaluation

```bash
.venv/bin/python scratch/uav-isac-drl/evaluate_policies.py \
  --methods static,random,rule,dqn \
  --checkpoint <selected_checkpoint_from_selection.json> \
  --seeds 9001,9002,9003,9004,9005 \
  --output results/uav-isac-drl/comparison.csv
```

Add `rf-static` and `--rf-candidate <CSV>` after generating the RF candidate.
All final paper comparisons must use a frozen DQN and unseen matched seeds.

For the paper, use more than one unseen test seed, for example:

```bash
.venv/bin/python scratch/uav-isac-drl/evaluate_policies.py \
  --methods static,rf-static,random,rule,dqn \
  --checkpoint <frozen_validation_selected_checkpoint> \
  --rf-candidate results/uav-isac-drl/rf-model/offline_suggested_candidates.csv \
  --seeds 9001,9002,9003,9004,9005,9006,9007,9008,9009,9010 \
  --simulation-time 300 \
  --output results/uav-isac-drl/final/comparison.csv
```

Do not inspect or rerun individual test seeds while choosing a model. Report
paired per-seed differences and confidence intervals, not only a pooled mean.

## Evaluation output semantics

`comparison.csv` now separates two different quantities that must not be
mixed in a paper:

- `episode_*` columns are whole-episode KPIs read from `run_summary.csv` and
  are the primary scientific outcomes.
- `final_window_*` columns describe only the last control window and are kept
  for trajectory diagnosis.
- `terminated`, `truncated`, `mission_duration_s`, `mission_completed`, and
  `battery_terminated` record whether a nominal-duration mission finished.
- `effective_control_actions` counts commands that actually changed the
  controlled state; `clipped_no_effect_actions` counts non-zero commands that
  were already at a bound or otherwise had no observable effect.

The evaluator writes Student-t 95% confidence intervals in
`comparison-summary.csv`,
`comparison-paired-vs-static.csv`, and
`comparison-paired-dqn-vs-baselines.csv`. The last file directly compares DQN
with every included baseline on matched seeds, including the rule policy.

Older comparison files can be corrected from preserved episode artifacts
without rerunning ns-3:

```bash
.venv/bin/python scratch/uav-isac-drl/reprocess_comparison.py \
  --input results/uav-isac-drl/pilot-v2/comparison/comparison.csv \
  --output results/uav-isac-drl/pilot-v2/comparison/comparison-corrected.csv
```

The corrected three-seed pilot is an analysis check, not a final performance
campaign. Its DQN mean return is 41.3591 versus 41.1188 for the rule baseline,
and only 37 of 87 DQN control attempts changed the controlled state. This is
not enough evidence for a DRL improvement or significance claim.
