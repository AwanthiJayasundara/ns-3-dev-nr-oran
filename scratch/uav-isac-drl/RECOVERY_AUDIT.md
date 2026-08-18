# Thesis dataset and command recovery audit

Audit date: 2026-08-18. This file separates recovered evidence from newly
constructed reproducibility machinery.

## Recovered evidence

- Public repository: `SeunOdusole180/uav-isac-livestock-monitoring`
- Audited commit: `8f5ce9977b37533e4fabffcc505751cb52998713`
- Commit timestamp: `2026-04-19T02:14:08+01:00`
- Available history: one commit on `main`; no tags or additional branches.
- The repository contains nine source/documentation files. It contains no CSV,
  pickle, shell script, experiment manifest, or recorded invocation.
- The supplied thesis PDF has no embedded files.
- Searches of the supplied workspace and Downloads found no
  `run_summary*.csv`, `xapp_kpi_log.csv`, or `offline_rf_model.pkl` belonging to
  this study.

Artifact SHA-256 values:

| Artifact | SHA-256 |
|---|---|
| Thesis PDF | `b9f7abc184c778736eb73f39ead6de941adbbff5f062c8c4392ab906f8e610cc` |
| GLOBECOM paper PDF | `732cb59741adbb923c9ae0258ae1b473e3aef795ba134d6dce4055253a47eeaa` |
| Released `offline_rl_analysis.py` | `5ab2b701271d9e8cfb059f9f464af6853c4e9204cee6d07dbbeae070e3ba95e8` |
| Released `uav-optimisation-xapp.cc` | `94f28a4f137971ada62eb5328d3ab2997b640dc6da3f7e907d0f9673be8eac9e` |

## Finding

The exact datasets and exact commands behind the reported 300+ runs cannot be
recovered from the supplied artifacts. The README calls the Random Forest
procedure "offline deep RL," but its code performs supervised regression of
`Jfinal` on independently sampled configurations. There are no sequential
transitions, actions chosen during an episode, Bellman targets, replay memory,
or target network. It is preserved here as `offline_rf_benchmark.py` and must
be reported as an RF surrogate/random-search benchmark.

The released RF code uses 28 configuration features. The thesis describes 30
variables because it additionally counts UE and gNB transmit power; those two
variables are present in the simulator output but absent from the released RF
feature list. Results must therefore state which 28-variable released method
was reproduced and must not silently claim recovery of the thesis dataset.

## Newly constructed, not recovered

`generate_rf_dataset.py` creates `configurations.csv` and `commands.json`
before launching any run. These are new deterministic commands based on the
released bounds, not the original thesis commands. The sampling seed, ns-3 RNG
run, configuration, simulator duration, output path, and full argv are thereby
preserved for every new experiment.

Example dry-run command:

```bash
.venv/bin/python scratch/uav-isac-drl/generate_rf_dataset.py \
  --runs 40 --sampling-seed 20260818 --simulation-time 300 --dry-run
```

Remove `--dry-run` only after building `uav-isac-drl`.
