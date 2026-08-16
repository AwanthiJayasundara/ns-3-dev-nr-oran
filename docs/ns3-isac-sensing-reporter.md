# ns-3 ISAC sensing reporter

The UAV autonomy scenario now supports an imperfect sensing path for locating
UEs classified as underserved by the communication measurements. Enable it
with:

```bash
--enable-isac-sensing=1
```

When disabled, the controller retains the former oracle-position baseline.
When enabled, each UAV generates a monostatic radar observation using the
radar range equation, samples a detection from an SNR-dependent logistic
probability, adds SNR-dependent Gaussian localisation error, and contributes
to an inverse-variance fused UE estimate. Only that fused estimate is supplied
to K-means. A UE with no successful detections is omitted from that control
update.

The clustered-coverage runner enables sensing by default. Use
`ENABLE_ISAC_SENSING=0` to run the oracle baseline:

```bash
SEEDS="1 2 3" SIM_TIME=200 ENABLE_ISAC_SENSING=1 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

## Outputs

Sensed runs add two files to the normal result directory:

- `isac-sensing-config.csv`: the complete sensing parameter set.
- `isac-sensing-trace.csv`: per-UAV `OBSERVATION` records and one `FUSED`
  record per underserved UE and control epoch.

The trace includes true positions only for evaluation. Controller code cannot
read those columns; it receives `EstimatedEastM` and `EstimatedNorthM` from
the fused record.

## Main command-line parameters

- `--isac-sensing-tx-power-dbm` (default `30`)
- `--isac-sensing-antenna-gain-dbi` (default `30`, applied on transmit and receive)
- `--isac-sensing-frequency-hz` (default `3.5e9`)
- `--isac-sensing-bandwidth-hz` (default `20e6`)
- `--isac-target-rcs-m2` (default `1`)
- `--isac-sensing-system-loss-linear` (default `1`)
- `--isac-noise-figure-db` (default `7`)
- `--isac-detection-slope` (default `0.5` per dB)
- `--isac-detection-midpoint-db` (default `-15` dB)
- `--isac-position-sigma-ref-m` (default `5` m)
- `--isac-position-snr-ref-db` (default `-10` dB)
- `--isac-position-sigma-min-m` and `--isac-position-sigma-max-m`
  (defaults `1` m and `50` m)

For paper results, run paired oracle and sensed experiments with identical
`RngRun` values. Sweep the RCS, detection midpoint, and reference localisation
error rather than reporting only the defaults.
