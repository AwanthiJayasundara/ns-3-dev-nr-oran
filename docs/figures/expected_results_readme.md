# Expected Results Figures

These files are **expected/synthetic interpretation figures**, not measured ns-3 results. Use them as the target pattern for the article and replace them with measured plots after all three simulations complete.

## Updated Degradation Model

All three scenarios now include a 15-30 s TN infrastructure degradation window using `--tn-degradation-start=15 --tn-degradation-stop=30 --tn-degradation-penalty-db=15`. The UAV scenarios also include the 15-30 s UAV xHaul degradation window using `--xhaul-degradation-start=15 --xhaul-degradation-stop=30 --xhaul-degradation-penalty-db=35`.

## Files

- `expected_figure1_qos_under_xhaul_degradation.png`: expected throughput and delay behavior under stressed TN.
- `expected_figure2_handover_counts.png`: expected successful/failed handover count pattern.
- `expected_figure3_xhaul_backhaul_mode.png`: expected TN TxPower stress, UAV xHaul RSRP, and backhaul route switching.
- `expected_results_summary.csv`: table explaining the expected comparison.

## Main Expected Pattern

TN-only should degrade during 15-30 s because the terrestrial layer is weakened. TN + UAV should improve access coverage compared with TN-only, but it still suffers when the UAV xHaul is degraded because `BackhaulMode` stays `TN_DIRECT`. TN + UAV + Satellite should switch to `SATELLITE_FALLBACK` during 15-30 s when satellite health is good, so it should show the best service continuity.

## Coverage Figure

- `expected_figure4_service_effective_coverage.png`: recommended single-panel coverage figure for the article.
- `expected_service_effective_coverage_summary.csv`: table explaining the service-effective coverage interpretation.
- `expected_figure4_coverage_comparison.png`: older two-panel version kept for reference.

Use service-effective coverage as the main coverage metric. It means a UE can connect to a cell that can actually carry traffic through a usable backhaul path. During 15-30 s, TN-only should be lowest, TN + UAV should improve but remain limited by degraded UAV xHaul, and TN + UAV + Satellite should be highest because satellite fallback protects service continuity.
