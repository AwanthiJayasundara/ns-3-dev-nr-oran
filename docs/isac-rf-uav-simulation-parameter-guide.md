# ISAC–O-RAN UAV Simulation Parameter and Experiment Guide

## 1. Purpose and status

This document is the configuration reference for the UAV-assisted ISAC experiment in
`oran-nr-uav-xhaul-autonomy-example.cc` and
`run-uav-clustered-coverage-rule-ai.sh`. It separates:

- **implemented** behaviour in the current ns-3 program;
- **configured** values supplied by the current runner;
- **recommended** values for a compact 90 s development experiment; and
- **planned** elements appearing in the mathematical model but not yet implemented.

The distinction is essential: a parameter in the paper must not be described as simulated
until the corresponding code path and trace have been verified.

**Revision status (17 August 2026):** the sense-first reactive ISAC controller, all-UE
discovery, best-feasible-cell test, UNKNOWN handling, persistence timer, and their traces are
implemented and have passed small runtime smoke tests. Centroid assignments and arrival
state are also traced. Grid construction, hotspot-RF
training/inference, and probability-weighted predictive K-means remain pending.

## 2. Main decision before running final results

Keep the UAV speed at **15 m/s**. The main problem is not the speed; it is the relationship
between area size, target distance, and simulation duration.

The selectable legacy large-area profile uses a 10 km by 10 km UE/UAV mission area and places the
three underserved clusters approximately 4.95 km radially from the origin. A UAV starting
near the origin needs approximately

\[
t_{\mathrm{travel}} = \frac{\sqrt{3500^2+3500^2}}{15}
                    \simeq 330\ \mathrm{s}
\]

to reach such a cluster. Therefore, a 60 s run of that geometry is useful only as a smoke
test. It cannot provide a fair before/after UAV-repositioning result.

Use the following staged approach:

1. **Implementation smoke test:** one short run, one seed, and one offered load. Its purpose is to
   validate traces and controller logic, not to support final performance claims.
2. **Geometry and link-budget calibration:** temporarily disable UAV movement and verify
   that central UEs are served while the intended outer clusters are genuinely underserved.
3. **Compact 90 s pilot:** use the candidate compact profile in Section 4 only after the
   calibration criteria in Section 12 are satisfied.
4. **Final experiment:** run all baselines with several independent seeds. If the original
   10 km by 10 km geometry is retained, use approximately 390 s (including a post-arrival
   observation window), or initialize UAVs much closer to their operational region.

## 3. Current code capability versus mathematical model

| Component | Current implementation | Status for paper |
|---|---|---|
| Monostatic sensing SNR | Radar-range equation with an \(r^{-4}\) echo-power term | Implemented |
| Detection | Bernoulli sample with logistic probability versus sensing SNR | Implemented |
| Position error | Zero-mean Gaussian error with SNR-dependent standard deviation | Implemented |
| Multi-UAV fusion | Inverse-variance weighted position fusion | Implemented |
| Oracle comparison | True UE positions can be passed directly to K-means when sensing is disabled | Implemented |
| Processing order | All relevant UEs are sensed first; communication service is assessed afterward | Implemented |
| Underserved decision | Serving RSRP plus best feasible TN and existing-UAV alternatives | Implemented |
| Sensing population | All UEs by default; UES2-only remains only as a legacy ablation | Implemented |
| Best feasible TN/UAV test | Fresh candidate RSRPs and per-cell spare capacity used for movement trigger | Implemented |
| TN-first policy | Runtime handover logic searches TN candidates before UAV fallback | Implemented for handover; initial attachment is maximum-RSRP rather than explicitly TN-first |
| Persistence timer \(T_{\mathrm{US}}\) | Weak/no-alternative state must persist; missing state is UNKNOWN first | Implemented |
| Static-UAV switch | `--enable-uav-repositioning=0` prevents target updates | Implemented |
| Grid heatmap \(\rho_g,H_g,Z_g\) | Grid-level feature reporter | Planned |
| Future repositioning-hotspot Random Forest | Predicts future spatial demand for UAV repositioning through \(q_g(t)\) | Planned |
| Probability-weighted K-means | Uses \(q_g\) alone as weights | Planned; current reactive K-means is unweighted |
| `RUN_HOTSPOT_RF=1` / `--enable-hotspot-rf=1` | Dedicated future-hotspot RF interface | Named and separated; inference intentionally rejected until implemented |

The RSRP measurement callbacks now populate the controller cache independently of file
output. Keep the RSRP trace enabled for final reproducibility, but disabling its file no longer
breaks the controller.

### Implemented reactive-controller order

At every control update, the current code performs the following sequence for each included
UE:

1. Every UAV sensing node samples detection and localisation.
2. Successful observations are fused; no fused position is produced after zero detections.
3. Only then does the controller inspect the UE's communication state.
4. Missing attachment or a missing fresh serving measurement produces `UNKNOWN`.
5. A serving RSRP at or above \(R_{\min}\) produces `SERVED`.
6. For weak service, the controller tests a fresh, capacity-feasible TN candidate first and
   an existing capacity-feasible UAV candidate second.
7. A feasible alternative produces `ALTERNATIVE_AVAILABLE`; it does not move a UAV.
8. No feasible alternative starts `PERSISTING`. Only after \(T_{\mathrm{US}}\) does the state
   become `MOVE_REQUIRED`.
9. A UE enters reactive K-means only when its state is `MOVE_REQUIRED` and a sensed fused
   position is available. The oracle baseline substitutes the true position at this final step.

The online controller receives UE group names only for trace labelling. With
`--uav-target-ues2-only=0`, group membership does not filter the decision set.

## 4. Scenario profiles

### 4.1 Profile A — selectable legacy large-area profile

Select this configuration with `SCENARIO_PROFILE=large`. It is no longer the runner default.

| Parameter | Current value |
|---|---:|
| Full UE mission area | \([-5000,5000]\times[-5000,5000]\) m = 100 km² |
| TN placement area | \([-2500,2500]\times[-1200,1200]\) m |
| UAV mission boundary | 10 km by 10 km |
| UAV initial placement | \([-3000,3000]\times[-1500,1500]\) m |
| Outer-cluster offset | 3500 m in each active coordinate |
| Outer-cluster radius | 500 m |
| Approximate radial target distance | 4949.7 m from the origin |
| UAV speed | 15 m/s |
| TN gNBs / gNB-mounted UAVs | 4 / 3 |
| Central UEs | 60 |
| Outer UEs | 20, 30, or 40 |
| Total UEs | 80, 90, or 100 |
| Suitable final duration | Approximately 390 s |

At 15 m/s, a hypothetical 200 s large-profile run permits at most 2925 m of movement after
a 5 s controller start. It is still insufficient for a typical 4.95 km journey. The runner
now defaults to the compact 90 s profile instead.

### 4.2 Profile B — default compact 90 s development profile

Select this profile with `SCENARIO_PROFILE=compact` (the default). It is intended to provide
enough time for controller warm-up, movement, and post-arrival observation. It remains a
**candidate engineering profile**
until the calibration gates pass.

| Parameter | Candidate value | Reason |
|---|---:|---|
| Simulation time | 90 s | Provisional compact development duration |
| Full UE/UAV mission area | 2 km by 2 km (`half-w=1000`, `half-h=1000`) | Bounds the experiment without making the target unreachable |
| TN placement half-width / half-height | 100 m / 100 m | Concentrates terrestrial infrastructure centrally |
| UAV initial half-width / half-height | 100 m / 100 m | Produces a measurable but reachable flight |
| Outer-cluster offset | 350 m | Radial centroid distance about 495 m |
| Outer-cluster radius | 100 m | Keeps three spatially distinct demand regions |
| UAV speed | 15 m/s | Plausible study value and already configured |
| Controller start | 12 s | Allows attachment, measurement reporting, TN-first handover, and persistence |
| Controller period | 1 s | Existing control resolution |
| Persistence \(T_{\mathrm{US}}\) | 6 s | Must exceed handover TTT plus effective RIC reaction time |
| TN gNBs / UAVs | 4 / 3 | Keeps the current infrastructure comparison |
| Central UEs | 60 | Current monitored set |
| Outer UEs | 20, 30, 40 | Low, middle, and high load |

For a centroid 495 m from the origin, nominal travel is about 33 s. Starting control at
12 s gives nominal arrival at about 45 s and leaves about 45 s for post-repositioning observation.
The worst starting corner in the proposed \(\pm100\) m initial box may require roughly
42 s of travel, giving a nominal worst-case arrival near 54 s. These geometric estimates do
not include changing centroids, persistence delays, or assignment changes. Therefore, 90 s
is a safe development choice, not yet the evidence-based final duration.

Determine the final duration from the centroid-assignment trace across calibration seeds.
Define UAV arrival as the first time its distance to the currently assigned centroid is below
the controller reach threshold and remains below it for a short dwell interval. Set the final
simulation duration to at least the 95th-percentile arrival time plus a predeclared 20–30 s
post-arrival observation window. Do not shorten the experiment to 60 s merely because the
nominal trajectory fits.

The compact geometry must be calibrated because the existing 46 dBm TN power may cover
the complete area. Suggested calibration candidates are TN powers of **34, 37, and 40 dBm**
and underserved thresholds of **-105, -100, and -95 dBm**. Select one combination using
the predeclared criteria in Section 12; do not choose a value solely because it produces the
largest gain.

### 4.3 Profile C — original large-area final experiment

If the 10 km by 10 km scenario is scientifically important, retain its current geometry and
use either:

- a duration of at least about **390 s** (approximately 330 s travel, startup/persistence,
  and at least 45 s of post-arrival observation); or
- operationally realistic forward deployment of UAVs closer to the outer region, with the
  initial placement declared clearly.

Do not increase UAV speed merely to force the UAV across several kilometres within 60 s.

## 5. Population, placement, and capacity parameters

| CLI parameter | Current runner | Recommended study value | Notes |
|---|---:|---:|---|
| `--num-uess1` | 60 | 60 | Central/monitored UEs |
| `--num-ground-ues` | 20, 30, 40 | 20, 30, 40 | Outer clustered UEs; total is 80, 90, 100 |
| `--num-tn-gnbs` | 4 | 4 | Terrestrial infrastructure |
| `--num-ntn-gnbs` | 3 | 3 | These gNB-mounted UAVs also act as ISAC sensing nodes |
| `--max-ues-tn` | 20 per TN | 20 | Aggregate nominal TN capacity = 80 UEs |
| `--max-ues-ntn` | 15 per UAV | 15 | Aggregate nominal UAV capacity = 45 UEs |
| `--split-ue-placement` | 1 | 1 | Separates central and outer demand |
| `--clustered-ues2-placement` | 1 | 1 | Creates three outer clusters |
| `--uav-target-ues2-only` | 0 | 0 | All UEs are discovered; central UEs disappear naturally when adequately served |
| `--ues2-cluster-radius-m` | 100 compact / 500 large | Same | Cluster spread |
| `--ues2-cluster-offset-m` | 350 compact / 3500 large | Same | Controls target distance |

Three UAVs and three clusters make the main case easy to interpret, but this can favour a
one-UAV-per-cluster solution. For the middle load of 90 UEs, add a sensitivity experiment
with 1, 2, and 3 UAVs.

## 6. Mobility and control timing

| CLI parameter | Current runner | Recommended compact pilot | Status |
|---|---:|---:|---|
| `--sim-time` | Environment variable; default 90 s | 90 s provisionally | Implemented |
| `--ground-attach-delay` | 5 s | 5 s | Implemented |
| `--uav-control-start` | 12 s compact / 5 s large | 12 s | Implemented |
| `--uav-control-period` | 1 s | 1 s | Implemented |
| `--enable-uav-repositioning` | 1 | Use 0 only for static baseline | Implemented |
| `--uav-speed-mps` | 15 m/s | 15 m/s | Implemented |
| `--mobility-update-ms` | Program default 200 ms | 200 ms | Implemented |
| `--lm-query-interval` | Program default 2 s | 2 s | Implemented |
| `--e2-send-interval` | Program default 2 s | 2 s | Implemented |
| `--tx-delay` | Program default 0.1 s | 0.1 s | Implemented |
| \(T_{\mathrm{US}}\) | 6 s | 6 s | Implemented |
| Unknown/unattached timeout | 6 s | 6 s | Implemented |
| Maximum RSRP sample age | 5 s | 5 s | Implemented |
| Handover TTT | 2 s | 2 s | Implemented and explicitly configured |

With the compact runner's 12 s controller start, the 90 s kinematic upper bound is
\(15(90-12)=1170\) m. The 6 s persistence timer starts only when a UE has a confirmed weak,
no-alternative state (or has passed the UNKNOWN timeout), so actual movement can begin later.
Use the assignment and trajectory traces to measure the realised travel window rather than
assuming 1170 m.

## 7. Access-radio and traffic parameters

| Parameter | Current runner/code value | Guidance |
|---|---:|---|
| Shared TN/UAV service carrier | 4.0 GHz | All monitored service bearers use BWP0; keep fixed across methods |
| Shared service bandwidth | 20 MHz | Main QoS comparison |
| Shared service channel | 3GPP UMa, shadowing disabled | Applies to both TN and low-altitude gNB-mounted UAV service on BWP0 |
| Auxiliary UAV/NTN band | 4.2 GHz, two provisioned BWPs | Not used by the monitored service bearers in the main comparison |
| Auxiliary-band scenario | `UMa` in the revised runner | `NTN-Urban` is available only for a separately justified sensitivity |
| Fast fading | Enabled | Keep for final results; disabling is allowed only for debugging |
| Channel matrix update | 100 ms | Current runner |
| LOS/NLOS condition update | 200 ms | Current runner |
| TN Tx power | 46 dBm | Calibrate for compact geometry; test 34/37/40 dBm |
| UAV Tx power | 37 dBm | Keep initially |
| UE Tx power | 23 dBm | Keep |
| gNB / UE noise figure | 7 / 13 dB | Current code |
| Scheduler | OFDMA proportional fair | Current code and runner |
| Error model | `NrEesmIrT2` | Current code |
| HARQ retransmissions | Disabled | State explicitly; consider one sensitivity run with HARQ enabled |
| RLC mode | AM | Current runner |
| PDCP discard timer | 1000 ms | Current runner |
| Monitored traffic | Bidirectional UDP | Current runner |
| DL / UL offered rate | 0.2 / 0.05 Mbit/s per monitored UE | Light-load service test; add a higher-load sensitivity if throughput claims are central |
| Packet size | 1000 bytes | Current runner |

The current traffic demand is deliberately light. It is suitable for coverage, reliability,
and handover comparisons, but it will not strongly stress scheduler capacity. Do not claim
high-load capacity improvement using only this offered load.

## 8. Handover and underserved-region parameters

| Parameter | Current value | Target model |
|---|---:|---|
| Initial-attach minimum RSRP | -110 dBm | Keep fixed after calibration |
| Handover target minimum RSRP | -110 dBm | Keep fixed after calibration |
| Handover hysteresis | 2 dB | Keep |
| Underserved threshold \(R_{\min}\) | -100 dBm | Calibrate jointly with TN power |
| Runtime preference | TN candidate first, UAV fallback | Keep |
| Initial-attachment preference | Maximum RSRP across feasible cells | Document or revise if strict TN-first attachment is claimed |
| Movement trigger | Persistent serving weakness with no feasible TN or existing UAV alternative | Implemented |
| Controller period \(T_{\mathrm{ctrl}}\) | 1 s | Target recomputation and mathematical control interval \(\Delta t\) |
| Internal mobility update | 200 ms | Numerical waypoint-motion integration; not \(T_{\mathrm{ctrl}}\) |
| E2 / LM intervals | 2 s / 2 s | Reporting and logic-module query periods; distinct from \(T_{\mathrm{ctrl}}\) |
| Persistence | 6 s | Verify \(T_{\mathrm{US}}>T_{\mathrm{TTT}}+T_{\mathrm{ctrl}}\) |
| UNKNOWN timeout | 6 s | Prevents attachment and missing-measurement transients from creating hotspots |
| RSRP maximum age | 5 s | Excludes stale serving/candidate measurements |

The target movement decision should distinguish experienced service from the need to move a
UAV:

- serving-cell RSRP records what the UE currently experiences;
- best feasible TN/UAV RSRPs determine whether an alternative cell can already solve the
  problem; and
- the persistence timer prevents movement during attachment or an ongoing handover.

The nominal service threshold \(R_{\min}=-100\) dBm is intentionally stricter than the current
-110 dBm minimum used for attachment and handover eligibility. Consequently, a UE may remain
legally connected between -110 and -100 dBm while being counted as weak-service demand. This
does not by itself trigger movement: UNKNOWN handling, feasible alternatives, and persistence
are still applied.

Missing RRC state or a missing/stale serving RSRP sample is now labelled **UNKNOWN** for the
configured timeout. It cannot immediately become a movement target. After the timeout, the
normal no-alternative persistence timer still has to expire.

The feasible-cell test currently includes fresh RSRP and spare per-cell capacity. This is
sufficient for the main TN+UAV profile, where the separate xHaul controller is disabled. Do
not claim backhaul-aware feasibility in the main experiment unless that check is explicitly
added to this movement controller.

## 9. ISAC sensing parameters

All values in this section are system-level assumptions. They have not been derived from a
specific radar front-end or measurement campaign and must be presented as simulation
parameters, then tested for sensitivity.

The prototype models ISAC at the system/control level through co-located sensing and
communication functions. It does not implement joint waveform design or explicit
sensing/communication resource sharing, and the paper must not claim those capabilities.

| CLI parameter | Current value | Meaning |
|---|---:|---|
| `--enable-isac-sensing` | 1 in runner | Uses sampled/fused positions instead of oracle coordinates |
| `--isac-sensing-tx-power-dbm` | 30 dBm | Monostatic sensing transmit power |
| `--isac-sensing-antenna-gain-dbi` | 30 dBi | Gain used on transmit and receive in the radar equation |
| `--isac-sensing-frequency-hz` | 4.0 GHz in the revised runner | Aligned with the shared TN/UAV service carrier at the system level |
| `--isac-sensing-bandwidth-hz` | 20 MHz | Noise bandwidth |
| `--isac-target-rcs-m2` | 1 m² | Assumed UE radar cross section |
| `--isac-sensing-system-loss-linear` | 1 | Aggregate linear loss; optimistic and should be calibrated |
| `--isac-noise-figure-db` | 7 dB | Sensing receiver NF |
| `--isac-detection-slope` | 0.5 | Logistic detection-curve slope |
| `--isac-detection-midpoint-db` | -15 dB | SNR giving 0.5 detection probability |
| `--isac-position-sigma-ref-m` | 5 m | Reference position standard deviation |
| `--isac-position-snr-ref-db` | -10 dB | SNR associated with reference deviation |
| `--isac-position-sigma-min-m` | 1 m | Lower error bound |
| `--isac-position-sigma-max-m` | 50 m | Upper error bound |

At minimum, sweep the target RCS, system loss or effective sensing power, and detection
midpoint. Report detection probability versus range, localisation RMSE versus range, fused
observation count, and no-detection rate. The fixed random streams `91001` and `91002` make
sensing repeatable for a given experiment; record this fact in reproducibility notes.

## 10. RF future repositioning-demand model parameters

These parameters are required by the mathematical model but are not yet implemented as a
working grid/RF pipeline. Only the dedicated `--enable-hotspot-rf` and
`--hotspot-rf-model` names currently exist; enabling RF deliberately stops with an explanatory
error. Implement and freeze the parameters below before the RF experiment.

| Proposed parameter | Initial pilot value | Required sensitivity or rule |
|---|---:|---|
| Grid-cell width | 100 m for compact profile | Test 50, 100, 200 m |
| Prediction horizon \(\Delta_p\) | 5 s | Also test 2 and 10 s |
| Minimum UEs \(N_{\min}\) | 3 | Choose before testing |
| RF probability threshold \(\tau\) | Selected on validation seeds | Do not tune on test seeds |
| RF tree count | 200 | Record maximum depth, feature sampling, and class weighting |
| K-means restarts | At least 10 | Fix RNG seed and retain lowest objective |
| K-means weight | \(q_g\) | Must match the final paper equation; do not multiply by current \(H_g\) |

Freeze the RF input exactly as

\[
\mathbf{x}_g(t)=[\rho_g,H_g,Z_g,\overline{R}_g,
\overline{\sigma}_g,\Delta H_g,d_g^{Q}]^{\mathsf T},
\]

with the hard grid assignment in the mathematical model.
Here, \(H_g\) is persistent UAV-repositioning demand, not a count of every weak UE.
Compute \(\overline{R}_g\) only over detected UEs assigned to \(g\) that have a fresh serving
measurement, and compute \(\overline{\sigma}_g\) over all detected UEs assigned to \(g\).
Use \(R_{\mathrm{floor}}\) when the RSRP averaging set is empty and
\(\sigma_{\max}\) when the detected-UE set is empty.

### Ground-truth label and reference policy

Freeze one RF-disabled reference policy \(\pi_{\mathrm{ref}}\) before dataset generation. Use
the **ISAC-sensed reactive controller** as the main reference policy, with geometry,
handover, persistence, radio, and sensing parameters fixed. For each future grid cell, compute
offline

\[
H_{g,\pi_{\mathrm{ref}}}^{\mathrm{true}}(t+\Delta_p)=
\sum_{u\in\mathcal U}
I_{u,\pi_{\mathrm{ref}}}^{\mathrm{move}}(t+\Delta_p)
\mathbf 1\{\mathbf p_u^{xy}(t+\Delta_p)\in g\}.
\]

Here, “true” means that the calculation uses the true future UE positions and the same
persistent no-feasible-alternative criterion used by the reactive controller. It does not
mean that oracle coordinates are supplied to online control. The training label is

\[
y_g(t)=\mathbf{1}\{
H_{g,\pi_{\mathrm{ref}}}^{\mathrm{true}}(t+\Delta_p)
\geq N_{\min}\}.
\]

Generate features and labels only from runs in which hotspot RF is disabled. Never generate
labels while the proactive RF controller is moving UAVs: its actions change future RSRP,
feasible-cell states, and therefore the prediction target. Store
`reference_policy=isac-reactive` in the dataset manifest. A static-policy label set may be
used as a separate sensitivity study, but it must not be mixed with reactive-policy samples.
These labels are explicitly conditional on \(\pi_{\mathrm{ref}}\); they are not claimed to be
policy-independent demand labels. Closed-loop proactive evaluation must therefore be reported
separately from offline RF test metrics.

There is no kernel bandwidth or underserved-share threshold in this final formulation.
Report RF precision, recall, and F1—not accuracy alone—because hotspot cells are likely the
minority class.

### Required temporal behaviour

The existing simulation uses random-waypoint motion for both UE groups. Outer clustered UEs
move at 4 m/s within their cluster region and central UEs move at 5 m/s; these speeds are now
exposed as `--ues2-speed-mps` and `--ues1-speed-mps`. Before training, verify from grid traces
that \(H_g(t)\) actually changes across cells over horizons of 2, 5, and 10 s. If hotspot-cell
labels remain nearly constant, enlarge the cluster mobility region, introduce controlled
cluster-centre drift, or vary service conditions. A classifier that merely memorises fixed
cluster centres is not an acceptable predictive experiment.

For each horizon, report at least:

- positive-cell prevalence;
- the fraction of cell labels that change over the horizon;
- hotspot lifetime and transition counts; and
- F1 or precision–recall performance for a persistence baseline that predicts the current
  label as the future label.

Proceed with RF only if every dataset split contains enough positive cells and transitions
to support meaningful precision, recall, and F1 estimates. If labels barely change, adjust
mobility, cluster spread/drift, grid size, or service dynamics before training; do not add
future information to the feature vector.

Training, validation, and test samples must use disjoint simulation seeds. Splitting adjacent
time samples from the same run across these sets would cause temporal leakage. Fit feature
normalisation or imputation using training data only, even though an RF does not require
standard scaling.

## 11. Optional satellite backhaul and xHaul parameters

| Parameter | Optional code default/reference value | Main-run status |
|---|---:|---|
| Deployment | TN + UAV | Enabled |
| Satellite monitor | Available in program | Disabled |
| UAV xHaul switching xApp | Available in program | Disabled |
| Satellite scenario | `NTN-Suburban` | Not used |
| Satellite carrier / bandwidth | 20 GHz / 400 MHz | Not used |
| xHaul donor Tx power | 46 dBm | Not used |
| xHaul path-loss exponent | Program default 2.0 | Not used |
| xHaul reference distance | Program default 1 m | Not used |
| Healthy xHaul threshold | -100 dBm | Not used |
| TN/satellite switching TTTs | 5 s / 5 s | Not used |

Satellite and xHaul switching are outside the main ISAC–RF repositioning comparison and are
disabled by the revised runner. If studied separately, the xHaul monitor is a geometry-based
policy proxy, not a physical NR backhaul UE mounted on the UAV; state this limitation.

## 12. Calibration gates before the experiment matrix

Run a no-repositioning calibration using the intended geometry. Select radio settings using
criteria declared in advance:

1. At least 90% of central UEs should be attached and above the service threshold after the
   warm-up period.
2. A clear majority of outer-cluster UEs should remain below the best-feasible TN threshold
   for the persistence interval; a useful target is at least 70%, without requiring 100%.
3. Outer UEs should become feasible UAV users once a UAV reaches a cluster. Otherwise the
   experiment tests movement but not service restoration.
4. No single cell should be the only feasible cell for every UE; otherwise handover and
   spatial control comparisons become trivial.
5. The sensing model must yield a mixture of detections and misses over the operational
   range. A permanent 0% or 100% detection rate cannot demonstrate sensing quality.
6. Inspect RSRP distributions rather than using only the mean. Save median, 5th percentile,
   and threshold-crossing fraction for central and outer groups separately.

Freeze the selected geometry, TN power, service threshold, and sensing parameters before
running the final controller comparison.

## 13. Required experiment matrix

The minimum method comparison is:

| Method | Position source | Target policy | Purpose |
|---|---|---|---|
| Static UAV baseline | None | `--enable-uav-repositioning=0` | Measures benefit of movement; supported |
| Oracle reactive K-means | True UE coordinates | Persistent UAV-repositioning demand with no feasible alternative | Upper reference for localisation |
| ISAC reactive K-means | Fused sensed coordinates | Detected persistent UAV-repositioning demand | Measures sensing penalty |
| ISAC + RF proactive K-means | Fused sensed coordinates | Predicted future repositioning-demand cells weighted by \(q_g\) | Proposed method; pending implementation |

### What is trained and what runs online

K-means is **not trained** and does not produce a reusable model file. In each reactive
controller update, the simulator first identifies UEs in `MOVE_REQUIRED`, constructs a new
set of target points, and runs K-means again. It chooses

\[
K=\min\{\text{available UAVs},\text{target points}\},
\]

uses 10 Lloyd iterations in the current implementation, and greedily assigns available UAVs
to the nearest unassigned centroids. Thus, K-means is an online target-calculation step.

The four methods differ as follows:

1. **Static UAV:** the target-update controller is not scheduled. The UAVs remain at their
   initial locations, so there is no target-point set and no K-means execution. This baseline
   measures performance without repositioning.
2. **Oracle reactive:** the normal communication logic identifies persistent
   `MOVE_REQUIRED` UEs, and their true simulator coordinates are passed to unweighted
   K-means. Ground truth replaces only the position input; it does not replace the RSRP,
   capacity, UNKNOWN, or persistence decisions. This is the localisation upper reference.
3. **ISAC reactive:** every relevant UE is sensed before service classification. A
   `MOVE_REQUIRED` UE enters unweighted K-means only when at least one sensing node detects
   it and a fused position is available. This comparison measures losses caused by missed
   detections and localisation error.
4. **RF predictive:** the Random Forest is the only learned component. It must first be
   trained offline from grid features \(\mathbf{x}_g(t)\) and future labels generated under
   the frozen RF-disabled reference policy \(\pi_{\mathrm{ref}}\). Online, it predicts
   \(q_g(t)\), selects future repositioning-demand cells, and uses \(q_g(t)\) as the K-means
   weight. The grid reporter, RF training/inference path, and weighted K-means are not yet
   implemented and must not be included in current result claims.

The current runnable comparison therefore contains the first three methods. Running
`METHODS="static oracle-reactive isac-reactive"` does not train any model; it executes the
static baseline and two online reactive-clustering baselines.

For final statistics, use at least 10 independent seeds if server time permits. Use 3 seeds
only for debugging and preliminary plots. Report mean with a confidence interval or show the
per-seed distribution. The three load points 80, 90, and 100 UEs should use identical method
settings.

### Priority paper results

Limit the main presentation to results that directly test the contribution:

1. ISAC detection probability and localisation RMSE versus sensing range.
2. A spatial sequence showing weak UEs, persistent repositioning targets, RF-predicted
   repositioning-hotspot cells/centroids, and UAV trajectories.
3. RF precision, recall, and F1 for \(\Delta_p\in\{2,5,10\}\) s.
4. Static, oracle-reactive, ISAC-reactive, and ISAC+RF results for persistent underserved-UE
   fraction, fifth-percentile throughput, and outage time.

For the 90-UE load, additionally compare 1, 2, and 3 UAVs so that the result does not depend
only on supplying exactly one UAV for each of three constructed clusters.

## 14. Results and trace policy

### Keep enabled

| Output | Current switch/file | Why needed |
|---|---|---|
| QoS and flow results | `--enable-flow-monitor=1`, `qos-vs-time.txt`, `flow-stats.log`, FlowMonitor XML when written | Throughput, PDR, delay, loss, 5th percentile |
| RSRP | `--enable-rsrp-trace=1`, `rsrp-trace.tr` | Service classification and radio validation; cache collection remains active even if file output is disabled |
| UE positions | `ues1-position-trace.tr`, `ues2-position-trace.tr` | Oracle error and spatial evaluation |
| UAV positions | `uav-position-trace.tr` | Movement, arrival, path length |
| Handover | `handover-trace.tr` | Successful cell changes |
| Handover failures | `handover-failure-trace.tr` | Reliability and false reaction analysis |
| ISAC observations | `isac-sensing-trace.csv` | SNR, detection, errors, fusion |
| ISAC configuration | `isac-sensing-config.csv` | Reproducibility |
| Service/movement decision | `uav-service-decision-trace.csv` | Serving/best-TN/best-UAV RSRP, UNKNOWN/persistence state, and target inclusion |
| UAV target assignment | `uav-target-assignment-trace.csv` | Assigned centroids, distance-to-target, reach threshold, and arrival state |
| xHaul autonomy | `xhaul-autonomy-trace.csv` | Optional; not required for the main TN+UAV experiment |
| Satellite backhaul | `sat-backhaul-trace.txt` | Optional; not required for the main TN+UAV experiment |

### Keep disabled for routine final runs

- verbose O-RAN INFO logging;
- NR helper INFO logging;
- setup and progress prints;
- per-candidate handover decision CSV, unless debugging a handover anomaly;
- O-RAN app-loss and cell-load reports unless used as RF features or paper results;
- fronthaul-control calculations and REM generation.

The current runner already follows this lightweight trace policy. A 2 s position interval is
reasonable for overall plots, but use 1 s if centroid-arrival error or trajectory timing is a
primary result.

### Already implemented controller trace

`uav-service-decision-trace.csv` records the serving RSRP, best feasible TN RSRP, best
feasible existing-UAV RSRP, service state, state age, sensed/oracle-position availability,
controller coordinates, and whether the UE entered the movement target set.
In `isac-sensing-trace.csv`, `PreClassificationUnderserved=-1` deliberately means “not yet
classified”; the communication result must be obtained by joining the later service-decision
row, preserving the sense-before-classify order.
`uav-target-assignment-trace.csv` records both new `ASSIGN` decisions and `TRACK` rows when
no new target is generated. Its `Arrived` field is computed from `DistanceToTargetM` and the
actual controller `ReachThresholdM`, and is valid only after `HasMovementAssignment=1`.
This prevents the initial hover target from being counted as an arrival and enables seed-wise
arrival-time measurement.

### Additional traces required by the RF target model

Before claiming the complete mathematical model was simulated, add:

- grid features \(\rho_g,H_g,Z_g\) and the KPI aggregates;
- RF probability \(q_g\), predicted label, oracle future label, and dataset split;
- RF-selected predictive centroids and their probabilities. Reactive centroid assignments
  and arrival state are already available in `uav-target-assignment-trace.csv`.

## 15. Build and run commands on the server

Build an optimized executable once:

```bash
./ns3 configure -d optimized --out=build-optimized --enable-examples --disable-tests --disable-logs
cmake --build build-optimized --target oran-nr-uav-xhaul-autonomy-example -j4
```

The runner refuses to select an executable older than the scenario source. This prevents an
old optimized binary from silently ignoring newly added CLI parameters. Rebuild whenever the
C++ scenario changes.

### 15.1 One middle-load development run

```bash
EXPERIMENT_TAG=dev-isac-v1 SCENARIO_PROFILE=compact SIM_TIME=90 \
  UES2_LOADS="30" SEEDS="1" JOBS=1 \
  ENABLE_UAV_REPOSITIONING=1 ENABLE_ISAC_SENSING=1 RUN_HOTSPOT_RF=0 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

### 15.2 Compact radio calibration sweep

Keep the controller and sensing reporters active but prevent physical movement with
`UAV_SPEED_MPS=0`. This produces service and sensing traces for calibration without changing
the UE radio environment through repositioning.

```bash
for tn_dbm in 34 37 40; do
  for rmin_dbm in -105 -100 -95; do
    EXPERIMENT_TAG="cal-tn${tn_dbm}-rmin${rmin_dbm}" \
      SCENARIO_PROFILE=compact SIM_TIME=90 UES2_LOADS="30" \
      SEEDS="1 2 3" JOBS=3 UAV_SPEED_MPS=0 \
      TN_TX_POWER_DBM="${tn_dbm}" UNDERSERVED_RSRP_DBM="${rmin_dbm}" \
      ENABLE_UAV_REPOSITIONING=1 ENABLE_ISAC_SENSING=1 RUN_HOTSPOT_RF=0 \
      contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
  done
done
```

Choose the TN power and \(R_{\min}\) using the gates in Section 12, then use exactly that pair
for every compared method. Do not select the pair using end-to-end method gain.

### 15.3 Three implemented comparison methods

Set the values chosen during calibration. The values below are examples and must be replaced
if calibration selects a different pair.

Run all three implemented methods with one runner invocation:

```bash
EXPERIMENT_TAG=reactive-comparison-v1 \
  METHODS="static oracle-reactive isac-reactive" \
  SCENARIO_PROFILE=compact SIM_TIME=90 UES2_LOADS="20 30 40" \
  SEEDS="1" JOBS=1 NUM_TN_GNBS=4 NUM_UAVS=3 UAV_SPEED_MPS=15 \
  TN_TX_POWER_DBM=37 UNDERSERVED_RSRP_DBM=-100 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

This is nine simulations: three methods multiplied by three UE-load cases. Methods run
sequentially; `JOBS` controls parallel seed batches within each method. The runner assigns
the required switches automatically: static disables repositioning, oracle reactive uses
true positions, and ISAC reactive uses sampled/fused positions.

The equivalent individual-method commands are retained below for debugging or selective
reruns.

```bash
CAL_TN_DBM=37
CAL_RMIN_DBM=-100
COMMON_ENV=(
  SCENARIO_PROFILE=compact
  SIM_TIME=90
  UES2_LOADS=30
  "SEEDS=1 2 3"
  JOBS=3
  "TN_TX_POWER_DBM=${CAL_TN_DBM}"
  "UNDERSERVED_RSRP_DBM=${CAL_RMIN_DBM}"
  RUN_HOTSPOT_RF=0
)
```

Run the static baseline:

```bash
env "${COMMON_ENV[@]}" EXPERIMENT_TAG=static-v1 \
  ENABLE_UAV_REPOSITIONING=0 ENABLE_ISAC_SENSING=1 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

Run oracle-position reactive K-means:

```bash
env "${COMMON_ENV[@]}" EXPERIMENT_TAG=oracle-reactive-v1 \
  ENABLE_UAV_REPOSITIONING=1 ENABLE_ISAC_SENSING=0 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

Run ISAC-sensed reactive K-means:

```bash
env "${COMMON_ENV[@]}" EXPERIMENT_TAG=isac-reactive-v1 \
  ENABLE_UAV_REPOSITIONING=1 ENABLE_ISAC_SENSING=1 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

The same `SEEDS`, load, geometry, radio parameters, traffic, and timing must be used in all
three commands. `JOBS=3` runs three seed batches concurrently; reduce it if server memory or
CPU contention increases runtime.

### 15.4 Arrival-time check

For each target-assignment CSV, print the first genuine arrival time for every UAV:

```bash
find results/nr/tn-ntn -name uav-target-assignment-trace.csv -print0 |
  while IFS= read -r -d '' file; do
    echo "${file}"
    awk -F, 'NR>1 && $12==1 && $13==1 && !seen[$3]++ {print "UAV=" $3, "arrival_s=" $1}' "${file}"
  done
```

Use the per-seed distribution—not one run—to calculate the 95th-percentile arrival time and
then add the declared 20–30 s post-arrival window.

### 15.5 Full implemented-method matrix after calibration

Replace the example calibrated values and choose fresh experiment tags:

```bash
SEEDS="1 2 3 4 5 6 7 8 9 10"
TN_DBM=37
RMIN_DBM=-100

EXPERIMENT_TAG=isac-reactive-final-v1 SCENARIO_PROFILE=compact SIM_TIME=90 \
  UES2_LOADS="20 30 40" SEEDS="${SEEDS}" JOBS=2 \
  TN_TX_POWER_DBM="${TN_DBM}" UNDERSERVED_RSRP_DBM="${RMIN_DBM}" \
  ENABLE_UAV_REPOSITIONING=1 ENABLE_ISAC_SENSING=1 RUN_HOTSPOT_RF=0 \
  contrib/oran/examples/run-uav-clustered-coverage-rule-ai.sh
```

Repeat this final command for the static and oracle-reactive settings shown above. Do not run
the final matrix until the checklist in Section 16 passes.

### 15.6 Runner behaviour and RF limitation

`UES2_LOADS="30"` runs only the middle-load development case; omit it to run 20, 30, and 40.
Set `METHODS="static oracle-reactive isac-reactive"` to execute all three implemented methods
in one invocation. When `METHODS` is set, it overrides the individual
`ENABLE_UAV_REPOSITIONING`, `ENABLE_ISAC_SENSING`, and `RUN_HOTSPOT_RF` method-selection
switches.
`RUN_HOTSPOT_RF` is deliberately separate from the old xHaul ONNX controller. The executable
currently rejects `RUN_HOTSPOT_RF=1` until grid-feature generation and RF inference are
implemented, preventing an xHaul model from being mislabeled as the hotspot predictor.

After calibration, launch all three loads by using `UES2_LOADS="20 30 40"`. Each seed runs
its load cases sequentially, while up to `JOBS` independent seed batches run concurrently.
For the static baseline, add `ENABLE_UAV_REPOSITIONING=0`; use `1` for both reactive cases.
The runner automatically labels outputs as `static`, `oracle-reactive`, `isac-reactive`, or
`isac-rf-predictive` from these switches, avoiding accidental baseline-name collisions.
Use a new `EXPERIMENT_TAG` for every intentional rerun. By default, the runner refuses to
overwrite an existing SQLite database; `ALLOW_EXISTING_RESULTS=1` should be used only when
that overwrite is deliberate.

Each experiment is stored under `results/nr/tn-ntn/<EXPERIMENT_TAG>/`. Individual ns-3 runs
receive separate subdirectories, SQLite files are placed in `db/`, and captured stdout/stderr
is placed in `runner-logs/`. Set `OUTPUT_ROOT` or `OUTPUT_PARENT_DIR` only when a different
layout is required.

Set `ENABLE_NS3_LOG=1` to create `ns3-oran-lm.log` inside every run directory. The runner
always creates a separate `<run-label>.console.log`, even when ns-3 component logging is
disabled. For full compiled `NS_LOG` output, build with `--enable-logs`; a binary configured
with `--disable-logs` still records explicit scenario trace messages but compiles out normal
component logging. `ENABLE_NR_HELPER_INFO_LOG=1` is available for focused debugging but is
too verbose for routine experiment matrices.

### 15.7 Plotting the method comparison

After the three-method development matrix finishes, generate the available comparison plots:

```bash
python3 contrib/oran/examples/plot-isac-method-comparison.py \
  --tags reactive-comparison-v1 \
  --warmup-s 20 \
  --output-dir results/plots/reactive-comparison-v1
```

The script writes PNG/PDF figures and per-run/aggregate CSV summaries. It uses a fixed
four-method order but never fabricates a missing curve. Before RF is implemented, it plots
static, oracle-reactive, and ISAC-reactive results and reports RF-predictive as missing.
After genuine RF-predictive runs exist under a separate tag, combine both tags:

```bash
python3 contrib/oran/examples/plot-isac-method-comparison.py \
  --tags reactive-comparison-v1 rf-predictive-test-v1 \
  --warmup-s 20 \
  --output-dir results/plots/four-method-comparison-v1
```

The current `qos-vs-time.txt` records the 60 monitored central UEs (UES1), so these figures
are explicitly labelled as central-UE QoS. Use `uav-service-decision-trace.csv` for all-UE
service/repositioning-state analysis. Do not describe the central-UE QoS curves as outer-UE
or all-UE metrics.

The initial nine runs validate the baseline controllers, sensing, traces, and plotting; they
are not yet an RF-training dataset. After implementing the grid reporter, rerun the frozen
ISAC-reactive reference policy over the declared training/validation seeds to export
\(\mathbf{x}_g(t)\) and policy-conditioned future labels. Train the RF offline, freeze its
model and threshold, and then run `isac-rf-predictive` on disjoint test seeds before creating
the four-method figure.

## 16. Go/no-go checklist

Do not start the final multi-seed matrix until every required item is true:

- [ ] Centroid-assignment traces show that the selected duration covers the 95th-percentile
      UAV arrival plus the declared post-arrival observation interval.
- [ ] Central/outer RSRP distributions satisfy the calibration gates.
- [x] RSRP cache collection is independent of optional trace-file output.
- [x] Best-feasible TN/UAV movement trigger is implemented and traced.
- [x] Persistence timer is implemented and traced.
- [x] Missing measurements are distinguished from confirmed underserved service.
- [ ] ISAC detection and localisation performance are non-degenerate and calibrated.
- [ ] Static, oracle-reactive, and ISAC-reactive baselines use identical scenario seeds.
- [ ] Grid reporter, RF predictor, and \(q_g\)-weighted K-means are implemented before claiming the
      complete proposed method.
- [ ] \(H_{g,\pi_{\mathrm{ref}}}^{\mathrm{true}}\) labels are generated under one frozen,
      RF-disabled reference policy and never under proactive RF control.
- [ ] Positive prevalence, label-transition rate, hotspot lifetime, and persistence-baseline
      performance confirm that prediction at 2/5/10 s is meaningful.
- [ ] Train/validation/test seeds are disjoint.
- [ ] All parameter values are exported with each run.

## 17. Immediate recommended configuration

For the next development run—not the final paper run—use one middle-load case (60 central +
30 outer UEs), one seed, 90 s, 15 m/s, FlowMonitor, RSRP, positions, handovers, and ISAC
traces. Use `UES2_LOADS="30"` to select only that load. Verify the new sense-first and
best-feasible-cell persistence states in `uav-service-decision-trace.csv`, then calibrate the
compact geometry and radio link before launching 80/90/100-UE multi-seed runs. Use
`uav-target-assignment-trace.csv` across calibration seeds to confirm or revise the 90 s
duration. Implement the grid/RF pipeline only after this reactive controller is validated;
then generate RF labels in separate RF-disabled reference-policy runs.
