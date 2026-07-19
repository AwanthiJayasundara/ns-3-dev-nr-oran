# Toward xHaul-Aware Mission-Adaptive UAV O-RAN for TN-NTN Service Continuity

## Abstract

The integration of terrestrial networks (TN), unmanned aerial vehicle (UAV) cells, and satellite-assisted non-terrestrial networks (NTN) is a promising direction for extending mobile connectivity in congested, remote, and disaster-affected environments. However, UAV-assisted radio access cannot be evaluated only from the user equipment (UE) fronthaul perspective. A UAV cell may provide strong access-link coverage to UEs while still being unable to deliver reliable service if its wireless xHaul connection to the terrestrial donor, core network, or control infrastructure is degraded. This article proposes an xHaul-aware mission-adaptive UAV O-RAN scenario for studying progressive UAV autonomy under infrastructure degradation. The same UAV platform is evaluated under three deployment modes: UE + TN only, UE + TN + UAV, and UE + TN + UAV + satellite. The UAV-to-ground TN donor link is used as the xHaul health indicator, while satellite backhaul monitoring is added in the third mode to represent service-continuity support when terrestrial infrastructure becomes degraded or unavailable. The proposed ns-3/ns-O-RAN simulation records QoS, handover, xHaul, and autonomy-mode traces, enabling a comparative evaluation of terrestrial-only service, UAV-assisted coverage, and satellite-assisted UAV continuity. The study provides a foundation for future AI-native O-RAN control in which UAV network functions adapt dynamically according to access, xHaul, and control-path conditions.

## 1. Introduction

Future 5G-Advanced and 6G systems are expected to support connectivity across heterogeneous three-dimensional network environments. Terrestrial infrastructure alone is often insufficient in remote regions, temporary crowded events, and disaster scenarios where ground base stations may be overloaded, damaged, or partially disconnected. UAV-mounted cells can provide flexible aerial coverage, while satellite links can provide additional resilience when terrestrial backhaul is degraded.

However, the main challenge is not simply whether a UAV can transmit a strong radio signal to UEs. A UAV cell also requires reliable xHaul connectivity toward the terrestrial network, edge cloud, core network, or O-RAN control plane. If the UAV access link is healthy but the UAV-to-ground donor connection is weak, delayed, or unavailable, the UAV may become a coverage island with limited service continuity. Therefore, UAV-assisted TN-NTN systems should be evaluated using both UE-facing radio metrics and infrastructure-facing xHaul/control metrics.

This article focuses on a mission-adaptive UAV O-RAN concept. The UAV changes its operational role depending on infrastructure health. When terrestrial xHaul and control connectivity are healthy, the UAV behaves mainly as an aerial coverage extension. When xHaul becomes degraded, it can activate additional local functions and operate in a semi-autonomous mode. When terrestrial connectivity is unavailable, the UAV can escalate to an autonomous emergency-network mode, optionally assisted by satellite backhaul.

## 2. Proposed Concept

The proposed concept is based on progressive UAV autonomy. The same UAV platform does not always operate with the same level of onboard network functionality. Instead, its role depends on the health of the surrounding infrastructure.

In normal conditions, the UAV provides radio coverage while relying on terrestrial infrastructure for network processing, O-RAN control, and core-network connectivity. This is suitable for events, temporary hotspots, or coverage extension where ground infrastructure remains healthy.

In degraded conditions, the UAV detects that its xHaul connection to the terrestrial donor is becoming weak. It can then activate more onboard functions, reduce dependency on remote control, and maintain service with a semi-autonomous aerial RAN mode.

In disaster or isolation conditions, the UAV may lose terrestrial xHaul or control reachability. In this case, it can switch to an autonomous emergency-network mode. If satellite connectivity is available, the satellite path can provide additional backhaul support for emergency service continuity.

The key idea is:

```text
Access coverage alone is not enough.
The UAV must also know whether its xHaul/control path is healthy.
```

## 3. Simulation Environment

The scenario is implemented in ns-3 using 5G-LENA NR and ns-O-RAN components. The main simulation file is:

```text
contrib/oran/examples/oran-nr-uav-xhaul-autonomy-example.cc
```

The comparison script is:

```text
contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

The simulation contains:

- terrestrial NR gNBs;
- UAV-mounted NR cell nodes;
- two UE groups;
- O-RAN Near-RT RIC-based handover control;
- UAV-to-ground TN donor xHaul monitoring;
- optional satellite backhaul monitoring;
- QoS, handover, position, decision, and autonomy traces.

The first UE group, UES1, is used for monitored traffic and handover/QoS evaluation. The second UE group, UES2, provides additional background load. In the current TN-only reference setup, 20 UES1 nodes are available from the start, and 50 UES2 background UEs attach at 5 s. The UAV-assisted scenarios can use a larger monitored group, currently 50 UES1 and 50 UES2, when stress-testing UAV and satellite continuity.

## 4. Handover and Control Mechanism

Initial UE attachment is performed using the strongest-RSRP cell selection mechanism:

```text
AttachToMaxRsrpGnb()
```

Runtime handover is controlled through the O-RAN logic module rather than the native NR handover algorithm. The NR helper keeps the native handover algorithm as:

```text
NrNoOpHandoverAlgorithm
```

This ensures that runtime UE handover decisions are issued by the O-RAN control logic. The UE Mobility xApp/logic module is:

```text
OranLmNr2NrRsrpHandoverWithTnNtn
```

This module uses UE-reported RSRP, hysteresis, minimum acceptable RSRP, cell capacity, TN/NTN cell type, and optional backhaul health information. A second RIC logic module, `OranLmUavAutonomyControl`, monitors UAV xHaul and satellite health, selects the active UAV control authority, updates the UAV backhaul/control-path state, and exposes UAV availability to the UE Mobility xApp through the effective UAV cell capacity. In the satellite-assisted scenario, this UAV Autonomy xApp is hosted by a separate simulated onboard UAV Near-RT RIC object that starts slightly before the terrestrial UE Mobility RIC in each nominal control cycle. The command management module is:

```text
OranCmmHandover
```

It executes the selected NR-to-NR handover command after the logic module chooses a target cell.

The current implementation therefore uses two coupled RIC control authorities. In the TN+UAV+satellite scenario, the simulated onboard UAV Near-RT RIC runs the UAV Autonomy xApp and decides whether each UAV cell is controlled by the terrestrial Near-RT RIC, by the onboard UAV RIC, or by local UAV autonomy. The terrestrial Near-RT RIC runs the UE Mobility xApp, which then selects UE handover targets using the UAV availability state produced by the autonomy xApp. Therefore, a UAV cell remains a valid UE handover target only when terrestrial xHaul is healthy or onboard UAV RIC fallback with satellite backhaul is available.

In ns-O-RAN, E2 reporting and command delivery are represented by virtual interfaces between E2 terminators and the Near-RT RIC rather than by physical IP packets. Therefore, this scenario records the intended control path as a control-state decision:

```text
TN_E2
ONBOARD_LOCAL_CONTROL_WITH_SAT_FEEDBACK
ONBOARD_LOCAL_CONTROL
LOCAL_AUTONOMY
```

The trace also records the active UAV control authority:

```text
TN_NEAR_RT_RIC
ONBOARD_UAV_RIC
LOCAL_UAV_AUTONOMY
```

The `ActiveUavRic` field should be read as "who is currently responsible for controlling the UAV cell":

| `ActiveUavRic` value | Meaning | When it appears |
|---|---|---|
| `TN_NEAR_RT_RIC` | The terrestrial Near-RT RIC is responsible for UAV control. The UAV uses terrestrial xHaul and behaves as a normal aerial coverage-extension cell. The onboard UAV RIC/xApp is standby. | UAV-to-TN xHaul is healthy |
| `ONBOARD_UAV_RIC` | The onboard UAV RIC/xApp becomes active and controls the UAV locally. If satellite backhaul is healthy, the UAV also switches user/core backhaul to `SATELLITE_FALLBACK`. | xHaul is bad, onboard UAV RIC is enabled, and `--enable-onboard-uav-ric=1` |
| `LOCAL_UAV_AUTONOMY` | No usable TN control path or onboard-RIC fallback policy is available. The UAV is treated as a local autonomous island, and normal UE handover to it is blocked. | xHaul is bad and onboard UAV RIC fallback is unavailable |

This means the simulation models whether UAV control is terrestrial, onboard-UAV-RIC-assisted, or local/autonomous. The satellite-assisted case uses a separate simulated onboard UAV RIC object for UAV autonomy decisions. However, the actual ns-O-RAN E2 message delivery still uses simulator object interfaces rather than packet-level E2 transport over the satellite link.

The onboard UAV RIC authority is only enabled in the `tn-uav-satellite` scenario using:

```text
--enable-onboard-uav-ric=1
```

It is disabled automatically for `tn-only` and `tn-uav`. Therefore, the onboard-RIC fallback logic does not alter the clean terrestrial baseline or the healthy-xHaul TN+UAV case.

## 5. Deployment Scenarios

### 5.1 Scenario 1: UE + TN Only

This is the terrestrial baseline.

```text
UEs -> terrestrial gNBs -> terrestrial/core network
```

There are no UAV cells and no satellite support. UEs can only connect to terrestrial gNBs. This scenario answers:

```text
How well does the terrestrial network perform by itself?
```

Expected observations include the baseline delay, throughput, packet delivery ratio, and handover behavior under only terrestrial coverage. In the current comparison script, this scenario is run without artificial TN degradation so it represents a clean semi-urban terrestrial reference. No UAV cells are installed, the satellite monitor is disabled, and the onboard UAV RIC authority is disabled.

### 5.2 Scenario 2: UE + TN + UAV

This scenario adds UAV-mounted cell nodes.

```text
UEs -> terrestrial gNBs
UEs -> UAV cell -> terrestrial donor/core network
```

The UAV improves radio access coverage, especially for UEs that are poorly served by ground cells. However, the UAV still depends on a UAV-to-ground TN donor connection. In this work, that donor connection is treated as the wireless xHaul link.

Each UAV gNB selects the best available terrestrial donor gNB within a 10 km donor range. As the UAV moves, the xHaul monitor re-evaluates all TN gNBs and records the currently selected donor cell. If no TN donor is within this range, the UAV xHaul state is marked `UNREACHABLE`. In the TN + UAV scenario, the UAV is expected to remain connected through terrestrial xHaul; the unreachable case is mainly used later to motivate the satellite-assisted fallback scenario.

In this scenario, satellite support is disabled. Therefore, when UAV xHaul is healthy the expected control state is:

```text
BackhaulMode = TN_DIRECT
ControlPath = TN_E2
ActiveUavRic = TN_NEAR_RT_RIC
NormalUeHandoverAllowed = 1
```

This means UAV access and UE handover remain controlled through the terrestrial Near-RT RIC.

The xHaul state is classified using the estimated UAV-to-TN donor RSRP:

```text
HEALTHY
DEGRADED
UNREACHABLE
```

The UAV autonomy mode is then selected as:

```text
TN_CONTROLLED_COVERAGE_EXTENSION
ONBOARD_AUTONOMY_WITH_SATELLITE_BACKHAUL
ONBOARD_EMERGENCY_CONTROL_WITH_SATELLITE_BACKHAUL
ONBOARD_LOCAL_AUTONOMY_SERVICE_LIMITED
AUTONOMOUS_LOCAL_ISLAND
```

The UE mobility and UAV autonomy policies are coupled through one rule:

```text
Healthy xHaul:
    UAV remains available for normal UE handover.

Degraded or unreachable xHaul with healthy satellite:
    onboard UAV RIC/xApp becomes active, the UAV switches to satellite
    fallback, and the UAV remains available for UE handover.

Degraded or unreachable xHaul without satellite:
    UAV is treated as a local/autonomous island for normal service, and
    ordinary UE handover to that UAV cell is blocked by setting its effective
    O-RAN cell capacity to 0.
```

This scenario answers:

```text
Does adding UAV coverage improve service compared with TN-only, and when does the UAV become limited by its terrestrial xHaul?
```

### 5.3 Scenario 3: UE + TN + UAV + Satellite

This scenario adds satellite backhaul monitoring to the TN + UAV deployment.

```text
UEs -> UAV cell -> terrestrial donor/core network
              \
               -> SAT -> GW -> core network fallback
```

The satellite path is used as a fallback UAV backhaul route when terrestrial xHaul is degraded or unavailable. In normal conditions, the satellite may not significantly improve UE access coverage because the UAV already serves as the aerial radio node. Its main value appears when terrestrial xHaul or infrastructure reachability becomes unreliable. In the simulation, this fallback is represented by switching the UAV S1-U route from the direct TN/core path to the UAV gNB -> SAT -> GW -> core path.

The same degradation event also changes the logical UAV control authority. During healthy xHaul, the UAV remains controlled by the terrestrial Near-RT RIC. During degraded or unreachable xHaul with healthy satellite support, the UAV autonomy trace records:

```text
BackhaulMode = SATELLITE_FALLBACK
ControlPath = ONBOARD_LOCAL_CONTROL_WITH_SAT_FEEDBACK
ActiveUavRic = ONBOARD_UAV_RIC
NormalUeHandoverAllowed = 1
```

This represents the article-level idea of an onboard UAV RIC/xApp executing autonomy control when the terrestrial control/xHaul link fails. In the current ns-O-RAN implementation, the onboard UAV RIC is a separate simulated RIC object, but its local/E2-like interaction is still not transported as physical packets over an onboard control bus. The satellite path supports UAV backhaul and optional remote coordination.

This scenario answers:

```text
Can satellite support improve service continuity when the UAV-to-TN xHaul is degraded?
```

## 6. Experimental Configuration

The current experiment uses the following settings:

| Parameter | Value |
|---|---:|
| Simulation time | 40 s |
| TN-only monitored UEs, UES1 | 20 |
| TN-only background/load UEs, UES2 | 50 |
| TN-only total UEs | 70 |
| TN-only UES2 attach time | 5 s |
| UAV-scenario monitored UEs, UES1 | 50 |
| UAV-scenario background/load UEs, UES2 | 50 |
| UAV-scenario total UEs | 100 |
| UAV-scenario UES2 attach time | 5 s |
| Terrestrial gNBs | 4 |
| UAV gNBs, when enabled | 3 |
| TN cell capacity | 20 UEs per cell |
| UAV/NTN cell capacity | 10 UEs |
| UAV-to-TN donor range | 10 km |
| Hysteresis | 2 dB |
| RLC mode for comparison runs | AM |
| Monitored traffic model | UDP |
| Monitored DL offered rate | 0.2 Mbps per UES1 UE |
| Monitored UL offered rate | 0.05 Mbps per UES1 UE |
| Monitored packet size | 1000 bytes |
| TN-only degradation window | None |
| Healthy TN + UAV degradation window | None |
| Healthy TN + UAV xHaul condition | Healthy donor selected within 10 km |
| Degraded TN + UAV baseline | Same natural mission/xHaul stress as satellite case, but no satellite fallback |
| Satellite-scenario TN degradation window | None |
| TN gNB TxPower penalty during satellite scenario | 0 dB |
| Natural mission movement start, degraded UAV runs | 15 s |
| UAV underserved-UE RSRP threshold | -105 dBm |
| UAV initial placement half-width, degraded UAV runs | 3 km |
| UAV initial placement half-height, degraded UAV runs | 1.5 km |
| UAV mission mobility half-width, degraded UAV runs | 9 km |
| UAV mission mobility half-height, degraded UAV runs | 4.5 km |
| UAV mission target scale, degraded UAV runs | 2.2 |
| UAV mission speed, degraded UAV runs | 220 m/s |
| UAV xHaul degradation method, degraded UAV runs | Donor distance + channel variation |
| xHaul healthy RSRP threshold, satellite scenario | -72 dBm |
| xHaul degraded RSRP threshold, satellite scenario | -82 dBm |
| xHaul shadowing standard deviation | 6 dB |
| xHaul fading-loss standard deviation | 4 dB |
| Synthetic xHaul penalty | Disabled |

Run all four scenarios with:

```bash
bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

The four individual modes are:

```bash
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only --run-label=clean --sim-time=40 --num-uess1=20 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=0 --max-ues-tn=20 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=healthy-xhaul --sim-time=40 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=natural-xhaul-no-sat --sim-time=40 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --tn-degradation-start=-1 --tn-degradation-stop=-1 --tn-degradation-penalty-db=0 --uav-control-start=15 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-105 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=9000 --uav-area-half-h-m=4500 --uav-mission-target-scale=2.2 --uav-speed-mps=220 --xhaul-degradation-start=-1 --xhaul-degradation-stop=-1 --xhaul-degradation-penalty-db=0 --xhaul-healthy-rsrp-dbm=-72 --xhaul-degraded-rsrp-dbm=-82 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --run-label=natural-xhaul-sat --sim-time=40 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --enable-sat-backhaul-monitor=1 --enable-onboard-uav-ric=1 --tn-degradation-start=-1 --tn-degradation-stop=-1 --tn-degradation-penalty-db=0 --uav-control-start=15 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-105 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=9000 --uav-area-half-h-m=4500 --uav-mission-target-scale=2.2 --uav-speed-mps=220 --xhaul-degradation-start=-1 --xhaul-degradation-stop=-1 --xhaul-degradation-penalty-db=0 --xhaul-healthy-rsrp-dbm=-72 --xhaul-degraded-rsrp-dbm=-82 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"
```

The TN-only scenario is the clean reference and does not include artificial degradation. The healthy TN + UAV scenario adds aerial access cells under healthy terrestrial xHaul, so it tests whether UAVs help when the TN layer has limited capacity for the 100-UE load. The degraded TN + UAV no-satellite baseline uses the same natural mission movement and xHaul thresholds as the satellite-assisted run, but satellite backhaul and onboard UAV RIC fallback are disabled. This gives a fair baseline for asking whether satellite support helps during weak UAV-to-TN xHaul. No fixed time-window RSRP penalty is applied:

| Time interval | Infrastructure condition | Expected interpretation |
|---|---|---|
| 0-15 s | Healthy TN infrastructure and healthy UAV-to-TN xHaul | Normal operation |
| 15-30 s | Degraded UAV runs enter mission movement toward underserved UEs; UAV-to-TN xHaul may become degraded/unreachable according to donor distance and channel variation | No-satellite degraded baseline is compared against satellite fallback |
| 30-40 s | Mission continues with possible xHaul recovery or further degradation depending on UAV location and channel variation | Continuity is compared under natural post-mission dynamics |

## 7. Measured Outputs and KPIs

The main output files are:

| Output file | Purpose |
|---|---|
| `qos-vs-time.txt` | Per-UE delay, jitter, throughput, and packet delivery ratio |
| `xhaul-autonomy-trace.csv` | UAV xHaul donor cell, donor distance, connectivity flag, RSRP, xHaul state, satellite health, onboard UAV RIC availability/state, selected backhaul mode, control path, active UAV RIC, normal-handover permission, and UAV autonomy mode |
| `tn-infrastructure-trace.csv` | TN degradation state and applied TN gNB transmit power |
| `handover-trace.tr` | Successful handover events |
| `handover-failure-trace.tr` | NR RRC handover failure events |
| `ml-ho-dataset.csv` | Candidate-level O-RAN decision records |
| `ns3-oran-lm.log` | Verbose O-RAN logic-module INFO log, including `TRACE HO_SUCCESS`, `TRACE HO_FAILURE`, `UAV_AUTONOMY_XAPP`, and logic-module rejection lines such as low-RSRP or xHaul-blocked handover candidates |
| `uav-position-trace.tr` | UAV movement and position evidence |
| `ues1-position-trace.tr` | Monitored UE position trace |
| `ues2-position-trace.tr` | Background UE position trace |

The key comparison metrics are:

- average downlink and uplink delay;
- average jitter;
- average throughput;
- packet delivery ratio;
- number of successful handovers;
- number of failed handovers;
- xHaul RSRP over time;
- fraction of time in healthy, degraded, and unreachable xHaul states;
- selected UAV autonomy mode over time;
- satellite backhaul SNR and health when satellite monitoring is enabled.

## 8. Comparison Method

The scenarios use the same simulation time, monitored UDP traffic model, TN deployment, and trace collection style. The TN-only scenario is a clean reference with 20 monitored UEs plus 50 delayed background UEs. The UAV scenarios use a larger 50 monitored UE stress case. The intended difference is the infrastructure support level and whether the run includes natural mission-driven xHaul degradation:

| Scenario | Infrastructure support | Main role |
|---|---|---|
| UE + TN only | Terrestrial cells only | Baseline |
| UE + TN + UAV, healthy xHaul | Terrestrial cells plus UAV cells | Aerial coverage extension |
| UE + TN + UAV, natural degraded xHaul, no satellite | Terrestrial cells plus UAV cells | Fair no-satellite degraded baseline |
| UE + TN + UAV + satellite, natural degraded xHaul | Terrestrial cells, UAV cells, satellite monitor, onboard UAV RIC | Aerial coverage plus continuity support |

The expected interpretation is:

```text
TN-only gives the healthy terrestrial reference for semi-urban service.
Healthy TN+UAV shows whether aerial access helps when TN capacity/coverage is insufficient but UAV-to-TN xHaul remains available.
Degraded TN+UAV without satellite shows what happens when UAV access exists but UAV-to-TN xHaul becomes weak.
TN+UAV+satellite shows whether continuity improves under the same weak-xHaul mission condition when satellite fallback is available.
```

The TN-only case is intentionally not degraded; it establishes the normal terrestrial service level. The TN+UAV case then adds three UAV/NTN cells under healthy terrestrial xHaul to test whether aerial cells relieve the 100-UE load. The satellite-assisted case introduces a natural mission movement period from 15 s onward. During this period, UAVs move toward scaled underserved-UE mission targets using a moderated speed and mission area. The goal is to keep UAV-to-UE access useful while allowing the UAV-to-TN donor link to become naturally degraded due to donor distance and urban channel variation. The selected donor distance, donor-range test, urban shadowing/fading variation, and xHaul RSRP thresholds determine whether the UAV remains TN-controlled or switches to onboard autonomy with satellite fallback. This separates the healthy terrestrial reference, the healthy-xHaul UAV access improvement case, and the satellite-assisted continuity case.

### 8.1 Sample Figure Set

Three sample comparison figures were generated from the currently available result folders:

| Figure | File | What it compares |
|---|---|---|
| Figure 1 | `docs/figures/figure1_qos_throughput_delay.png` | Mean downlink throughput and delay over time |
| Figure 2 | `docs/figures/figure2_handover_counts.png` | Successful and failed handover counts |
| Figure 3 | `docs/figures/figure3_xhaul_autonomy_satellite.png` | UAV xHaul RSRP and satellite backhaul SNR |

The generated summary table is:

```text
docs/figures/uav_xhaul_comparison_summary.csv
```

These figures should be treated as preliminary until they are regenerated from the current UDP-based KPI runs. For final article figures, all three runs should complete the full 40 s simulation and produce the same set of output files, especially `qos-vs-time.txt`, `final-flow-report.txt`, `xhaul-autonomy-trace.csv`, and `tn-infrastructure-trace.csv`.

## 9. Expected Findings

The expected outcome is not that satellite always improves every KPI. Instead, the expected behavior is condition-dependent.

In the UE + TN only case, performance represents the clean terrestrial reference. It should show the normal delay, throughput, packet delivery ratio, and handover behavior expected from the semi-urban TN deployment without artificial infrastructure disruption.

In the UE + TN + UAV case, the UAV may improve UE access performance by serving users from a better aerial location. However, this improvement is meaningful only when the UAV has sufficient xHaul connectivity to the terrestrial donor. If the xHaul becomes weak, the UAV may still provide radio coverage, but service continuity can degrade.

In the UE + TN + UAV + satellite case, the satellite path provides a fallback UAV backhaul route. During healthy terrestrial xHaul periods, performance may be similar to the TN + UAV case because the UAV can still use direct TN backhaul. During degraded or isolated periods, the satellite-assisted mode should show stronger continuity because the UAV backhaul can switch to the satellite path.

The main article claim can therefore be:

```text
UAVs improve access coverage, but xHaul-aware autonomy is needed to maintain service continuity.
Satellite assistance is most valuable when terrestrial xHaul or control reachability is degraded.
```

## 10. Current Limitation and Next Step

The current simulation records xHaul health, satellite backhaul health, onboard UAV RIC availability/state, the selected UAV backhaul mode, the intended control path, the active UAV control authority, and whether normal UE handover to a UAV cell is allowed. In the satellite-assisted scenario, the UAV S1-U route can switch from direct TN backhaul to a satellite fallback path when terrestrial xHaul is degraded or unreachable. The UAV Autonomy xApp influences the UE Mobility xApp by setting the effective UAV cell capacity to 0 when the UAV has degraded or unreachable xHaul and no onboard-RIC/satellite-backhaul fallback.

The remaining limitation is transport realism: although the satellite-assisted scenario uses a separate simulated onboard UAV RIC object for UAV autonomy, ns-O-RAN E2 messages still use simulator object interfaces rather than packet-level TN E2 transport or an explicit onboard control-bus model. The satellite backhaul itself is represented as the `UAV gNB -> SAT -> GW -> core` fallback path. The next improvement is to make the policy richer and the transport model more explicit:

1. Model packet-level E2/control-plane transport delay and loss over TN and satellite paths.
2. Make UAV movement objectives, donor selection, and UE handover decisions jointly optimized.
3. Prefer UAV/satellite-supported service when terrestrial infrastructure is degraded.
4. Add service-specific policies for emergency traffic, video traffic, and background traffic.
5. Compare against a baseline that ignores xHaul health and selects cells using only UE-facing RSRP.

This would turn the current monitoring framework into a stronger AI-native or policy-driven control framework.

## 11. Conclusion

This article presented an xHaul-aware UAV O-RAN scenario for evaluating service continuity across TN, UAV, and satellite-assisted deployments. The key contribution is the separation of UE access quality from UAV infrastructure reachability. By comparing UE + TN only, UE + TN + UAV, and UE + TN + UAV + satellite modes, the simulation can show how UAVs improve aerial access coverage and how satellite assistance may improve continuity under terrestrial xHaul degradation. The resulting traces provide a practical basis for future AI-driven handover, UAV positioning, and autonomy-mode control in integrated TN-NTN systems.
