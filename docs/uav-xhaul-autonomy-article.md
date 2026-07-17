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

This ensures that handover decisions are issued by the O-RAN control logic. The default logic module is:

```text
OranLmNr2NrRsrpHandoverWithTnNtn
```

This module uses UE-reported RSRP, hysteresis, minimum acceptable RSRP, cell capacity, TN/NTN cell type, and optional backhaul health information. The command management module is:

```text
OranCmmHandover
```

It executes the selected NR-to-NR handover command after the logic module chooses a target cell.

In the current implementation, the xHaul/autonomy trace is deliberately separated from the handover decision path. It records the evidence needed to compare the three scenarios and, in the satellite-assisted mode, switches the UAV S1-U backhaul route between direct TN backhaul and satellite fallback. It does not yet force UE handovers based on xHaul RSRP, which keeps the O-RAN/xApp handover process clean for baseline comparison.

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

Expected observations include the baseline delay, throughput, packet delivery ratio, and handover behavior under only terrestrial coverage. In the current comparison script, this scenario is run without artificial TN degradation so it represents a clean semi-urban terrestrial reference.

### 5.2 Scenario 2: UE + TN + UAV

This scenario adds UAV-mounted cell nodes.

```text
UEs -> terrestrial gNBs
UEs -> UAV cell -> terrestrial donor/core network
```

The UAV improves radio access coverage, especially for UEs that are poorly served by ground cells. However, the UAV still depends on a UAV-to-ground TN donor connection. In this work, that donor connection is treated as the wireless xHaul link.

Each UAV gNB selects the best available terrestrial donor gNB within a 10 km donor range. As the UAV moves, the xHaul monitor re-evaluates all TN gNBs and records the currently selected donor cell. If no TN donor is within this range, the UAV xHaul state is marked `UNREACHABLE`. In the TN + UAV scenario, the UAV is expected to remain connected through terrestrial xHaul; the unreachable case is mainly used later to motivate the satellite-assisted fallback scenario.

The xHaul state is classified using the estimated UAV-to-TN donor RSRP:

```text
HEALTHY
DEGRADED
UNREACHABLE
```

The UAV autonomy mode is then selected as:

```text
COVERAGE_EXTENSION
EDGE_ASSISTED_LOCAL
EDGE_ASSISTED_WITH_SATELLITE
AUTONOMOUS_EMERGENCY_ISLAND
SATELLITE_ASSISTED_EMERGENCY
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
| Monitored DL offered rate | 1 Mbps per UES1 UE |
| Monitored UL offered rate | 0.25 Mbps per UES1 UE |
| Monitored packet size | 1000 bytes |
| TN-only degradation window | None |
| TN + UAV degradation window | None |
| TN + UAV xHaul condition | Healthy donor selected within 10 km |
| Satellite-scenario TN degradation window | 15-30 s |
| TN gNB TxPower penalty during satellite-scenario degradation | 15 dB |
| Satellite-scenario UAV xHaul degradation window | 15-30 s |
| UAV xHaul RSRP penalty during satellite-scenario degradation | 35 dB |

Run all three scenarios with:

```bash
bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

The three individual modes are:

```bash
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only --sim-time=40 --num-uess1=20 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=0 --max-ues-tn=20 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=1.0 --monitored-ul-rate-mbps=0.25 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --sim-time=40 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=1.0 --monitored-ul-rate-mbps=0.25 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --sim-time=40 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=1.0 --monitored-ul-rate-mbps=0.25 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --enable-sat-backhaul-monitor=1 --tn-degradation-start=15 --tn-degradation-stop=30 --tn-degradation-penalty-db=15 --xhaul-degradation-start=15 --xhaul-degradation-stop=30 --xhaul-degradation-penalty-db=35"
```

The TN-only scenario is the clean reference and does not include artificial degradation. The TN + UAV scenario adds aerial access cells under healthy terrestrial xHaul, so it tests whether UAVs help when the TN layer has limited capacity for the 100-UE load. The satellite-assisted run then adds a matching TN degradation and UAV-to-TN xHaul degradation interval:

| Time interval | Infrastructure condition | Expected interpretation |
|---|---|---|
| 0-15 s | Healthy TN infrastructure and healthy UAV-to-TN xHaul | Normal operation |
| 15-30 s | TN-only and TN+UAV remain healthy; satellite scenario has degraded TN infrastructure and degraded UAV-to-TN xHaul | Satellite fallback is tested under infrastructure stress |
| 30-40 s | Satellite-scenario TN and xHaul recovery | Service should recover toward normal operation |

## 7. Measured Outputs and KPIs

The main output files are:

| Output file | Purpose |
|---|---|
| `qos-vs-time.txt` | Per-UE delay, jitter, throughput, and packet delivery ratio |
| `xhaul-autonomy-trace.csv` | UAV xHaul donor cell, donor distance, connectivity flag, RSRP, xHaul state, satellite health, selected backhaul mode, and UAV autonomy mode |
| `tn-infrastructure-trace.csv` | TN degradation state and applied TN gNB transmit power |
| `handover-trace.tr` | Successful handover events |
| `handover-failure-trace.tr` | NR RRC handover failure events |
| `ml-ho-dataset.csv` | Candidate-level O-RAN decision records |
| `ns3-oran-lm.log` | Verbose O-RAN logic-module INFO log, including `TRACE HO_SUCCESS`, `TRACE HO_FAILURE`, and logic-module rejection lines such as low-RSRP handover candidates |
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

The scenarios use the same simulation time, monitored UDP traffic model, TN deployment, and trace collection style. The TN-only scenario is a clean reference with 20 monitored UEs plus 50 delayed background UEs. The TN+UAV and satellite scenarios use a larger 50 monitored UE stress case. The intended difference is the infrastructure support level and whether the run includes the satellite-scenario degradation window:

| Scenario | Infrastructure support | Main role |
|---|---|---|
| UE + TN only | Terrestrial cells only | Baseline |
| UE + TN + UAV | Terrestrial cells plus UAV cells | Aerial coverage extension |
| UE + TN + UAV + satellite | Terrestrial cells, UAV cells, and satellite monitor | Aerial coverage plus continuity support |

The expected interpretation is:

```text
TN-only gives the healthy terrestrial reference for semi-urban service.
TN+UAV shows whether aerial access helps when TN capacity/coverage is insufficient but UAV-to-TN xHaul remains available.
TN+UAV+satellite shows whether continuity improves when UAV xHaul is weak but satellite fallback is available.
```

The TN-only case is intentionally not degraded; it establishes the normal terrestrial service level. The TN+UAV case then adds three UAV/NTN cells under healthy terrestrial xHaul to test whether aerial cells relieve the 100-UE load. The satellite-assisted case introduces a controlled 15-30 s degradation interval: terrestrial gNB transmit power is reduced, and the UAV-to-TN donor xHaul RSRP estimate receives an additional penalty. This separates the healthy terrestrial reference, the healthy-xHaul UAV access improvement case, and the satellite-assisted continuity case.

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

The current simulation records xHaul health, satellite backhaul health, and the selected UAV backhaul mode. In the satellite-assisted scenario, the UAV S1-U route can switch from direct TN backhaul to a satellite fallback path when terrestrial xHaul is degraded or unreachable. The remaining limitation is that UE handover decisions are still based on the O-RAN RSRP/capacity policy rather than directly using xHaul state.

The next improvement is to close the control loop:

1. Use xHaul state inside the O-RAN logic module.
2. Penalize or avoid UAV cells when their xHaul is unreachable.
3. Prefer UAV/satellite-supported service when terrestrial infrastructure is degraded.
4. Add service-specific policies for emergency traffic, video traffic, and background traffic.
5. Compare against a baseline that ignores xHaul health and selects cells using only UE-facing RSRP.

This would turn the current monitoring framework into a stronger AI-native or policy-driven control framework.

## 11. Conclusion

This article presented an xHaul-aware UAV O-RAN scenario for evaluating service continuity across TN, UAV, and satellite-assisted deployments. The key contribution is the separation of UE access quality from UAV infrastructure reachability. By comparing UE + TN only, UE + TN + UAV, and UE + TN + UAV + satellite modes, the simulation can show how UAVs improve aerial access coverage and how satellite assistance may improve continuity under terrestrial xHaul degradation. The resulting traces provide a practical basis for future AI-driven handover, UAV positioning, and autonomy-mode control in integrated TN-NTN systems.
