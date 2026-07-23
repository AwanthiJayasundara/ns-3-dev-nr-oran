# Toward xHaul-Aware Mission-Adaptive UAV O-RAN for TN-NTN Service Continuity

## Abstract

The integration of terrestrial networks (TN), unmanned aerial vehicle (UAV) cells, and satellite-assisted non-terrestrial networks (NTN) is a promising direction for extending mobile connectivity in congested, remote, and disaster-affected environments. However, UAV-assisted radio access cannot be evaluated only from the user equipment (UE) fronthaul perspective. A UAV cell may provide strong access-link coverage to UEs while still being unable to deliver reliable service if its wireless donor/backhaul connection to the terrestrial network, core network, or control infrastructure is degraded. This article proposes an xHaul-aware mission-adaptive UAV O-RAN scenario for studying progressive UAV autonomy under infrastructure degradation. The same UAV platform is evaluated under three deployment modes: UE + TN only, UE + TN + UAV with healthy donor backhaul, and UE + TN + UAV + satellite with mission-driven donor-backhaul degradation. The UAV-to-ground TN donor/backhaul link is used as an xHaul-health indicator, while satellite backhaul monitoring is added in the satellite mode to represent service-continuity support when terrestrial infrastructure becomes degraded or unavailable. The proposed ns-3/ns-O-RAN simulation records QoS, handover, donor-backhaul/xHaul-health, and autonomy-mode traces, enabling a comparative evaluation of terrestrial-only service, UAV-assisted coverage, and satellite-assisted UAV continuity. The study provides a foundation for future AI-native O-RAN control in which UAV network functions adapt dynamically according to access, donor-backhaul, and control-path conditions.

## 1. Introduction

Future 5G-Advanced and 6G systems are expected to support connectivity across heterogeneous three-dimensional network environments. Terrestrial infrastructure alone is often insufficient in remote regions, temporary crowded events, and disaster scenarios where ground base stations may be overloaded, damaged, or partially disconnected. UAV-mounted cells can provide flexible aerial coverage, while satellite links can provide additional resilience when terrestrial backhaul is degraded.

However, the main challenge is not simply whether a UAV can transmit a strong radio signal to UEs. A UAV cell also requires reliable donor/backhaul connectivity toward the terrestrial network, edge cloud, core network, or O-RAN control plane. If the UAV access link is healthy but the UAV-to-ground donor connection is weak, delayed, or unavailable, the UAV may become a coverage island with limited service continuity. Therefore, UAV-assisted TN-NTN systems should be evaluated using both UE-facing radio metrics and infrastructure-facing donor-backhaul/control metrics.

This article focuses on a mission-adaptive UAV O-RAN concept. The UAV changes its operational role depending on infrastructure health. When terrestrial donor-backhaul and control connectivity are healthy, the UAV behaves mainly as an aerial coverage extension. When the donor/backhaul path becomes degraded, it can activate additional local functions and operate in a semi-autonomous mode. When terrestrial connectivity is unavailable, the UAV can escalate to an autonomous emergency-network mode, optionally assisted by satellite backhaul.

## 2. Proposed Concept

The proposed concept is based on progressive UAV autonomy. The same UAV platform does not always use the same backhaul/control mode. Instead, its role depends on the health of the surrounding infrastructure.

In normal conditions, the UAV provides radio coverage while relying on terrestrial infrastructure for network processing, O-RAN control, and core-network connectivity. This is suitable for events, temporary hotspots, or coverage extension where ground infrastructure remains healthy.

In degraded conditions, the UAV detects that its wireless donor/backhaul connection to the terrestrial network is becoming weak. The Near-RT RIC switching xApp can then select satellite fallback, reduce dependency on the terrestrial donor path, and maintain service continuity.

In disaster or isolation conditions, the UAV may lose terrestrial donor-backhaul or control reachability. In this case, it can switch to an autonomous emergency-network mode. If satellite connectivity is available, the satellite path can provide additional backhaul support for emergency service continuity.

The key idea is:

```text
Access coverage alone is not enough.
The UAV must also know whether its donor-backhaul/control path is healthy.
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
- O-RAN Near-RT RIC-based UE mobility support and UAV TN/NTN switching control;
- UAV-to-ground TN donor/backhaul health monitoring;
- optional satellite backhaul monitoring;
- QoS, handover, position, decision, and autonomy traces.

Terminology note: this work uses `xHaul` as a broad service-continuity term for the UAV infrastructure-facing transport/control health. The simulated UAV-to-TN metric is more precisely a wireless donor/backhaul health proxy between a UAV gNB and terrestrial donor infrastructure. The simulator does not implement a separate full 3GPP IAB protocol stack, IAB-MT/IAB-DU split, BAP routing, or packet-level Xn/E2 transport.

The first UE group, UES1, means UE Set 1: the monitored UEs used for traffic, mobility, and QoS evaluation. The second UE group, UES2, means UE Set 2: delayed background-load UEs. In the current TN-only reference setup, 20 UES1 nodes are available from the start, and 50 UES2 background UEs attach at 5 s. The UAV-assisted scenarios use a larger monitored group, currently 50 UES1 and 50 UES2, when stress-testing UAV and satellite continuity.

## 4. Handover and Control Mechanism

Initial UE attachment is performed using the strongest-RSRP cell selection mechanism:

```text
AttachToMaxRsrpGnb()
```

Runtime handover is controlled through the O-RAN logic module rather than the native NR handover algorithm. The NR helper keeps the native handover algorithm as:

```text
NrNoOpHandoverAlgorithm
```

This ensures that runtime UE association changes are issued by the O-RAN control logic rather than by the native NR handover algorithm. In this implementation, UE mobility is handled by the support logic module:

```text
OranLmNr2NrRsrpHandoverWithTnNtn
```

This module uses UE-reported RSRP, hysteresis, minimum acceptable RSRP, cell capacity, TN/NTN cell type, and optional backhaul health information. It is not the main contribution of the paper; it is the simulation mechanism that lets UEs attach to either TN gNBs or UAV gNBs.

The main proposed control function is the UAV TN/NTN Switching xApp:

```text
OranLmUavAutonomyControl
```

This xApp monitors UAV donor-backhaul/xHaul-health and satellite health, selects whether the UAV uses terrestrial or satellite backhaul, and exposes UAV availability to the UE mobility logic through the effective UAV cell capacity. It is hosted in the Near-RT RIC together with the UE mobility xApp. The UAV switching xApp runs first in each RIC query cycle, so the UE mobility xApp sees the updated UAV backhaul state before selecting TN or UAV serving-cell changes. The command management module is:

```text
OranCmmHandover
```

It executes the selected NR-to-NR handover command after the logic module chooses a target cell.

The current implementation therefore uses two coupled xApps in one Near-RT RIC. The UAV TN/NTN Switching xApp decides whether each UAV uses the terrestrial donor path, satellite fallback, or an unavailable/blocked path. The UE mobility support xApp then selects UE serving-cell changes using the UAV availability state produced by the switching xApp. Therefore, a UAV gNB remains a valid serving target only when the terrestrial donor-backhaul state is healthy or satellite fallback is available.

In ns-O-RAN, E2 reporting and command delivery are represented by virtual interfaces between E2 terminators and the Near-RT RIC rather than by physical IP packets. Therefore, this scenario records the intended control path as a control-state decision:

```text
TN_E2
SATELLITE_BACKHAUL_CONTROL
NEAR_RT_RIC_SWITCHING_NO_SATELLITE_BACKHAUL
LOCAL_AUTONOMY
```

The trace also records the active UAV control authority:

```text
TN_NEAR_RT_RIC
LOCAL_UAV_AUTONOMY
```

The `ActiveUavRic` field should be read as "who is currently responsible for controlling the UAV cell":

| `ActiveUavRic` value | Meaning | When it appears |
|---|---|---|
| `TN_NEAR_RT_RIC` | The Near-RT RIC is responsible for UAV control. Its switching xApp either keeps the UAV on terrestrial donor backhaul or switches the UAV user/core path to satellite fallback. | UAV switching xApp is enabled |
| `LOCAL_UAV_AUTONOMY` | No usable RIC switching policy is available. The UAV is treated as a local autonomous island, and normal UE handover to it is blocked. | UAV switching xApp is disabled and donor backhaul is bad |

This means the simulation models UAV control as a Near-RT RIC policy decision. The satellite-assisted case does not create an onboard UAV RIC. Instead, the Near-RT RIC switching xApp changes the UAV user/core backhaul path from the terrestrial donor path to the `UAV gNB -> SAT -> GW -> core` fallback path when the terrestrial donor path is unavailable and satellite backhaul is healthy. The actual ns-O-RAN E2 message delivery still uses simulator object interfaces rather than packet-level E2 transport over the satellite link.

The UAV TN/NTN switching xApp can be explicitly enabled using:

```text
--enable-uav-switching-xapp=1
```

The deprecated alias `--enable-onboard-uav-ric=1` is still accepted for old commands, but it no longer creates an onboard RIC. It maps to the Near-RT RIC switching xApp.

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

Expected observations include the baseline delay, throughput, packet delivery ratio, and handover behavior under only terrestrial coverage. In the current comparison script, this scenario is run without artificial TN degradation so it represents a clean semi-urban terrestrial reference. No UAV cells are installed, the satellite monitor is disabled, and the UAV switching xApp has no UAV cells to control.

### 5.2 Scenario 2: UE + TN + UAV

This scenario adds UAV-mounted cell nodes.

```text
UEs -> terrestrial gNBs
UEs -> UAV gNB -> TN donor/gateway -> core network
```

The UAV improves radio access coverage, especially for UEs that are poorly served by ground cells. However, the UAV still depends on a UAV-to-ground TN donor/backhaul connection. In this work, that donor connection is treated as an xHaul-health proxy. It should be read as a wireless donor/backhaul health model, not as a full 3GPP IAB implementation and not as the Xn interface itself.

Each UAV gNB selects the best available terrestrial donor gNB or ground gateway within a 10 km donor range. As the UAV moves, the donor-backhaul monitor re-evaluates all TN donor candidates and records the currently selected donor cell. If no TN donor is within this range, the UAV donor-backhaul state is marked `UNREACHABLE`. In the healthy TN + UAV scenario, the UAV is expected to remain connected through the terrestrial donor/backhaul path; the unreachable case is mainly used later to motivate the satellite-assisted fallback scenario.

In this scenario, satellite support is disabled. Therefore, when the UAV donor-backhaul state is healthy the expected control state is:

```text
BackhaulMode = TN_DIRECT
ControlPath = TN_E2
ActiveUavRic = TN_NEAR_RT_RIC
NormalUeHandoverAllowed = 1
```

This means UAV access remains under the terrestrial Near-RT RIC, while UE serving-cell changes are handled by the UE mobility support logic. NR inter-gNB handover support still uses the simulator's X2/Xn-style handover plumbing; that logical handover interface is separate from the donor-backhaul health proxy used by the UAV TN/NTN switching policy.

The donor-backhaul/xHaul-health state is classified using the estimated UAV-to-TN donor RSRP:

```text
RSRP >= -90 dBm             HEALTHY
-110 dBm <= RSRP < -90 dBm  DEGRADED
RSRP < -110 dBm             UNREACHABLE
```

These are simulation policy thresholds for the UAV-to-TN donor/backhaul link. They are not fixed 3GPP standard cutoffs. The UE handover logic still uses its own minimum acceptable UE-facing RSRP threshold.

The UAV autonomy mode is then selected as:

```text
TN_CONTROLLED_COVERAGE_EXTENSION
NEAR_RT_RIC_SWITCHING_WITH_SATELLITE_BACKHAUL
NEAR_RT_RIC_EMERGENCY_SWITCHING_WITH_SATELLITE_BACKHAUL
NEAR_RT_RIC_SERVICE_LIMITED_NO_SATELLITE_BACKHAUL
AUTONOMOUS_LOCAL_ISLAND
```

The UE mobility and UAV autonomy policies are coupled through one rule:

```text
Healthy donor backhaul:
    UAV remains available for normal UE handover.

Degraded or unreachable donor backhaul with healthy satellite:
    the Near-RT RIC switching xApp selects satellite fallback, and the
    UAV remains available for UE handover.

Degraded or unreachable donor backhaul without satellite:
    the Near-RT RIC switching xApp removes/blocks the UAV route to the core,
    and ordinary UE handover to that UAV cell is blocked by setting its
    effective O-RAN cell capacity to 0.
```

This scenario answers:

```text
Does adding UAV coverage improve service compared with TN-only, and when does the UAV become limited by its terrestrial donor backhaul?
```

### 5.3 Scenario 3: UE + TN + UAV + Satellite

This scenario adds satellite backhaul monitoring to the TN + UAV deployment.

```text
UEs -> UAV gNB -> TN donor/gateway -> core network
             \
              -> SAT -> GW -> core network fallback
```

The satellite path is used as a fallback UAV backhaul route when terrestrial donor backhaul is degraded or unavailable. In normal conditions, the satellite may not significantly improve UE access coverage because the UAV already serves as the aerial radio node. Its main value appears when terrestrial donor-backhaul or infrastructure reachability becomes unreliable. In the simulation, this fallback is represented by switching the UAV S1-U route from the direct TN/core path to the UAV gNB -> SAT -> GW -> core path. In NTN terminology, the UAV satellite terminal to satellite hop is the service link, while the satellite to ground gateway hop is the feeder link.

The SAT node is an ns-3 node with a geocentric mobility position, NTN propagation/channel models, satellite-link SNR monitoring, and IP forwarding behavior. In the current configuration, the SAT node is placed at the scenario reference latitude/longitude with a GEO-like altitude, while the satellite gateway is placed at the same reference point at near-ground altitude:

```text
Satellite altitude = 35,786,000 m
Gateway altitude   = 20 m
```

The fallback route is installed as an IP path through the SAT and GW nodes:

```text
Downlink: Remote host/core -> SGW/Core -> GW -> SAT -> UAV gNB -> UE
Uplink:   UE -> UAV gNB -> SAT -> GW -> SGW/Core -> remote host/core
```

The route delay is modeled using point-to-point satellite-route links:

```text
UAV gNB -> SAT delay = 120 ms
SAT -> GW delay      = 120 ms
GW -> SGW/Core delay = 1 ms
One-way satellite fallback delay = 241 ms
```

The satellite monitor evaluates both the service link and feeder link. In simplified link-budget notation:

```text
P_rx,dB = P_tx,dB + G_tx,dB + G_rx,dB - L_NTN,dB
N_dB    = kTB_dB + NF_dB
SNR_dB  = P_rx,dB - N_dB
```

where `L_NTN,dB` is the loss produced by the selected 3GPP NTN propagation/channel model, `B` is the configured satellite backhaul bandwidth, and `NF_dB` is the receiver noise figure. In the simulator implementation, the same idea is computed from received and noise power spectral densities:

```text
SNR_dB = 10 log10(sum(PSD_rx) / sum(PSD_noise))
```

The monitored satellite backhaul health is limited by the weaker hop and direction:

```text
BackhaulDlSnr = min(ServiceDlSnr, FeederDlSnr)
BackhaulUlSnr = min(ServiceUlSnr, FeederUlSnr)
SatBackhaulHealthy = min(BackhaulDlSnr, BackhaulUlSnr) >= SatBackhaulMinSnr
```

When `SatBackhaulHealthy = 1` and the terrestrial donor path is degraded or unavailable, the UAV switching xApp selects:

```text
BackhaulMode = SATELLITE_FALLBACK
```

and the EPC helper changes the UAV gNB S1-U route from the TN donor path to the `UAV gNB -> SAT -> GW -> core` path.

The same degradation event also changes the logical UAV control authority. During healthy donor backhaul, the UAV remains controlled by the terrestrial Near-RT RIC. During degraded or unreachable donor backhaul with healthy satellite support, the UAV autonomy trace records:

```text
BackhaulMode = SATELLITE_FALLBACK
ControlPath = SATELLITE_BACKHAUL_CONTROL
ActiveUavRic = TN_NEAR_RT_RIC
NormalUeHandoverAllowed = 1
```

This represents the article-level idea of a Near-RT RIC switching xApp preserving UAV service when the terrestrial donor-backhaul path fails. The satellite path supports the UAV user/core backhaul route, while the RIC policy decision is represented logically by the simulation trace.

This scenario answers:

```text
Can satellite support improve service continuity when the UAV-to-TN donor backhaul is degraded?
```

## 6. Experimental Configuration

The current experiment uses the following settings:

| Parameter | Value | Explanation |
|---|---:|---|
| Simulation time | 120 s | Gives enough time to observe normal operation, donor outage, satellite fallback, recovery, and post-recovery behavior. |
| TN-only monitored UEs, UE Set 1/UES1 | 20 | Main UEs used for QoS and mobility measurements in the clean TN baseline. |
| TN-only background/load UEs, UE Set 2/UES2 | 50 | Additional delayed UEs used to create background load in the TN baseline. |
| TN-only total UEs | 70 | Total UE load in the clean terrestrial reference case. |
| TN-only UES2 attach time | 5 s | Background UEs join after the network has already started. |
| UAV-scenario monitored UEs, UE Set 1/UES1 | 50 | Larger monitored UE group used to stress TN+UAV and satellite-assisted scenarios. |
| UAV-scenario background/load UEs, UE Set 2/UES2 | 50 | Additional delayed UEs used to create congestion and coverage pressure. |
| UAV-scenario total UEs | 100 | Total UE load after 5 s in UAV-assisted scenarios. |
| UAV-scenario UES2 attach time | 5 s | Background UEs enter after initial attachment, creating a load increase. |
| Terrestrial gNBs | 4 | Ground cellular layer used as the TN infrastructure. |
| UAV gNBs, when enabled | 3 | Aerial cells added to improve access coverage/capacity. |
| TN cell capacity | 20 UEs per cell | O-RAN policy limit used to model finite TN serving capacity. |
| UAV/NTN cell capacity | 10 UEs | O-RAN policy limit used to model smaller UAV-cell serving capacity. |
| UAV-to-TN donor range | 10 km | Maximum distance at which the UAV gNB can use a TN donor/gateway path. Beyond this range, the TN donor path is treated as unavailable. |
| TN access channel model | 3GPP UMa | Urban macro model for terrestrial UE-to-gNB access links. |
| UAV/NTN access channel model | 3GPP NTN-Urban | NTN-Urban model for UAV/NTN access links. |
| Satellite fallback/backhaul channel model | 3GPP NTN-Suburban | NTN-Suburban model for the satellite fallback/backhaul monitor. |
| Satellite altitude | 35,786 km | GEO-like satellite altitude used by the SAT node position model. |
| Satellite gateway altitude | 20 m | Near-ground altitude for the satellite gateway node. |
| Satellite backhaul carrier frequency | 20 GHz | Representative Ka-band satellite backhaul carrier. |
| Satellite backhaul bandwidth | 400 MHz | Broadband satellite backhaul bandwidth used in SNR calculation. |
| Satellite backhaul RB bandwidth | 120 kHz | Resource-block bandwidth used for the satellite PSD calculation. |
| Satellite EIRP density | 40 dBW/MHz | Satellite transmit power density used in link-budget/SNR monitoring. |
| Satellite antenna gain | 58.5 dB | SAT antenna gain used for service-link and feeder-link SNR calculations. |
| UAV satellite-terminal antenna gain | 39.7 dB | UAV terminal antenna gain for the UAV-to-SAT service link. |
| Gateway antenna gain | 45 dB | Gateway antenna gain for the SAT-to-GW feeder link. |
| UAV satellite-terminal transmit power | 33 dBm | UAV terminal uplink transmit power toward the satellite. |
| Gateway transmit power | 46 dBm | Gateway feeder-uplink transmit power toward the satellite. |
| UAV/GW/satellite receiver noise figure | 1.2 dB | Receiver noise figure used in satellite SNR calculations. |
| Satellite backhaul minimum SNR threshold | 0 dB | Minimum SNR for treating the satellite backhaul as healthy. |
| UAV gNB -> SAT route delay | 120 ms | One satellite-route hop delay from UAV gNB to SAT. |
| SAT -> GW route delay | 120 ms | One satellite-route hop delay from SAT to gateway. |
| GW -> SGW/Core route delay | 1 ms | Ground gateway to core/EPC route delay. |
| NR access fading | Enabled | Allows time-varying radio conditions on access links. |
| NR access shadowing | Disabled | Keeps access shadowing disabled for repeatability; xHaul variation is modeled separately. |
| Channel matrix update period | 100 ms | How often the channel realization/fading matrix is refreshed. |
| LOS/NLOS condition update period | 200 ms | How often the simulator re-evaluates LOS/NLOS channel condition. |
| Hysteresis | 2 dB | Handover margin used to avoid too-frequent cell changes. |
| RLC mode for comparison runs | AM | Acknowledged Mode, used for reliable RLC behavior in the comparison runs. |
| Monitored traffic model | UDP | Simple traffic model used for final QoS/PDR comparison. |
| Monitored DL offered rate | 0.2 Mbps per monitored UE | Downlink offered load for each monitored UE. |
| Monitored UL offered rate | 0.05 Mbps per monitored UE | Uplink offered load for each monitored UE. |
| Monitored packet size | 1000 bytes | UDP packet size used by monitored traffic. |
| TN-only degradation window | None | TN-only case is a clean reference without artificial infrastructure disruption. |
| Healthy TN + UAV degradation window | None | Healthy TN+UAV case keeps the terrestrial donor path available. |
| Healthy TN + UAV xHaul condition | Healthy donor selected within 10 km | UAVs are expected to remain connected through TN donor backhaul. |
| Satellite-scenario TN degradation window | None | TN transmit power is not artificially reduced in the satellite run. |
| TN gNB TxPower penalty during satellite scenario | 0 dB | No synthetic TN transmit-power penalty is applied. |
| Natural mission movement start, degraded-donor runs | 30 s | UAV movement toward underserved UEs starts after the initial healthy period. |
| UAV underserved-UE RSRP threshold | -110 dBm | UEs below this RSRP are treated as weak/cell-edge users for UAV mission targeting. |
| UAV initial placement half-width, degraded-donor runs | 3 km | UAVs initially start inside a 6 km-wide region near the TN deployment, so TN donor backhaul is likely available at the beginning. |
| UAV initial placement half-height, degraded-donor runs | 1.5 km | UAVs initially start inside a 3 km-high region near the TN deployment, supporting the initial healthy-backhaul period. |
| UAV mission mobility half-width, degraded-donor runs | 16 km | During the mission, UAVs may move inside a 32 km-wide region to reach underserved UEs farther from TN donors. |
| UAV mission mobility half-height, degraded-donor runs | 8 km | During the mission, UAVs may move inside a 16 km-high region, increasing the chance of donor-backhaul stress. |
| UAV mission target scale, degraded-donor runs | 4 | Scales the underserved-UE target region so UAVs move farther toward weak-coverage users. |
| UAV mission speed, degraded-donor runs | 25 m/s | Realistic UAV movement speed during mission repositioning. |
| UAV donor-backhaul degradation method, degraded-donor runs | Donor distance, channel variation, and TN donor/gateway unavailability | Degradation is created by geometry and channel behavior, not by a fixed RSRP penalty. |
| TN donor/gateway unavailable window, degraded-donor runs | 30-75 s | Main outage interval where no-satellite UAV backhaul is blocked and satellite fallback can help. |
| xHaul healthy RSRP threshold, degraded-donor runs | -90 dBm | Policy threshold above which the UAV donor path is treated as healthy. |
| xHaul degraded RSRP threshold, degraded-donor runs | -110 dBm | Policy threshold below which the UAV donor path is treated as unreachable. |
| xHaul shadowing standard deviation | 6 dB | Random slow variation applied to the UAV-to-TN donor RSRP proxy. |
| xHaul fading-loss standard deviation | 4 dB | Random fast variation applied to the UAV-to-TN donor RSRP proxy. |
| Synthetic xHaul penalty | Disabled | No artificial time-window RSRP penalty is applied in the final scenario. |

### 6.1 Standards Alignment

The experiment separates standards-based radio/channel assumptions from scenario policy thresholds:

| Item | Status | Explanation |
|---|---|---|
| TN access propagation | Standards-based model | Uses the 3GPP UMa channel model supported by 5G-LENA/ns-3. |
| UAV/NTN access propagation | Standards-based model | Uses the 3GPP NTN-Urban channel model supported by 5G-LENA/ns-3. |
| Satellite fallback/backhaul propagation | Standards-based model | Uses the 3GPP NTN-Suburban channel model supported by 5G-LENA/ns-3. |
| Fading and LOS/NLOS condition updates | Simulator configuration for 3GPP models | The update periods control how often the ns-3 3GPP channel realization and LOS/NLOS state are refreshed. They are simulation-time-resolution settings, not service requirements. |
| UE-facing minimum handover RSRP | Policy threshold using NR measurements | The UE mobility xApp uses RSRP reported through the NR/O-RAN measurement path. The selected cutoff is a handover policy threshold, not a universal 3GPP service-loss value. |
| UAV-to-TN donor HEALTHY/DEGRADED/UNREACHABLE thresholds | Proposed xApp policy | 3GPP specifies RSRP measurement/reporting behavior and channel models, but it does not define a universal "healthy UAV donor backhaul" RSRP threshold. These thresholds are therefore explicitly treated as RIC policy parameters. |
| TN donor/gateway unavailable interval | Scenario event | Represents operation outside the usable TN donor region or a blocked/damaged terrestrial donor path. It is not a standards parameter. |
| Cell capacity limits | Scenario load-control policy | Used to create comparable congestion/load conditions for the O-RAN handover study. |

Therefore, the standard-compliant part of the study is the use of 3GPP channel models and NR/O-RAN measurement/control mechanisms. The healthy/degraded/unreachable donor-backhaul labels are part of the proposed UAV TN/NTN switching xApp policy.

Run the comparison set with:

```bash
bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

The main individual runs are:

```bash
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only --run-label=clean --sim-time=120 --num-uess1=20 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=0 --max-ues-tn=20 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --sat-backhaul-scenario=NTN-Suburban --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=healthy-xhaul --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --sat-backhaul-scenario=NTN-Suburban --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=donor-unavailable-no-sat --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --sat-backhaul-scenario=NTN-Suburban --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --tn-degradation-start=-1 --tn-degradation-stop=-1 --tn-degradation-penalty-db=0 --uav-control-start=30 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-110 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=16000 --uav-area-half-h-m=8000 --uav-mission-target-scale=4 --uav-speed-mps=25 --xhaul-degradation-start=-1 --xhaul-degradation-stop=-1 --xhaul-degradation-penalty-db=0 --xhaul-donor-unavailable-start=30 --xhaul-donor-unavailable-stop=75 --xhaul-healthy-rsrp-dbm=-90 --xhaul-degraded-rsrp-dbm=-110 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --run-label=donor-unavailable-sat --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=83 --uav-tx-power-dbm=78 --ue-tx-power-dbm=43 --init-min-rsrp=-160 --xhaul-max-donor-distance-m=10000 --sat-backhaul-scenario=NTN-Suburban --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --enable-sat-backhaul-monitor=1 --enable-uav-switching-xapp=1 --tn-degradation-start=-1 --tn-degradation-stop=-1 --tn-degradation-penalty-db=0 --uav-control-start=30 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-110 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=16000 --uav-area-half-h-m=8000 --uav-mission-target-scale=4 --uav-speed-mps=25 --xhaul-degradation-start=-1 --xhaul-degradation-stop=-1 --xhaul-degradation-penalty-db=0 --xhaul-donor-unavailable-start=30 --xhaul-donor-unavailable-stop=75 --xhaul-healthy-rsrp-dbm=-90 --xhaul-degraded-rsrp-dbm=-110 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"
```

The TN-only scenario is the clean reference and does not include artificial degradation. The healthy TN + UAV scenario adds aerial access cells under healthy terrestrial donor backhaul, so it tests whether UAVs help when the TN layer has limited capacity for the 100-UE load. The satellite-assisted scenario uses a natural mission movement period from 30 s onward. During this mission period, UAVs move toward underserved UE clusters, which can improve UAV access coverage while naturally increasing the UAV-to-TN donor distance. No fixed time-window RSRP penalty is applied:

| Time interval | Infrastructure condition | Expected interpretation |
|---|---|---|
| 0-30 s | Healthy TN infrastructure and healthy UAV-to-TN donor backhaul | Normal operation |
| 30-75 s | Degraded-donor runs enter mission movement toward underserved UEs; the TN donor/gateway path is unavailable, representing operation outside the usable donor-link coverage region or a blocked terrestrial donor path | No-satellite run should lose/limit UAV backhaul; satellite run should switch to satellite fallback |
| 75-90 s | TN donor/gateway path becomes available again; donor-backhaul health is again determined by donor distance and channel variation | Recovery/fallback release behavior can be observed |
| 90-120 s | Mission continues with possible xHaul recovery or further degradation depending on UAV location and channel variation | Continuity is compared under natural post-mission dynamics |

## 7. Measured Outputs and KPIs

The main output files are:

| Output file | Purpose |
|---|---|
| `qos-vs-time.txt` | Per-UE delay, jitter, throughput, and packet delivery ratio |
| `xhaul-autonomy-trace.csv` | UAV xHaul donor cell, donor distance, connectivity flag, RSRP, xHaul state, TN donor/gateway unavailable flag, satellite health, UAV switching xApp availability/state, selected backhaul mode, control path, active RIC, normal-handover permission, and UAV autonomy mode |
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
| UE + TN + UAV, healthy donor backhaul | Terrestrial cells plus UAV cells | Aerial coverage extension |
| UE + TN + UAV, degraded donor backhaul, no satellite | Terrestrial cells plus UAV cells | No-satellite degraded baseline |
| UE + TN + UAV + satellite, degraded donor backhaul | Terrestrial cells, UAV cells, satellite monitor, Near-RT RIC switching xApp | Aerial coverage plus continuity support |

The expected interpretation is:

```text
TN-only gives the healthy terrestrial reference for semi-urban service.
Healthy TN+UAV shows whether aerial access helps when TN capacity/coverage is insufficient but UAV-to-TN donor backhaul remains available.
TN+UAV+satellite shows whether satellite fallback activates when the UAV-to-TN donor backhaul becomes unavailable during the mission period.
```

The TN-only case is intentionally not degraded; it establishes the normal terrestrial service level. The TN+UAV case then adds three UAV/NTN cells under healthy terrestrial donor backhaul to test whether aerial cells relieve the 100-UE load. The degraded no-satellite and satellite-assisted cases introduce a mission movement period from 30 s onward. During this period, UAVs move toward scaled underserved-UE mission targets using an expanded mission area. The goal is to keep UAV-to-UE access useful while allowing the UAV-to-TN donor path to become unavailable or naturally degraded due to donor distance and urban channel variation. The selected donor distance, donor-range test, urban shadowing/fading variation, and donor-backhaul RSRP thresholds determine whether the UAV remains on TN donor backhaul, is blocked without satellite, or switches to satellite fallback.

### 8.1 Sample Figure Set

The architecture/concept diagram for the current three-scenario design is:

```text
docs/figures/uav-oran-three-scenarios-updated.png
```

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

These figures should be treated as preliminary until they are regenerated from the current UDP-based KPI runs. For final article figures, all comparison runs should complete the full 120 s simulation and produce the same set of output files, especially `qos-vs-time.txt`, `final-flow-report.txt`, `xhaul-autonomy-trace.csv`, and `tn-infrastructure-trace.csv`.

## 9. Expected Findings

The expected outcome is not that satellite always improves every KPI. Instead, the expected behavior is condition-dependent.

In the UE + TN only case, performance represents the clean terrestrial reference. It should show the normal delay, throughput, packet delivery ratio, and handover behavior expected from the semi-urban TN deployment without artificial infrastructure disruption.

In the UE + TN + UAV case, the UAV may improve UE access performance by serving users from a better aerial location. However, this improvement is meaningful only when the UAV has sufficient donor-backhaul connectivity to the terrestrial network. If the donor backhaul becomes weak, the UAV may still provide radio coverage, but service continuity can degrade.

In the UE + TN + UAV + satellite case, the satellite path provides a fallback UAV backhaul route. During healthy terrestrial donor-backhaul periods, performance may be similar to the TN + UAV case because the UAV can still use direct TN backhaul. During degraded or isolated periods, the satellite-assisted mode should show stronger continuity because the UAV backhaul can switch to the satellite path.

The main article claim can therefore be:

```text
UAVs improve access coverage, but xHaul-aware autonomy is needed to maintain service continuity.
Satellite assistance is most valuable when terrestrial donor-backhaul or control reachability is degraded.
```

## 10. Current Limitation and Next Step

The current simulation records donor-backhaul/xHaul-health state, satellite backhaul health, UAV switching xApp availability/state, the selected UAV backhaul mode, the intended control path, the active RIC authority, and whether normal UE handover to a UAV cell is allowed. In the satellite-assisted scenario, the UAV S1-U route can switch from direct TN backhaul to a satellite fallback path when terrestrial donor backhaul is degraded or unreachable. The UAV TN/NTN Switching xApp influences the UE Mobility xApp by setting the effective UAV cell capacity to 0 when the UAV has degraded or unreachable donor backhaul and no satellite-backhaul fallback.

The remaining limitation is transport realism: ns-O-RAN E2 messages still use simulator object interfaces rather than packet-level TN or satellite E2 transport. The satellite backhaul itself is represented as the `UAV gNB -> SAT -> GW -> core` fallback path. The next improvement is to make the policy richer and the transport model more explicit:

1. Model packet-level E2/control-plane transport delay and loss over TN and satellite paths.
2. Make UAV movement objectives, donor selection, and UE handover decisions jointly optimized.
3. Prefer UAV/satellite-supported service when terrestrial infrastructure is degraded.
4. Add service-specific policies for emergency traffic, video traffic, and background traffic.
5. Compare against a baseline that ignores xHaul health and selects cells using only UE-facing RSRP.

This would turn the current monitoring framework into a stronger AI-native or policy-driven control framework.

## 11. Conclusion

This article presented an xHaul-aware UAV O-RAN scenario for evaluating service continuity across TN, UAV, and satellite-assisted deployments. The key contribution is the separation of UE access quality from UAV infrastructure reachability. By comparing UE + TN only, healthy UE + TN + UAV, and UE + TN + UAV + satellite with donor-backhaul stress, the simulation can show how UAVs improve aerial access coverage and how satellite assistance may improve continuity under terrestrial donor-backhaul degradation. The resulting traces provide a practical basis for future AI-driven TN/NTN switching, UAV positioning, and autonomy-mode control in integrated TN-NTN systems.
