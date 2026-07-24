# Toward xHaul-Aware Mission-Adaptive UAV O-RAN for TN-NTN Service Continuity

## Abstract

The integration of terrestrial networks (TN), unmanned aerial vehicle (UAV) cells, and satellite-assisted non-terrestrial networks (NTN) is a promising direction for extending mobile connectivity in congested, remote, and disaster-affected environments. However, UAV-assisted radio access cannot be evaluated only from the user equipment (UE) fronthaul perspective. A UAV cell may provide strong access-link coverage to UEs while still being unable to deliver reliable service if its wireless donor/backhaul connection to the terrestrial network, core network, or control infrastructure is degraded. This article proposes an xHaul-aware mission-adaptive UAV O-RAN scenario for studying progressive UAV autonomy under infrastructure degradation. The same UAV platform is evaluated under three deployment modes: UE + TN only, UE + TN + UAV with healthy donor backhaul, and UE + TN + UAV + satellite with mission-driven donor-backhaul outage. The UAV-to-ground TN donor/backhaul link is used as an xHaul-health indicator, while satellite backhaul monitoring is added in the satellite mode to represent service-continuity support when terrestrial infrastructure becomes degraded or unavailable. The proposed ns-3/ns-O-RAN simulation records QoS, handover, donor-backhaul/xHaul-health, and autonomy-mode traces, enabling a comparative evaluation of terrestrial-only service, UAV-assisted coverage, and satellite-assisted UAV continuity. The study provides a foundation for future AI-native O-RAN control in which UAV network functions adapt dynamically according to access, donor-backhaul, and control-path conditions.

## 1. Introduction

Future 5G-Advanced and 6G systems are expected to support connectivity across heterogeneous three-dimensional network environments. Terrestrial infrastructure alone is often insufficient in remote regions, temporary crowded events, and disaster scenarios where ground base stations may be overloaded, damaged, or partially disconnected. UAV-mounted cells can provide flexible aerial coverage, while satellite links can provide additional resilience when terrestrial backhaul is degraded.

However, the main challenge is not simply whether a UAV can transmit a strong radio signal to UEs. A UAV cell also requires reliable donor/backhaul connectivity toward the terrestrial network, edge cloud, core network, or O-RAN control plane. If the UAV access link is healthy but the UAV-to-ground donor connection is weak, delayed, or unavailable, the UAV may become a coverage island with limited service continuity. Therefore, UAV-assisted TN-NTN systems should be evaluated using both UE-facing radio metrics and infrastructure-facing donor-backhaul/control metrics.

This article focuses on a mission-adaptive UAV O-RAN concept. The UAV changes its operational role depending on infrastructure health. When terrestrial donor-backhaul and control connectivity are healthy, the UAV behaves mainly as an aerial coverage extension. When the donor/backhaul path becomes degraded, it can activate additional local functions and operate in a semi-autonomous mode. When terrestrial connectivity is unavailable, the UAV can escalate to an autonomous emergency-network mode, optionally assisted by satellite backhaul.

## 2. Related Work and Research Gap

This work is related to four active research directions: UAV-assisted O-RAN control, UAV wireless donor/backhaul, TN-NTN service continuity, and satellite-assisted backhaul resilience.

O-RAN-based UAV studies have shown that the Near-RT RIC can host xApps for UAV trajectory, resource allocation, and mobility management. For example, the work "When RAN Intelligent Controller in O-RAN Meets Multi-UAV Enabled Wireless Network" develops an O-RAN-based multi-UAV framework for joint UAV trajectory and offloading-task allocation. More recent xApp-based UAV mobility studies use learning methods such as DDQN to optimize UAV handover and resource decisions inside the Near-RT RIC. These works support the idea of using RIC/xApp intelligence for UAV control, but they mainly focus on UAV mobility, association, or compute/resource allocation rather than satellite fallback for a UAV gNB whose terrestrial donor path is unavailable.

UAV and aerial Integrated Access and Backhaul (IAB) studies show that UAV-mounted network nodes can provide access coverage while depending on wireless backhaul toward terrestrial infrastructure. This supports the central assumption of this article: a UAV gNB may have a good UE access link but still fail to deliver useful service if its donor/backhaul path is weak or unavailable.

TN-NTN and O-RAN NTN studies show that satellite and other non-terrestrial platforms can improve coverage and service continuity when terrestrial infrastructure is limited, overloaded, or damaged. The O-RAN Alliance NTN deployment white paper also emphasizes that TN systems need awareness of NTN state and location information to support service continuity and handover. 3GPP NTN work provides the standards background for integrating satellites, HAPS, and UAV-related non-terrestrial components into 5G systems.

Compared with these related works, this article focuses on a specific service-continuity question:

```text
What happens when a UAV gNB can still serve UEs over the access link,
but its terrestrial donor/backhaul path to the core network becomes unavailable?
```

The proposed comparison separates four cases: clean TN-only service, healthy TN+UAV coverage extension, TN+UAV with unavailable donor backhaul and no satellite, and TN+UAV+satellite with the same donor-backhaul outage. This makes the contribution different from ordinary UAV coverage optimization. The main contribution is the xHaul/donor-backhaul-aware switching logic that decides whether the UAV should use terrestrial donor backhaul, satellite fallback, or no normal service path, and then shows the QoS difference with and without satellite fallback.

Representative related sources include:

| Topic | Example source |
|---|---|
| O-RAN multi-UAV optimization | Pham et al., "When RAN Intelligent Controller in O-RAN Meets Multi-UAV Enabled Wireless Network," IEEE Transactions on Cloud Computing, 2023. https://ieeexplore.ieee.org/document/9844892/ |
| UAV mobility xApp and AI/RL | Qazzaz et al., "xApp Empowered Resource Management for Non-Terrestrial Users in 5G O-RAN Networks," 2026. https://arxiv.org/abs/2605.10704 |
| O-RAN and NTN deployment | O-RAN Alliance, "Deployments of O-RAN-based Non-Terrestrial Networks," 2025. https://mediastorage.o-ran.org/ecosystem-resources/O-RAN-2025.04.02.WP.O-RAN_NTN_Deployments-v08.4.pdf |
| 3GPP NTN standards background | 3GPP TR 38.821, "Solutions for NR to support Non-Terrestrial Networks." https://www.3gpp.org/dynareport/38821.htm |
| IAB and wireless backhaul | Madapatha et al., "A Survey on Integrated Access and Backhaul Networks," Frontiers in Communications and Networks, 2021. https://www.frontiersin.org/journals/communications-and-networks/articles/10.3389/frcmn.2021.647284/full |
| UAV-based IAB | "UAV-Based In-Band Integrated Access and Backhaul for 5G Communications." https://arxiv.org/pdf/1807.07230 |
| TN/NTN service continuity | 5G-STARDUST service-continuity studies for integrated TN/NTN. https://www.5g-stardust.eu/download/service_continuity_paper_vf/ |

## 3. Proposed Concept

The proposed concept is based on progressive UAV autonomy. The same UAV platform does not always use the same backhaul/control mode. Instead, its role depends on the health of the surrounding infrastructure.

In normal conditions, the UAV provides radio coverage while relying on terrestrial infrastructure for network processing, O-RAN control, and core-network connectivity. This is suitable for events, temporary hotspots, or coverage extension where ground infrastructure remains healthy.

In degraded conditions, the UAV detects that its wireless donor/backhaul connection to the terrestrial network is becoming weak. The Near-RT RIC switching xApp can then select satellite fallback, reduce dependency on the terrestrial donor path, and maintain service continuity.

In disaster or isolation conditions, the UAV may lose terrestrial donor-backhaul or control reachability. In this case, it can switch to an autonomous emergency-network mode. If satellite connectivity is available, the satellite path can provide additional backhaul support for emergency service continuity.

The key idea is:

```text
Access coverage alone is not enough.
The UAV must also know whether its donor-backhaul/control path is healthy.
```

## 4. Simulation Environment

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

## 5. Handover and Control Mechanism

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

The decision flow is:

```text
Near-RT RIC query cycle
        |
        v
UAV TN/NTN Switching xApp
        |
        |-- TN donor backhaul healthy?
        |       |
        |       +-- yes --> BackhaulMode = TN_DIRECT
        |                  UAV usable for normal UE service
        |
        |-- TN donor backhaul unavailable?
                |
                |-- satellite healthy?
                        |
                        +-- yes --> BackhaulMode = SATELLITE_FALLBACK
                        |          UAV usable for normal UE service
                        |
                        +-- no  --> BackhaulMode = NO_BACKHAUL_AVAILABLE
                                   UAV blocked for normal UE handover

Then:

UE Mobility xApp
        |
        |-- UAV usable? yes --> UE may handover to UAV gNB if RSRP/capacity are good
        |
        +-- UAV usable? no  --> UE handover to UAV gNB is rejected/blocked
```

In simple terms, "UAV usable" means that the UAV has either a terrestrial donor-backhaul path (`TN_DIRECT`) or a satellite fallback path (`SATELLITE_FALLBACK`). If neither path exists, the UAV may still have radio coverage, but it is not selected for normal UE service because packets cannot reach the core network.

In ns-O-RAN, E2 reporting and command delivery are represented by virtual interfaces between E2 terminators and the Near-RT RIC. This is the same logical RIC connection used for TN gNBs and UAV gNBs. It is separate from the UAV user/core backhaul route. Therefore, this scenario records the RIC switching decision as a control-state value:

```text
TN_E2
NEAR_RT_RIC_SWITCHING_TO_SATELLITE_BACKHAUL
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

This means the simulation models UAV control as a Near-RT RIC policy decision. The satellite-assisted case does not create an onboard UAV RIC and does not route RIC/E2 control through the satellite. Instead, the Near-RT RIC switching xApp changes the UAV user/core backhaul path from the terrestrial donor path to the `UAV gNB -> SAT -> GW -> core` fallback path when the terrestrial donor path is unavailable and satellite backhaul is healthy.

The UAV TN/NTN switching xApp can be explicitly enabled using:

```text
--enable-uav-switching-xapp=1
```

## 6. Deployment Scenarios

### 6.1 Scenario 1: UE + TN Only

This is the terrestrial baseline.

```text
UEs -> terrestrial gNBs -> terrestrial/core network
```

There are no UAV cells and no satellite support. UEs can only connect to terrestrial gNBs. This scenario answers:

```text
How well does the terrestrial network perform by itself?
```

Expected observations include the baseline delay, throughput, packet delivery ratio, and handover behavior under only terrestrial coverage. In the current comparison script, this scenario is run without artificial TN degradation so it represents a clean semi-urban terrestrial reference. No UAV cells are installed, the satellite monitor is disabled, and the UAV switching xApp has no UAV cells to control.

### 6.2 Scenario 2: UE + TN + UAV

This scenario adds UAV-mounted cell nodes.

```text
UEs -> terrestrial gNBs
UEs -> UAV gNB -> TN donor/gateway -> core network
```

The UAV improves radio access coverage, especially for UEs that are poorly served by ground cells. However, the UAV still depends on a UAV-to-ground TN donor/backhaul connection. In this work, that donor connection is treated as an xHaul-health proxy. It should be read as a wireless donor/backhaul health model, not as a full 3GPP IAB implementation and not as the Xn interface itself.

Each UAV gNB selects the best available terrestrial donor gNB or ground gateway using the estimated donor-link RSRP. In the final comparison runs, the hard donor-distance cutoff is disabled, so distance affects the donor path through RSRP/path loss rather than through a separate 10 km yes/no rule. As the UAV moves farther from the TN donor region, the estimated donor RSRP can fall below the usable threshold. In the healthy TN + UAV scenario, the UAV is expected to remain connected through the terrestrial donor/backhaul path; the unreachable case is mainly used later to motivate the satellite-assisted fallback scenario.

In this scenario, satellite support is disabled. Therefore, when the UAV donor-backhaul state is healthy the expected control state is:

```text
BackhaulMode = TN_DIRECT
RicControlState = TN_E2
ActiveUavRic = TN_NEAR_RT_RIC
NormalUeHandoverAllowed = 1
```

This means UAV access remains under the terrestrial Near-RT RIC, while UE serving-cell changes are handled by the UE mobility support logic. The RIC state is not the same as the user/core packet route. NR inter-gNB handover support still uses the simulator's X2/Xn-style handover plumbing; that logical handover interface is separate from the donor-backhaul health proxy used by the UAV TN/NTN switching policy.

The donor-backhaul/xHaul-health state is classified using the estimated UAV-to-TN donor RSRP:

```text
RSRP >= -120 dBm  HEALTHY
RSRP < -120 dBm   UNREACHABLE
```

This is a simulation policy threshold for the UAV-to-TN donor/backhaul link. It is not a fixed 3GPP standard cutoff. The UE handover logic still uses its own minimum acceptable UE-facing RSRP threshold. The `DEGRADED` label is not used in the final comparison policy; the final behavior intentionally uses only `HEALTHY` and `UNREACHABLE` donor states.

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

Unreachable donor backhaul with healthy satellite:
    the Near-RT RIC switching xApp selects satellite fallback, and the
    UAV remains available for UE handover.

Unreachable donor backhaul without satellite:
    no valid UAV-to-core backhaul is available, so the Near-RT RIC switching xApp
    removes/blocks the UAV route to the core,
    and ordinary UE handover to that UAV cell is blocked by setting its
    effective O-RAN cell capacity to 0.
```

This scenario answers:

```text
Does adding UAV coverage improve service compared with TN-only, and when does the UAV become limited by its terrestrial donor backhaul?
```

### 6.3 Scenario 3: UE + TN + UAV + Satellite

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

If the terrestrial donor path is unreachable and the satellite backhaul is also not healthy, the trace records:

```text
BackhaulMode = NO_BACKHAUL_AVAILABLE
NormalUeHandoverAllowed = 0
```

This means the UAV may still radiate an access signal, but it is not allowed as a normal serving target because it has no usable path to the core network.

The same degradation event also changes the logical UAV control authority. During healthy donor backhaul, the UAV remains controlled by the terrestrial Near-RT RIC. During unreachable donor backhaul with healthy satellite support, the UAV autonomy trace records:

```text
BackhaulMode = SATELLITE_FALLBACK
RicControlState = NEAR_RT_RIC_SWITCHING_TO_SATELLITE_BACKHAUL
ActiveUavRic = TN_NEAR_RT_RIC
NormalUeHandoverAllowed = 1
```

This represents the article-level idea of a Near-RT RIC switching xApp preserving UAV service when the terrestrial donor-backhaul path fails. The RIC/xApp control remains the normal logical E2 control connection to the UAV gNB. The satellite path supports only the UAV user/core backhaul route.

This scenario answers:

```text
Can satellite support improve service continuity when the UAV-to-TN donor backhaul becomes unreachable?
```

## 7. Experimental Configuration

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
| TN gNB access-link transmit power | 46 dBm | Realistic macro/small-macro style conducted power used for TN access links. |
| UAV gNB access-link transmit power | 37 dBm | Lower-power aerial gNB setting used for UAV/NTN access links. |
| UE access-link transmit power | 23 dBm | Typical NR UE power-class setting for uplink access transmission. |
| TN cell capacity | 20 UEs per cell | O-RAN policy limit used to model finite TN serving capacity. |
| UAV/NTN cell capacity | 10 UEs | O-RAN policy limit used to model smaller UAV-cell serving capacity. |
| UAV-to-TN donor distance cutoff | Disabled in final runs | No hard 10 km cutoff is used; distance affects donor quality through RSRP/path loss. |
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
| Healthy TN + UAV xHaul condition | Best donor RSRP >= -120 dBm | UAVs are expected to remain connected through TN donor backhaul. |
| Satellite-scenario TN degradation window | None | TN transmit power is not artificially reduced in the satellite run. |
| TN gNB TxPower penalty during satellite scenario | 0 dB | No synthetic TN transmit-power penalty is applied. |
| Natural mission movement start, donor-outage runs | 30 s | UAV movement toward underserved UEs starts after the initial healthy period. |
| UAV underserved-UE RSRP threshold | -120 dBm | UEs below this RSRP are treated as severe weak/cell-edge users for UAV mission targeting. |
| UAV initial placement half-width, donor-outage runs | 3 km | UAVs initially start inside a 6 km-wide region near the TN deployment, so TN donor backhaul is likely available at the beginning. |
| UAV initial placement half-height, donor-outage runs | 1.5 km | UAVs initially start inside a 3 km-high region near the TN deployment, supporting the initial healthy-backhaul period. |
| UAV mission mobility half-width, donor-outage runs | 16 km | During the mission, UAVs may move inside a 32 km-wide region to reach underserved UEs farther from TN donors. |
| UAV mission mobility half-height, donor-outage runs | 8 km | During the mission, UAVs may move inside a 16 km-high region, increasing the chance of donor-backhaul stress. |
| UAV mission target scale, donor-outage runs | 4 | Scales the underserved-UE target region so UAVs move farther toward weak-coverage users. |
| UAV mission speed, donor-outage runs | 25 m/s | Realistic UAV movement speed during mission repositioning. |
| UAV donor-backhaul outage method, donor-outage runs | Donor RSRP, channel variation, and TN donor/gateway unavailability | Degradation is created by geometry/channel behavior and a sudden donor outage event, not by a fixed RSRP penalty. |
| TN donor/gateway unavailable window, donor-outage runs | 30-75 s | Main outage interval where no-satellite UAV backhaul is blocked and satellite fallback can help. |
| xHaul usable RSRP threshold, donor-outage runs | -120 dBm | Policy threshold above which the UAV donor path is treated as healthy; below it the donor path is treated as unreachable. |
| xHaul shadowing standard deviation | 6 dB | Random slow variation applied to the UAV-to-TN donor RSRP proxy. |
| xHaul fading-loss standard deviation | 4 dB | Random fast variation applied to the UAV-to-TN donor RSRP proxy. |
| Synthetic xHaul penalty | Disabled | No artificial time-window RSRP penalty is applied in the final scenario. |

### 7.1 Standards Alignment

The experiment separates standards-based radio/channel assumptions from scenario policy thresholds:

| Item | Status | Explanation |
|---|---|---|
| TN access propagation | Standards-based model | Uses the 3GPP UMa channel model supported by 5G-LENA/ns-3. |
| UAV/NTN access propagation | Standards-based model | Uses the 3GPP NTN-Urban channel model supported by 5G-LENA/ns-3. |
| Satellite fallback/backhaul propagation | Standards-based model | Uses the 3GPP NTN-Suburban channel model supported by 5G-LENA/ns-3. |
| Fading and LOS/NLOS condition updates | Simulator configuration for 3GPP models | The update periods control how often the ns-3 3GPP channel realization and LOS/NLOS state are refreshed. They are simulation-time-resolution settings, not service requirements. |
| UE-facing minimum handover RSRP | Policy threshold using NR measurements | The UE mobility xApp uses RSRP reported through the NR/O-RAN measurement path. The selected cutoff is a handover policy threshold, not a universal 3GPP service-loss value. |
| UAV-to-TN donor usable/unreachable threshold | Proposed xApp policy | 3GPP specifies RSRP measurement/reporting behavior and channel models, but it does not define a universal "healthy UAV donor backhaul" RSRP threshold. The `-120 dBm` cutoff is therefore treated as a RIC policy parameter. |
| TN donor/gateway unavailable interval | Scenario event | Represents operation outside the usable TN donor region or a blocked/damaged terrestrial donor path. It is not a standards parameter. |
| Cell capacity limits | Scenario load-control policy | Used to create comparable congestion/load conditions for the O-RAN handover study. |

Therefore, the standard-compliant part of the study is the use of 3GPP channel models and NR/O-RAN measurement/control mechanisms. The healthy/unreachable donor-backhaul labels are part of the proposed UAV TN/NTN switching xApp policy.

Run the comparison set with:

```bash
bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

For final article comparison, repeat the four main scenarios over multiple random seeds. The recommended first batch is four seeds:

```bash
SEEDS="1 2 3 4" bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

This launches 16 runs in sequence: the four main scenarios are repeated with `RngRun=1`, `RngRun=2`, `RngRun=3`, and `RngRun=4`. The scenario parameters remain the same, while the random seed changes the random channel, mobility, and event realizations. The resulting run labels include the seed number, for example:

```text
clean-seed1
healthy-xhaul-seed1
donor-unavailable-no-sat-seed1
donor-unavailable-sat-seed1
```

The final paper results should report the mean and variation across these seed runs, such as average PDR, throughput, delay, and handover counts over four seeds. This four-seed comparison is intended for the article KPI figures. The larger AI dataset sweep is separate and is used later for model training.

The main individual runs are:

```bash
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only --run-label=clean --sim-time=120 --num-uess1=20 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=0 --max-ues-tn=20 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=46 --uav-tx-power-dbm=37 --ue-tx-power-dbm=23 --init-min-rsrp=-160 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=healthy-xhaul --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=46 --uav-tx-power-dbm=37 --ue-tx-power-dbm=23 --init-min-rsrp=-160 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=donor-unavailable-no-sat --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=46 --uav-tx-power-dbm=37 --ue-tx-power-dbm=23 --init-min-rsrp=-160 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --uav-control-start=30 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-120 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=16000 --uav-area-half-h-m=8000 --uav-mission-target-scale=4 --uav-speed-mps=25 --xhaul-donor-unavailable-start=30 --xhaul-donor-unavailable-stop=75 --xhaul-healthy-rsrp-dbm=-120 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"

./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --run-label=donor-unavailable-sat --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=46 --uav-tx-power-dbm=37 --ue-tx-power-dbm=23 --init-min-rsrp=-160 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --sat-backhaul-scenario=NTN-Suburban --enable-sat-backhaul-monitor=1 --enable-uav-switching-xapp=1 --uav-control-start=30 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-120 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=16000 --uav-area-half-h-m=8000 --uav-mission-target-scale=4 --uav-speed-mps=25 --xhaul-donor-unavailable-start=30 --xhaul-donor-unavailable-stop=75 --xhaul-healthy-rsrp-dbm=-120 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"
```

The TN-only scenario is the clean reference and does not include artificial degradation. The healthy TN + UAV scenario adds aerial access cells under healthy terrestrial donor backhaul, so it tests whether UAVs help when the TN layer has limited capacity for the 100-UE load. The satellite-assisted scenario uses a natural mission movement period from 30 s onward. During this mission period, UAVs move toward underserved UE clusters, which can improve UAV access coverage while naturally increasing the UAV-to-TN donor distance. No fixed time-window RSRP penalty is applied:

| Time interval | Infrastructure condition | Expected interpretation |
|---|---|---|
| 0-30 s | Healthy TN infrastructure and healthy UAV-to-TN donor backhaul | Normal operation |
| 30-75 s | Degraded-donor runs enter mission movement toward underserved UEs; the TN donor/gateway path is unavailable, representing operation outside the usable donor-link coverage region or a blocked terrestrial donor path | No-satellite run should lose/limit UAV backhaul; satellite run should switch to satellite fallback |
| 75-90 s | TN donor/gateway path becomes available again; donor-backhaul health is again determined by donor distance and channel variation | Recovery/fallback release behavior can be observed |
| 90-120 s | Mission continues with possible xHaul recovery or further degradation depending on UAV location and channel variation | Continuity is compared under natural post-mission dynamics |

## 8. Measured Outputs and KPIs

The main output files are:

| Output file | Purpose |
|---|---|
| `qos-vs-time.txt` | Per-UE delay, jitter, throughput, and packet delivery ratio |
| `xhaul-autonomy-trace.csv` | UAV xHaul donor cell, donor distance, connectivity flag, RSRP, xHaul state, TN donor/gateway unavailable flag, satellite health, UAV switching xApp availability/state, selected backhaul mode, RIC control state, active RIC, normal-handover permission, and UAV autonomy mode |
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

## 9. Comparison Method

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

The TN-only case is intentionally not degraded; it establishes the normal terrestrial service level. The TN+UAV case then adds three UAV/NTN cells under healthy terrestrial donor backhaul to test whether aerial cells relieve the 100-UE load. The no-satellite outage and satellite-assisted cases introduce a mission movement period from 30 s onward. During this period, UAVs move toward scaled underserved-UE mission targets using an expanded mission area. The goal is to keep UAV-to-UE access useful while allowing the UAV-to-TN donor path to become unavailable due to donor RSRP loss, urban channel variation, or the sudden TN donor/gateway outage event. The selected donor RSRP and urban shadowing/fading variation determine whether the UAV remains on TN donor backhaul, is blocked without satellite, or switches to satellite fallback.

### 9.1 Expected Figure Set

The following figures are generated as expected/illustrative behavior for the four final comparison cases. They are not measured KPI plots. Final article results should replace these with plots generated from `qos-vs-time.txt`, `final-flow-report.txt`, and `xhaul-autonomy-trace.csv` after all four 120 s simulations complete.

| Figure | File | Expected message |
|---|---|---|
| Expected Figure 0 | `docs/figures/expected_figure0_four_case_architecture.png` | Four-case architecture and service path comparison |
| Expected Figure 1 | `docs/figures/expected_figure1_backhaul_mode_timeline.png` | Backhaul mode should switch from `TN_DIRECT` to `SATELLITE_FALLBACK` only in the satellite case during 30-75 s |
| Expected Figure 2 | `docs/figures/expected_figure2_qos_pdr_delay.png` | Satellite fallback should improve outage-period PDR compared with no-satellite, but with higher delay |
| Expected Figure 3 | `docs/figures/expected_figure3_switching_xapp_availability.png` | Without satellite, UAV normal service should be blocked during donor outage; with satellite, it should remain allowed |

The expected-result summary table is:

```text
docs/figures/expected_results_summary.csv
```

The expected trace behavior is:

```text
0-30 s:   BackhaulMode = TN_DIRECT
30-75 s:  no-satellite case -> NO_BACKHAUL_AVAILABLE
30-75 s:  satellite case    -> SATELLITE_FALLBACK
75-120 s: recovery toward TN_DIRECT when donor RSRP is healthy again
```

## 10. Expected Findings

The expected outcome is not that satellite always improves every KPI. Instead, the expected behavior is condition-dependent.

In the UE + TN only case, performance represents the clean terrestrial reference. It should show the normal delay, throughput, packet delivery ratio, and handover behavior expected from the semi-urban TN deployment without artificial infrastructure disruption.

In the UE + TN + UAV case, the UAV may improve UE access performance by serving users from a better aerial location. However, this improvement is meaningful only when the UAV has sufficient donor-backhaul connectivity to the terrestrial network. If the donor backhaul becomes unreachable, the UAV may still provide radio coverage, but service continuity can degrade.

In the UE + TN + UAV + satellite case, the satellite path provides a fallback UAV backhaul route. During healthy terrestrial donor-backhaul periods, performance may be similar to the TN + UAV case because the UAV can still use direct TN backhaul. During unreachable or isolated periods, the satellite-assisted mode should show stronger continuity because the UAV backhaul can switch to the satellite path.

The strongest expected result is the direct outage-period comparison between the degraded-donor case without satellite and the degraded-donor case with satellite:

```text
Without satellite:
    UAV access coverage is not enough.
    When donor backhaul fails, UAV-served UE QoS drops.

With satellite:
    UAV-served UEs keep receiving service through SAT -> GW -> Core.
    PDR and throughput improve during the donor-backhaul outage.
    Delay is higher than TN_DIRECT because the satellite path is longer,
    but this is preferable to losing 5G service entirely.
```

This result should be verified mainly during the 30-75 s donor-unavailable interval. The key evidence is that the no-satellite run records `NO_BACKHAUL_AVAILABLE` with reduced UAV-served UE PDR/throughput, while the satellite run records `SATELLITE_FALLBACK` with better outage-period packet delivery. In the current configuration, the satellite fallback route adds approximately 241 ms of one-way backhaul delay (`120 ms + 120 ms + 1 ms`) before NR access/RLC/scheduling delay is added. Therefore, the expected satellite-fallback delay is higher than direct TN backhaul, but it is still better than no packet delivery for UAV-served UEs.

The main article claim can therefore be:

```text
UAVs improve access coverage, but xHaul-aware autonomy is needed to maintain service continuity.
Satellite assistance is most valuable when terrestrial donor-backhaul reachability is unavailable.
```

## 11. Scope and Next Step

The current simulation records donor-backhaul/xHaul-health state, satellite backhaul health, UAV switching xApp availability/state, the selected UAV backhaul mode, the RIC control state, the active RIC authority, and whether normal UE handover to a UAV cell is allowed. In the satellite-assisted scenario, the UAV S1-U route can switch from direct TN backhaul to a satellite fallback path when terrestrial donor backhaul is unreachable. The UAV TN/NTN Switching xApp influences the UE Mobility xApp by setting the effective UAV cell capacity to 0 when the UAV has unreachable donor backhaul and no satellite-backhaul fallback.

The scope of this work is UE service continuity through UAV backhaul switching. Therefore, the satellite path is modeled for user/core traffic as:

```text
UE -> UAV gNB -> SAT -> GW -> core network
```

The Near-RT RIC/xApp is used to decide when the UAV should use the TN donor path, when it should switch to satellite fallback, and when UAV handover should be blocked because no valid backhaul exists. The RIC/E2 control connection is modeled as the simulator's normal logical O-RAN control interface to the UAV gNB, while the satellite route is used for UE user/core traffic.

The current implementation uses rule-based xApp logic: donor RSRP, donor outage state, satellite SNR, and cell-capacity limits determine whether the UAV uses TN backhaul, satellite fallback, or no normal service. A useful future extension is to replace or augment these fixed rules with an AI/RL-based UAV TN/NTN switching xApp.

Future work can therefore focus on:

1. Learning UAV movement and satellite-fallback decisions from observed QoS, donor RSRP, satellite SNR, and handover outcomes.
2. Adding service-specific policies for emergency traffic, video traffic, and background traffic.
3. Training the UAV switching xApp to balance access coverage, backhaul availability, delay, packet delivery ratio, and handover stability.

For a paper-quality AI dataset, the current rule-based simulation should be repeated with controlled parameter variation rather than using only one run. Each run can produce labeled samples from `xhaul-autonomy-trace.csv`, `qos-vs-time.txt`, `ml-ho-dataset.csv`, `handover-trace.tr`, and `handover-failure-trace.tr`. The supervised-learning label can be the selected `BackhaulMode`, while the input features can include donor RSRP, satellite SNR, donor outage state, UAV load, UE load, delay, PDR, and handover counters.

Recommended AI-dataset sweep values are:

| Parameter varied | Suggested values | Why it is useful for AI training |
|---|---|---|
| Random seed / run index | 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 | Gives different fading, mobility, and traffic realizations so the model does not learn one fixed trace. |
| UAV speed | 10, 15, 25, 35 m/s | Represents slow patrol, normal mission motion, and faster emergency repositioning. |
| Donor outage window | 30-75 s, 40-90 s, 60-105 s | Teaches the model that fallback decisions are not tied to one fixed time interval. |
| UE load | 50+50, 60+60, 70+70 UEs | Tests light, medium, and high congestion around TN and UAV cells. |
| Monitored DL offered rate | 0.1, 0.2, 0.4 Mbps per monitored UE | Tests whether the switching policy remains valid under different traffic demand levels. |
| Donor healthy RSRP threshold | -110, -120, -125 dBm | Tests sensitivity around cell-edge donor-backhaul conditions. |
| xHaul shadowing standard deviation | 4, 6, 8 dB | Represents low, nominal, and severe large-scale channel variation. |
| xHaul fading-loss standard deviation | 2, 4, 6 dB | Represents low, nominal, and severe fast channel fluctuation. |
| Satellite backhaul scenario | NTN-Suburban, NTN-Urban | Tests whether the model can adapt to different satellite-link conditions. |
| Satellite fallback availability | enabled, disabled | Provides both positive fallback examples and no-backhaul examples. |

The full Cartesian product of all values would be too large. A practical first dataset can use 30-60 simulation runs by varying one or two parameters at a time around the final scenario. This is enough to train an initial supervised classifier and to analyze which features are most important for the switching decision.

The dataset sweep can be launched with:

```bash
bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
```

For a quick smoke test before running the full batch, limit the script to the first two runs:

```bash
RUN_LIMIT=2 bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
```

The sweep script uses unique run labels such as `ai-seed1-sat`, `ai-speed25-no-sat`, and `ai-rsrp-120-sat`. These labels keep the output folders separate so the traces can later be merged into one AI training dataset.

Suitable AI models for this future extension include:

1. A supervised classifier such as Random Forest, XGBoost, or a small neural network to predict `TN_DIRECT`, `SATELLITE_FALLBACK`, or `NO_BACKHAUL_AVAILABLE` from donor RSRP, satellite SNR, UE load, delay, and PDR.
2. A reinforcement-learning agent such as DQN or PPO to learn switching and UAV repositioning actions from rewards based on PDR, delay, handover stability, and backhaul availability.
3. A multi-agent RL approach such as MAPPO if multiple UAVs coordinate movement and backhaul selection together.

This would move the current rule-based framework toward a stronger AI-native UAV O-RAN control framework.

## 12. Remote Server Execution Note

The full 120 s ns-3 runs can take a long time on a remote server. If the terminal session is connected through VPN or SSH, the simulation should be run inside `tmux` so that it continues even if the local connection drops.

Start a persistent terminal session with:

```bash
cd ~/workspace/ns-3-dev-nr-oran
tmux new -s uavsat
```

Then run the selected ns-3 command inside the `tmux` session. For example, the satellite fallback case can be launched as:

```bash
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --run-label=donor-unavailable-sat --sim-time=120 --num-uess1=50 --num-ground-ues=50 --ground-attach-delay=5 --num-tn-gnbs=4 --num-ntn-gnbs=3 --max-ues-tn=20 --max-ues-ntn=10 --rlc-mode=am --monitored-traffic=udp --monitored-dl-rate-mbps=0.2 --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000 --tn-tx-power-dbm=46 --uav-tx-power-dbm=37 --ue-tx-power-dbm=23 --init-min-rsrp=-160 --enable-flow-monitor=1 --enable-rsrp-trace=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-decision-csv=1 --enable-oran-info-log=1 --sat-backhaul-scenario=NTN-Suburban --enable-sat-backhaul-monitor=1 --enable-uav-switching-xapp=1 --uav-control-start=30 --uav-control-period=2 --uav-underserved-rsrp-thresh-dbm=-120 --uav-initial-area-half-w-m=3000 --uav-initial-area-half-h-m=1500 --uav-area-half-w-m=16000 --uav-area-half-h-m=8000 --uav-mission-target-scale=4 --uav-speed-mps=25 --xhaul-donor-unavailable-start=30 --xhaul-donor-unavailable-stop=75 --xhaul-healthy-rsrp-dbm=-120 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4 --channel-update-ms=100 --channel-condition-update-ms=200"
```

To run the recommended four-case, four-seed article batch inside `tmux`, use:

```bash
SEEDS="1 2 3 4" bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
```

Detach from `tmux` without stopping the simulation by pressing:

```text
Ctrl+b, then d
```

Reconnect later with:

```bash
tmux attach -t uavsat
```

The output files can be checked from another terminal while the simulation is still running:

```bash
watch -n 5 'ls -lh results/nr/tn-ntn/tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_donor-unavailable-sat'
```

The UAV switching behavior can also be monitored live:

```bash
tail -f results/nr/tn-ntn/tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_donor-unavailable-sat/xhaul-autonomy-trace.csv
```

A complete final run should produce `final-flow-report.txt`, a `qos-vs-time.txt` trace reaching approximately 120 s, and `xhaul-autonomy-trace.csv` rows after the 75 s donor-outage recovery point.

## 13. Conclusion

This article presented an xHaul-aware UAV O-RAN scenario for evaluating service continuity across TN, UAV, and satellite-assisted deployments. The key contribution is the separation of UE access quality from UAV infrastructure reachability. By comparing UE + TN only, healthy UE + TN + UAV, and UE + TN + UAV + satellite with donor-backhaul stress, the simulation can show how UAVs improve aerial access coverage and how satellite assistance may improve continuity under terrestrial donor-backhaul outage. The resulting traces provide a practical basis for future AI-driven TN/NTN switching, UAV positioning, and autonomy-mode control in integrated TN-NTN systems.
