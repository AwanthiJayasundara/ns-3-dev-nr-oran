# ORAN Handover Optimization (ns-3 NR/ORAN)

This repository extends the original ns-3 ORAN handover examples into a **large-scale-optimized, performance-oriented multi-cell experiment** with additional **NR-compatible reporters, learning models, CMM orchestration logic, and SQLite-based data storage**.

---

## 📌 Example: `oran-lte-2-lte-rsrp-ue-handover-simulation.cc`
This example is used to verify the simulation speed compared to NR channel and retrieve the results for more than 50 users. With Nr channel conditions, it's taking a longer time to simulate, and earlier it was suspected that due to the effect of UAV dynamic movements, but it was not the reason behind the NR simulation taking a longer time to end. 

### Background (Original ORAN Example)
The original ORAN example **`oran-lte-2-lte-rsrp-handover-lm-example`** is intentionally minimal and focused:

- **1 LTE UE** moving between **2 eNBs**
- UE reports:
  - **Location**
  - **RSRP / RSRQ**
  - (optionally SINR)
- A simple RIC **Logic Module (LM)** periodically checks PHY measurements
- The LM triggers **only one decision**:
  ✅ whether to **initiate a handover**

This example is mainly meant to demonstrate:

- how to connect ORAN tracing (RSRP/RSRQ/SINR),
- how to add LM processing delays,
- and how to trigger a handover from the RIC.

---

### What This Script Adds (My Extended Scenario)
In **`oran-lte-2-lte-rsrp-ue-handover-simulation.cc`**, the same *handover steering idea* is expanded into a **large-scale multi-cell experiment** designed for performance evaluation.

#### Key Enhancements
✅ **Multi-cell deployment**
- **7 macro LTE cells**
- **100 UEs**

✅ **Realistic mobility**
- Random mobility using a **box model**
- Total distance span: **~1000 meters**

✅ **Continuous application traffic**
- UDP traffic flows generated throughout the simulation

✅ **QoS Metrics using FlowMonitor**
Collected KPIs include:
- **End-to-end delay**
- **Jitter**
- **Throughput**
- **Packet Delivery Ratio (PDR)**

✅ **Cell Load Reporting**
- MAC scheduler load reporting
- Used to monitor network utilization at scale

✅ **ORAN Reporting + RSRP-based HO Logic**
- ORAN reporters continuously feed RIC data
- A **RSRP-based LM** uses PHY info to steer handovers for many UEs

---

### Summary Comparison
| Feature | Original Example | This Script |
|--------|------------------|------------|
| Cells | 2 eNBs | 7 macro eNBs |
| UEs | 1 UE | 100 UEs |
| Measurements | RSRP/RSRQ (+SINR optional) | RSRP/RSRQ (+optional SINR), cell info, cell load |
| Decision | Single HO trigger | HO steering for many UEs + KPI evaluation |
| Traffic | None / minimal | Continuous UDP traffic |
| Metrics | PHY tracing demo | QoS: delay, jitter, throughput, PDR |
| Goal | Demonstrate wiring | Large-scale experiment + performance pipeline |

---

### ⏱ Simulation Time & Runtime Notes
- Simulation duration: **~100 seconds**
- Runtime for **100 UEs + 7 eNBs**: **~60 minutes**
- Total scenario span: **~1000 meters**

> Note: runtime depends on machine and build type (debug vs optimized).

---

## 📡 Cell Load Reporters (LTE + NR)

In addition to the handover experiment above, cell load reporting support was implemented for both LTE and NR.

### ✅ LTE Cell Load
- **Files:**
  - `model/oran-report-lte-cell-load.cc`
  - `model/oran-report-lte-cell-load.h`

### ✅ NR Cell Load
- **Files:**
  - `model/oran-report-nr-cell-load.cc`
  - `model/oran-report-nr-cell-load.h`

***These reports enable the RIC/LM pipeline to track how busy each cell is and enable future load-aware decision policies.***

---

# ✅ Commit #1: NR Compatibility + ORAN Extensions

The first commit introduced a set of new components to make ORAN functionality compatible with NR and to expand ORAN capabilities beyond the baseline LTE examples.

---

## 1️⃣ NR UE Reporters (Cell Info + RSRP/RSRQ)

### ✅ NR UE Cell Info Reporter
Implemented `OranReporterNrUeCellInfo` to capture and report:

- **NR Cell ID**
- **RNTI**
- **Timestamp (time for the UE report)**

**Files:**
- `oran-reporter-nr-ue-cell-info.h`
- `model/oran-reporter-nr-ue-cell-info.cc`
- `model/oran-report-nr-ue-cell-info.cc`

---

### ✅ NR UE RSRP/RSRQ Reporter
Implemented `OranReporterNrUeRsrpRsrq` to report:

- **RSRP** (Reference Signal Received Power)
- **RSRQ** (Reference Signal Received Quality)

**Files:**
- `oran-reporter-nr-ue-rsrp-rsrq.h`
- `model/oran-reporter-nr-ue-rsrp-rsrq.cc`
- `model/oran-report-nr-ue-rsrp-rsrq.cc`

---

### ✅ Additional Reporting Models Added
**NR UE handover trigger report:**
- `model/oran-report-trigger-nr-ue-handover.cc`

**NR cell load report:**
- `model/oran-report-nr-cell-load.cc`

**LTE cell load report:**
- `model/oran-report-lte-cell-load.cc`

---

## 2️⃣ NR FFR Algorithm and SAPs

Introduced an **NR FFR (Fractional Frequency Reuse) Algorithm** for frequency reuse management.

### Core algorithm logic
- `nr-ffr-algorithm.cc`
- `nr-ffr-algorithm.h`

### Service Access Points (SAPs)
- `nr-ffr-sap.cc`
- `nr-ffr-sap.h`

---

## 3️⃣ ORAN CMM (Control/Management) Logic

Implemented ORAN **CMM handover coordination logic** to manage how HO commands are issued.

### ✅ Handover orchestration
- `model/oran-cmm-handover.cc`

### ✅ Single-command-per-node control
- `model/oran-cmm-single-command-per-node.cc`

These are used together with the NR→NR command + learning models to coordinate handover behavior per node.

---

## 4️⃣ Data Repository with SQLite Backend

Extended ORAN data handling with a repository abstraction and SQLite storage backend.

### ✅ Generic repository interface
- `model/oran-data-repository.h`

### ✅ SQLite-backed implementation
- `model/oran-data-repository-sqlite.cc`
- `model/oran-data-repository-sqlite.h`

---

## 5️⃣ NR E2 Node Terminators

Extended E2 node terminator support for NR endpoints.

### ✅ NR gNB E2 node terminator
- `model/oran-e2-node-terminator-nr-gnb.cc`
- `model/oran-e2-node-terminator-nr-gnb.h`

### ✅ NR UE E2 node terminator
- `model/oran-e2-node-terminator-nr-ue.cc`
- `model/oran-e2-node-terminator-nr-ue.h`

---

## 6️⃣ Build Integration and Robustness

All new components were integrated into the ORAN module build system and hardened with logging + error handling.

### ✅ Build integration updates
- `contrib/oran/CMakeLists.txt`

### Robustness improvements across
- NR UE reporters  
- NR load/cell reporting  
- NR FFR algorithm + SAPs  
- NR→NR learning models + commands  
- CMM handover logic  
- SQLite-backed data repository  
- NR E2 node terminators  

---
### Protocol
<img width="1079" height="792" alt="Screenshot from 2026-01-21 12-28-51" src="https://github.com/user-attachments/assets/14c011e4-fa52-45e4-b944-b3a68413e684" />


## 7️⃣ NR → NR Handover Optimization Models

Implemented multiple NR→NR handover decision models and the command interface to execute HO decisions.

## ✅ NR → NR Handover Decision Models & Command Interface

Implemented multiple **NR→NR handover decision models** and the **command interface** required to execute handover actions from the Near-RT RIC.

These components enable policy-driven and ML-driven handover control where:
- UEs and gNBs send measurement/state reports to the Near-RT RIC
- Logic Modules (LMs) make decisions periodically (based on the configured query interval)
- Commands are issued back to the source gNB to perform NR→NR handover

---

### *** Policy-Based Handover Mechanisms ***

#### ✅ RSRP-based NR→NR HO Logic Module (LM)
A lightweight **policy-based handover** model that selects the best serving gNB using **maximum RSRP**:

- Retrieves UE serving-cell info (CellId, RNTI)
- Reads UE measurement reports (RSRP/RSRQ to multiple gNBs)
- Chooses the **highest RSRP** gNB as the target cell
- Avoids repeated handover retries by falling back to the **second-best** RSRP candidate when needed

**File:**
- `model/oran-lm-nr-2-nr-rsrp-handover.cc`

---

#### ✅ NR→NR Handover Command Implementation
Implements the command object used by the RIC to instruct the **current serving gNB** to handover a specific UE to a **target NR cell**.

The command contains:
- Source/Target E2 Node ID
- UE RNTI
- Target Cell ID

**Files:**
- `model/oran-command-nr-2-nr-handover.cc`
- `model/oran-command-nr-2-nr-handover.h`
---

## 🔴 Present Work

- Looking for a way to add a **maximum cell load capacity threshold**
- Adding support to retrieve and track **handover decision failures** (e.g., HO command issued but not successfully completed)

  ## ⚡ LTE vs NR Runtime Comparison (Why NR is Slower)

| Feature / Component | LTE Scenario | NR Scenario | Why NR is Slower |
|---------------------|-------------|-------------|------------------|
| Channel / Propagation Model | `Cost231PropagationLossModel` (simple pathloss) | `ThreeGppChannelModel (UMa)` (realistic 3GPP) | 3GPP model computes LOS/NLOS, shadowing, fading behavior → higher CPU cost |
| PHY Processing | Lightweight LTE PHY | Heavy NR PHY | NR PHY has more detailed signal processing & scheduling operations |
| Fading | Usually minimal / simplified | ✅ Enabled (`INIT_FADING`) | Fast fading calculations increase per-link computation |
| Beamforming | ❌ Not used | ✅ Enabled (IdealBeamformingHelper + QuasiOmni) | Beamforming adds gain calculations + channel matrix handling |
| Carrier/BWP Setup | Single carrier (simpler) | ✅ Dual-band FDD + 2 BWPs (DL + UL) | More BWPs = more PHY instances + more scheduling + mapping operations |
| Scheduling Complexity | LTE RR scheduler (`RrFfMacScheduler`) | NR OFDMA scheduler (`NrMacSchedulerOfdmaRR`) | NR OFDMA scheduling is heavier due to resource-grid level allocations |
| Mobility Speed | UE speed: **1–2.5 m/s** | UAV speed: **20–30 m/s** | Higher speed changes channel faster → more frequent recalculations |
| Channel Update Period | Not explicitly heavy | UpdatePeriod configured (`ThreeGppChannelModel::UpdatePeriod`) | Even with large update period, NR calculations per update are costly |
| QoS / FlowMonitor Prints | Moderate | High (frequent `std::cout`) | Console printing inside loops slows execution significantly |
| Simulation Outcome | Faster runtime | Slower runtime | NR is more realistic and detailed → expensive per timestep |

✅ **Summary:** NR simulation runs slower mainly due to **3GPP channel model + fading + beamforming + multi-BWP FDD + OFDMA scheduling**, even if the number of nodes is lower than LTE.
  

This repository currently extends the ns-3 **NR (5G-LENA)** + **ns-O-RAN** examples into a **large-scale UAV mobility experiment** where a Near-RT RIC continuously collects measurements and issues **NR→NR handover commands** using a lightweight **policy-based logic module**.

### ✅ Example 02: `oran-nr-uav-rsrp-handover-example.cc`

**What the scenario does**
- Deploys multiple **fixed NR gNB macro cells** (e.g., 5) and many **UAV UEs** (e.g., 50–75).
- UAVs move using `RandomDirection2dMobilityModel` inside a large 2D bounded area with randomized altitude.
- Downlink UDP traffic is generated from a remote host through **NR EPC** (`NrPointToPointEpcHelper`).
- Uses **3GPP UMa channel model** (optionally with fast fading) and **Ideal Beamforming** (Quasi-Omni direct path).
- Uses a **single-carrier TDD setup** (1 band, 1 BWP) with a configurable DL/UL pattern.
- Enables X2 between gNBs to support inter-cell handover.
- Collects **QoS KPIs** using FlowMonitor and writes time-series traces:
  - delay, jitter, throughput (Mbps), packet delivery ratio (PDR), packet loss ratio (PLR)
- Logs mobility + HO events:
  - UAV position trace (`position-trace.tr`)
  - Handover success events (`NrGnbRrc/HandoverEndOk` → `handover-trace.tr`)
  - RSRP measurements (`rsrp-trace.tr`)
- Optionally generates a **NR Radio Environment Map (REM)** snapshot.

---

## Current ORAN NR Examples
The ORAN module currently builds the following NR examples. These are the
renamed `./ns3 run` targets in `contrib/oran/examples/CMakeLists.txt`.

### NR RSRP handover
```shell
./ns3 run "oran-nr-tn-rsrp-handover-example"
./ns3 run "oran-nr-ntn-rsrp-handover-example"
./ns3 run "oran-nr-uav-rsrp-handover-example"
./ns3 run "oran-nr-uav-load-aware-rsrp-handover-example"
```

### NR UAV and ground UE load-aware handover
```shell
./ns3 run "oran-nr-uav-ground-traffic-handover-example"
./ns3 run "oran-nr-uav-ground-qos-handover-example"
./ns3 run "oran-nr-uav-ground-fronthaul-handover-example"
./ns3 run "oran-nr-tn-ntn-load-aware-handover-example"
./ns3 run "oran-nr-tn-ntn-xr-handover-example"
./ns3 run "oran-nr-tn-ntn-uav-mobility-handover-example"
```

### NR TN/NTN and secrecy-aware handover
```shell
./ns3 run "oran-nr-tn-ntn-rsrp-handover-example"
./ns3 run "oran-nr-tn-ntn-hybrid-xr-example"
./ns3 run "oran-nr-tn-ntn-baseline-handover-example"
./ns3 run "oran-nr-tn-ntn-secrecy-handover-example"
./ns3 run "oran-nr-tn-ntn-secrecy-onnx-handover-example"
```

### NR UAV satellite backhaul
```shell
./ns3 run "oran-nr-uav-satellite-backhaul-example"
./ns3 run "oran-nr-tn-ntn-uav-satellite-handover-example"
./ns3 run "oran-nr-tn-ntn-uav-satellite-load-handover-example"
./ns3 run "oran-nr-tn-uav-satellite-handover-example"
./ns3 run "oran-nr-hybrid-tn-ntn-uav-satellite-handover-example"
```

### NR TN/NTN UAV satellite handover with O-RAN LM logs
To write `NS_LOG_INFO` messages such as `LOAD`, `TTT_WAIT`, `LM HO`, and
`LM HO_FAIL_LOW_RSRP` to `ns3-oran-lm.log`, configure the optimized build with
runtime logging enabled first:

```shell
./ns3 configure --build-profile=optimized --enable-logs --enable-examples --enable-tests
./ns3 build
```

Then run the example with O-RAN LM logging enabled:

```shell
./ns3 run "oran-nr-tn-ntn-uav-satellite-handover-example --sim-time=40 --enable-flow-monitor=1 --enable-rsrp-trace=0 --enable-position-trace=1 --position-trace-interval=5 --enable-handover-trace=1 --enable-handover-failure-trace=1 --enable-sat-backhaul-monitor=0 --enable-decision-csv=1 --enable-oran-info-log=1 --enable-nr-helper-info-log=0 --enable-setup-prints=0 --enable-progress=1 --progress-interval=5 --mobility-update-ms=1000 --e2-send-interval=5 --lm-query-interval=5 --ground-attach-delay=2 --channel-update-ms=1000 --channel-condition-update-ms=1000 --enable-fh-control=0 --use-fixed-mcs=1 --fixed-mcs=4 --enable-srs-in-ul-slots=0 --enable-srs-in-f-slots=0"
```

Use `--enable-setup-prints=1` when initial attach messages are also needed.
The SRS switches are disabled in this stress-run command to avoid SRS reception
overlapping with uplink data reception when using ideal beamforming.

### UAV underserved-heatmap predictor training
After running the TN/NTN UAV satellite handover example, the Python script
`train_uav_trajectory_final.py` can train models that predict where underserved
UE hotspots will appear next. It reads:

- `ues1-position-trace.tr`
- `ues2-position-trace.tr`
- `ml-ho-dataset.csv`
- `ns3-oran-lm.log`

The script converts UE positions into local meter coordinates, aligns O-RAN/ML
event timestamps with the nearest position trace timestamp, builds underserved
UE heatmaps, and trains persistence, MLP, CNN, and GRU predictors using a
chronological train/validation/test split.

The split is chronological, so future heatmaps are not mixed into past
training data:

```text
60% training   -> the model studies past examples and updates its weights
15% validation -> checks which epoch/model version is best
25% testing    -> final exam used only for reporting final performance
```

Validation is needed because a model can keep improving on the training data
by memorizing it, while getting worse on unseen future data. After each epoch,
the script measures validation loss and keeps the model checkpoint with the
best validation result. The test split is kept untouched until the end, so the
reported metrics are not used to choose the model.

Example:

```shell
python3 train_uav_trajectory_final.py \
  --ues1 results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ues1-position-trace.tr \
  --ues2 results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ues2-position-trace.tr \
  --ml-csv results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ml-ho-dataset.csv \
  --log results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ns3-oran-lm.log \
  --outdir results/nr/tn-ntn/ml_uav_final \
  --train-ratio 0.60 --val-ratio 0.15 --event-time-tolerance 2.5
```

Main outputs are written under `--outdir`, including
`predictor_comparison.csv`, `training_history_all_models.csv`,
`underserved_heatmap_summary.csv`, `predictor_config.json`, comparison plots,
and saved model state dictionaries. Use `--export-onnx` to export the selected
neural predictor to ONNX when ONNX dependencies are installed.

To create the GRU ONNX model used by the predictive UAV load-handover example,
run:

```shell
python3 train_uav_trajectory_final.py --rsrp-thresh -120 --export-onnx --export-model gru
```

This writes:

```text
results/nr/tn-ntn/ml_uav_final/uav_underserved_heatmap_gru_ir8.onnx
```

Then run the predictive UAV load-handover example:

```shell
./ns3 run "oran-nr-tn-ntn-uav-satellite-load-handover-example --sim-time=122 --enable-predictive-uav=1 --uav-predictor-onnx=results/nr/tn-ntn/ml_uav_final/uav_underserved_heatmap_gru_ir8.onnx --uav-heat-norm-max=3 --uav-underserved-rsrp-thresh=-120 --enable-flow-monitor=1 --enable-position-trace=1 --position-trace-interval=5 --enable-sat-backhaul-monitor=0 --enable-oran-info-log=1 --enable-nr-helper-info-log=1 --ground-attach-delay=2 --mobility-update-ms=1000 --e2-send-interval=5 --lm-query-interval=5"
```

`--uav-heat-norm-max` must match `normalization_max_count` in
`predictor_config.json`, and `--uav-underserved-rsrp-thresh` must match the
RSRP threshold used during training.

---

## 🧠 ORAN Near-RT RIC Closed-Loop Control (Current Implementation)

When ORAN is enabled (`--use-oran=1`), the following control loop runs:

1. **UAV UEs and gNBs act as E2 nodes** using NR E2 terminators.
2. UEs periodically report:
   - location,
   - serving cell info (CellId/RNTI),
   - RSRP/RSRQ measurements (to multiple cells),
   - application Tx/Rx loss stats (from UDP traces).
3. gNBs periodically report:
   - location,
   - cell-load indicators derived from NR MAC scheduling callbacks (DL scheduling).
4. A Near-RT RIC queries the repository at `LmQueryInterval` and runs the selected **Logic Module (LM)**.
5. The LM issues an **NR→NR handover command** (source gNB + UE RNTI + target cellId).
6. A Conflict Mitigation Module (CMM) applies command scheduling policies.

Repository backend: **SQLite** (`OranDataRepositorySqlite`) for reproducible logging of reports and commands.

---

## ✅ Load-Aware RSRP Handover Logic Module (RSRP-only)

### LM: `OranLmNr2NrRsrpHandoverWithCellLoad`

This LM implements **RSRP-based HO target selection** with:
- **hysteresis margin** to reduce ping-pong,
- **rate limiting / cooldown** between HO commands,
- **pending-HO timeout handling**,
- optional **cell capacity gating** (`MaxUesPerCell`),
- a **minimum acceptable target RSRP** guard with backoff on failure.

### Key Attributes (configurable from the simulation script)
- `HysteresisDb` (default `2.0` dB): neighbor must exceed `servingRsrp + hysteresis` to be considered.
- `Warmup` (default `2s`): LM does not issue HOs before warmup ends.
- `MinHoInterval` (default `2s`): minimum time between HO commands per UE.
- `HoAttemptTimeout` (default `2s`): pending HO is considered stale after this duration.
- `MaxUesPerCell` (default `0`):  
  - **`0` disables the capacity cap** (no load restriction).  
  - **`>0` enables a hard cap**: candidate target cells at/above capacity are rejected.
- `TryNextBest` (default `true`): if best cell is full, try the next-best RSRP candidate.
- `MinAcceptableRsrpDbm` (default `-120 dBm`): rejects HO if the target candidate is too weak.
- `LowRsrpRecheck` (default `2s`): after a low-RSRP rejection, the UE is blocked from re-evaluation for this time.

---

## 🔁 Handover Decision Logic (Step-by-Step)

Inside `GetHandoverCommands(...)`, the LM follows this per-tick logic:

### 1) Build a live cell-load map (cellId → UE count)
- The LM counts UEs per cell using *serving-cell* PHY measurements (BWP/CC 0), with a fallback to UE cell info if needed.
- It also **reserves capacity for pending HO targets**, so multiple HOs in the same tick don’t exceed the cap.

### 2) Apply safety gates per UE
Before evaluating candidates, the LM skips UEs when:
- simulation time is still in `Warmup`,
- the UE is in **low-RSRP backoff** (`lowRsrpBlockUntil`),
- a previous HO is still pending and not timed out (`pendingHoTarget` + `HoAttemptTimeout`),
- the UE is still in HO cooldown (`MinHoInterval`).

### 3) Build candidate list from measurements
- Reads RSRP measurements for the UE.
- Extracts:
  - the **serving cell** (CellId/RNTI + serving RSRP),
  - the **best RSRP per candidate cell** (max across measurements for that cell).
- Sorts candidates by **descending RSRP**.

### 4) Candidate filtering + target selection
For each candidate (best-to-worst):
- **Hysteresis gate:**  
  candidate must satisfy:  
  `candRsrp > servingRsrp + HysteresisDb`
- **Min acceptable RSRP gate:**  
  if `candRsrp < MinAcceptableRsrpDbm` → reject HO and apply UE backoff for `LowRsrpRecheck`.
- **Capacity gate (only if enabled):**  
  if `MaxUesPerCell > 0` and `cellLoad >= MaxUesPerCell` → reject that target.  
  - if `TryNextBest=true`, continue searching other candidates  
  - else stop and keep current serving cell
- First candidate that passes all gates becomes the HO target.

### 5) Emit HO command (only if a valid target exists)
If a target cell is selected (different from serving cell):
- Creates `OranCommandNr2NrHandover`:
  - `TargetE2NodeId` = serving gNB node ID (**source** gNB),
  - `TargetRnti` = UE RNTI,
  - `TargetCellId` = chosen target cell ID.
- Logs the command to the repository and prints a summary log line.
- Updates internal state:
  - `lastHoCmdTime[ue] = now`
  - `pendingHoTarget[ue] = chosenCell`
  - increments `cellUeCount[chosenCell]` (reserve slot immediately)

### 6) Behavior when `MaxUesPerCell = 0`
When `MaxUesPerCell` is **0**:
- **Capacity checks are fully bypassed** (`m_maxUesPerCell > 0` condition is false).
- Logs print `cap=disabled`.
- HO decisions become purely **RSRP + hysteresis + min-RSRP + timing gates**.

---

## 📌 Outputs Useful for Debugging/Validation
- Per-tick load snapshot logs show:
  - `cell X load=Y (cap=disabled)` OR `cell X load=Y/Z`
- HO logs:
  - `LM HO UE=... servingCell->targetCell servingRsrp=... targetRsrp=...`
- Low-RSRP rejection logs:
  - `LM HO_FAIL_LOW_RSRP ... recheckAfter=...`

---

## Next Steps (Planned)
- Extend load awareness beyond a static hard cap into:
  - dynamic policies (e.g., load balancing objectives),
  - multi-objective HO
  - future ML/game-theoretic handover optimizers.


 ## 🔴 ***Below will be implemented in the Future***
  
  ### *** Optimize handover based on Game theory ***
  
  ### *** ML-Based Handover Mechanisms ***

### Torch-based NR→NR HO Learning Model
- `model/oran-lm-nr-2-nr-torch-handover.cc`
- `model/oran-lm-nr-2-nr-torch-handover.h`

### ONNX-based NR→NR HO Learning Model
- `model/oran-lm-nr-2-nr-onnx-handover.cc`
- `model/oran-lm-nr-2-nr-onnx-handover.h`

---

## ✅ Notes / Future Extensions Summary
This framework can be extended further to support:
- Load-aware HO steering (RSRP + cell load) : Keep a cell load capacity and design the model
- Multi-objective optimization (QoS + PHY + load) - Game theory
- RIC-driven ML-based decision loops using the SQLite repository

---

### Proposed dynamic movement of the UAV
<img width="1112" height="734" alt="Screenshot from 2026-01-21 11-44-38" src="https://github.com/user-attachments/assets/f3cca852-05b9-48d1-b783-8ab03dfc4933" />

Currently using ns3::RandomDirection2dMobilityModel for testing purposes with static height.

### Architecture
<img width="1121" height="804" alt="Screenshot from 2026-01-21 12-27-03" src="https://github.com/user-attachments/assets/1a95fdbb-1238-4624-a13a-7863cca40451" />
