# ORAN Handover Optimization + Cell Load Experiment (ns-3 NR/ORAN)

This repository extends the original ns-3 ORAN handover examples into a **large-scale-optimized, performance-oriented multi-cell experiment** with additional **NR-compatible reporters, learning models, CMM orchestration logic, and SQLite-based data storage**.

---

## 📌 Example: `oran-lte-2-lte-rsrp-ue-handover-simulation.cc`

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

These reports enable the RIC/LM pipeline to track how busy each cell is and enable future load-aware decision policies.

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

## 3️⃣ NR → NR Handover Learning Models and Command

Implemented multiple NR→NR handover decision models and the command interface to execute HO decisions.

### ✅ Torch-based NR→NR HO Learning Model
- `model/oran-lm-nr-2-nr-torch-handover.cc`
- `model/oran-lm-nr-2-nr-torch-handover.h`

### ✅ ONNX-based NR→NR HO Learning Model
- `model/oran-lm-nr-2-nr-onnx-handover.cc`
- `model/oran-lm-nr-2-nr-onnx-handover.h`

### ✅ RSRP-based NR→NR HO Learning Model
- `model/oran-lm-nr-2-nr-rsrp-handover.cc`

### ✅ NR→NR HO Command Implementation
- `model/oran-command-nr-2-nr-handover.cc`
- `model/oran-command-nr-2-nr-handover.h`

---

## 4️⃣ ORAN CMM (Control/Management) Logic

Implemented ORAN **CMM handover coordination logic** to manage how HO commands are issued.

### ✅ Handover orchestration
- `model/oran-cmm-handover.cc`

### ✅ Single-command-per-node control
- `model/oran-cmm-single-command-per-node.cc`

These are used together with the NR→NR command + learning models to coordinate handover behavior per node.

---

## 5️⃣ Data Repository with SQLite Backend

Extended ORAN data handling with a repository abstraction and SQLite storage backend.

### Generic repository interface
- `model/oran-data-repository.h`

### SQLite-backed implementation
- `model/oran-data-repository-sqlite.cc`
- `model/oran-data-repository-sqlite.h`

---

## 6️⃣ NR E2 Node Terminators

Extended E2 node terminator support for NR endpoints.

### ✅ NR gNB E2 node terminator
- `model/oran-e2-node-terminator-nr-gnb.cc`
- `model/oran-e2-node-terminator-nr-gnb.h`

### ✅ NR UE E2 node terminator
- `model/oran-e2-node-terminator-nr-ue.cc`
- `model/oran-e2-node-terminator-nr-ue.h`

---

## 7️⃣ Build Integration and Robustness

All new components were integrated into the ORAN module build system and hardened with logging + error handling.

### Build integration updates
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

## ✅ Notes / Future Extensions
This framework can be extended further to support:
- Load-aware HO steering (RSRP + cell load)
- Multi-objective optimization (QoS + PHY + load)
- RIC-driven ML-based decision loops using the SQLite repository

---
