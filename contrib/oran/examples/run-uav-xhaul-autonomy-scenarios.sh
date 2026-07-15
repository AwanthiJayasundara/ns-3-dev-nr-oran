#!/usr/bin/env bash
set -euo pipefail

# Run this script from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
#
# Purpose:
#   Compare the same 100-UE experiment under three deployment modes:
#     1. TN only
#     2. TN + UAV
#     3. TN + UAV + satellite
#
# Common experiment size:
#   --sim-time=40          Keep the full 40 s experiment duration.
#   --num-uess1=50        50 monitored/mobile UEs with QoS and handover traces.
#   --num-ground-ues=50   50 additional background/load UEs.
#   --num-tn-gnbs=4       Four terrestrial gNBs over the study area.
#   --num-ntn-gnbs=2      Two UAV cell nodes when UAVs are enabled.
#   --rlc-mode=am         Use acknowledged-mode RLC for robust handover/load runs.
#
# Common outputs:
#   qos-vs-time.txt             Delay, jitter, throughput, and PDR.
#   xhaul-autonomy-trace.csv    UAV xHaul RSRP, xHaul state, and UAV mode.
#   handover-trace.tr           Successful handover events.
#   handover-failure-trace.tr   Failed handover events.
#   ml-ho-dataset.csv           Candidate/decision records from the RIC logic module.

COMMON_ARGS="--sim-time=40 \
  --num-uess1=50 \
  --num-ground-ues=50 \
  --num-tn-gnbs=4 \
  --num-ntn-gnbs=2 \
  --rlc-mode=am \
  --enable-flow-monitor=1 \
  --enable-position-trace=1 \
  --enable-handover-trace=1 \
  --enable-handover-failure-trace=1 \
  --enable-decision-csv=1 \
  --enable-progress=1"

# Build only the new example target. Configure first if this is a fresh build tree:
#   ./ns3 configure --build-profile=optimized --enable-examples
./ns3 build oran-nr-uav-xhaul-autonomy-example

# Scenario 1: UE + TN only.
# This is the terrestrial baseline. No UAV cells are installed, so the
# xhaul-autonomy-trace.csv file will contain only the header.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only ${COMMON_ARGS}"

# Scenario 2: UE + TN + UAV.
# UAVs are active cell nodes. The UAV-to-ground TN donor link is monitored as
# xHaul using estimated RSRP/SINR health. Satellite backhaul is disabled.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav ${COMMON_ARGS}"

# Scenario 3: UE + TN + UAV + satellite.
# Same TN+UAV deployment, but satellite backhaul monitoring is enabled so that
# UAV service continuity can be compared under terrestrial/xHaul degradation.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite ${COMMON_ARGS} --enable-sat-backhaul-monitor=1"

# Optional stress run: TN + UAV with degraded terrestrial xHaul.
# Uncomment this command to force an xHaul RSRP penalty from 15 s to 30 s.
# ./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav ${COMMON_ARGS} --xhaul-degradation-start=15 --xhaul-degradation-stop=30 --xhaul-degradation-penalty-db=35"
