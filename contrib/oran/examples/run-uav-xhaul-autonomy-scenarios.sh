#!/usr/bin/env bash
set -euo pipefail

# Run this script from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
#
# Purpose:
#   Compare terrestrial, UAV-assisted, and satellite-assisted service. The
#   TN-only scenario is a clean semi-urban terrestrial reference with no
#   artificial disruption. The UAV scenarios use a 15-30 s terrestrial
#   infrastructure degradation window and a matching UAV xHaul degradation
#   window:
#     1. TN only under healthy terrestrial infrastructure
#     2. TN + UAV with degraded terrestrial infrastructure and degraded xHaul
#     3. TN + UAV + satellite with the same degradation, plus satellite fallback
#
# Scenario 1 experiment size:
#   --num-uess1=20        20 monitored/mobile UEs available from the start.
#   --num-ground-ues=50   50 background/load UEs attached at 5 s.
#   --num-tn-gnbs=4       Four terrestrial gNBs over the study area.
#   --max-ues-tn=20       Per-cell TN capacity. With 4 cells, total TN capacity
#                          is 80 UEs, enough for 20 + 50 UEs.
#
# Common outputs:
#   qos-vs-time.txt             Delay, jitter, throughput, and PDR.
#   xhaul-autonomy-trace.csv    UAV xHaul RSRP, xHaul state, and UAV mode.
#   handover-trace.tr           Successful handover events.
#   handover-failure-trace.tr   Failed handover events.
#   tn-infrastructure-trace.csv TN degradation state and TN gNB TxPower.
#   ml-ho-dataset.csv           Candidate/decision records from the RIC logic module.
#   ns3-oran-lm.log             Verbose O-RAN logic-module INFO log.

COMMON_ARGS="--sim-time=40 \
  --num-tn-gnbs=4 \
  --rlc-mode=am \
  --monitored-traffic=udp \
  --monitored-dl-rate-mbps=1.0 \
  --monitored-ul-rate-mbps=0.25 \
  --monitored-packet-size=1000 \
  --tn-tx-power-dbm=83 \
  --uav-tx-power-dbm=78 \
  --ue-tx-power-dbm=43 \
  --init-min-rsrp=-160 \
  --enable-flow-monitor=1 \
  --enable-rsrp-trace=1 \
  --enable-position-trace=1 \
  --enable-handover-trace=1 \
  --enable-handover-failure-trace=1 \
  --enable-decision-csv=1 \
  --enable-oran-info-log=1 \
  --enable-progress=1"

TN_ONLY_ARGS="--num-uess1=20 \
  --num-ground-ues=50 \
  --ground-attach-delay=5 \
  --max-ues-tn=20 \
  --num-ntn-gnbs=0"

UAV_ARGS="--num-uess1=50 \
  --num-ground-ues=50 \
  --ground-attach-delay=6 \
  --max-ues-tn=20 \
  --max-ues-ntn=10 \
  --num-ntn-gnbs=2"

DEGRADED_TN_ARGS="--tn-degradation-start=15 \
  --tn-degradation-stop=30 \
  --tn-degradation-penalty-db=15"

DEGRADED_XHAUL_ARGS="--xhaul-degradation-start=15 \
  --xhaul-degradation-stop=30 \
  --xhaul-degradation-penalty-db=35"

# Build only the new example target. Configure first if this is a fresh build tree:
#   ./ns3 configure --build-profile=optimized --enable-examples
./ns3 build oran-nr-uav-xhaul-autonomy-example

# Scenario 1: UE + TN only with healthy terrestrial infrastructure.
# This is the clean semi-urban TN reference. No UAV cells are installed,
# no satellite monitor is enabled, and no artificial TN/xHaul degradation is
# applied. The xhaul-autonomy-trace.csv file will contain only the header.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only ${COMMON_ARGS} ${TN_ONLY_ARGS}"

# Scenario 2: UE + TN + UAV with degraded TN infrastructure and degraded xHaul.
# UAVs are active cell nodes. The UAV-to-ground TN donor link is monitored as
# xHaul using estimated RSRP/SINR health. Satellite backhaul is disabled.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav ${COMMON_ARGS} ${UAV_ARGS} ${DEGRADED_TN_ARGS} ${DEGRADED_XHAUL_ARGS}"

# Scenario 3: UE + TN + UAV + satellite with the same degradation.
# Same TN+UAV deployment, but satellite backhaul monitoring is enabled so that
# UAV service continuity can be compared under terrestrial/xHaul degradation.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite ${COMMON_ARGS} ${UAV_ARGS} --enable-sat-backhaul-monitor=1 ${DEGRADED_TN_ARGS} ${DEGRADED_XHAUL_ARGS}"
