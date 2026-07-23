#!/usr/bin/env bash
set -euo pipefail

# Run this script from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
#
# Purpose:
#   Compare terrestrial, UAV-assisted, and satellite-assisted service. The
#   TN-only scenario is a clean semi-urban terrestrial reference with no
#   artificial disruption. The TN+UAV scenario adds aerial cells under healthy
#   terrestrial donor backhaul. The satellite-assisted scenario uses a natural
#   mission period starting at 30 s where UAVs move toward underserved UEs.
#   In the satellite case, the TN donor/gateway path is unavailable from 30-75 s,
#   representing a mission segment outside donor coverage or with blocked TN
#   donor reachability. No hand-set RSRP penalty is used.
#     1. TN only under healthy terrestrial infrastructure
#     2. TN + UAV with extra aerial access capacity and healthy TN donor backhaul
#     3. TN + UAV with the same donor-backhaul outage but no satellite fallback
#     4. TN + UAV + satellite with terrestrial donor-backhaul outage and a
#        Near-RT RIC UAV TN/satellite switching xApp fallback
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
#   xhaul-autonomy-trace.csv    UAV donor-backhaul RSRP, xHaul-health state,
#                               switching-xApp state, backhaul path, and UAV mode.
#   handover-trace.tr           Successful handover events.
#   handover-failure-trace.tr   Failed handover events.
#   tn-infrastructure-trace.csv TN degradation state and TN gNB TxPower.
#   ml-ho-dataset.csv           Candidate/decision records from the RIC logic module.
#   ns3-oran-lm.log             Verbose O-RAN logic-module INFO log.
#
# Channel model convention used by the example:
#   TN UE access:                 3GPP UMa
#   UAV/NTN UE access:            3GPP NTN-Urban
#   Satellite fallback/backhaul:  3GPP NTN-Suburban

COMMON_ARGS="--sim-time=120 \
  --num-tn-gnbs=4 \
  --rlc-mode=am \
  --monitored-traffic=udp \
  --monitored-dl-rate-mbps=0.2 \
  --monitored-ul-rate-mbps=0.05 \
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
  --ground-attach-delay=5 \
  --max-ues-tn=20 \
  --max-ues-ntn=10 \
  --num-ntn-gnbs=3"

NATURAL_MISSION_ARGS="--uav-control-start=30 \
  --uav-control-period=2 \
  --uav-underserved-rsrp-thresh-dbm=-110 \
  --uav-initial-area-half-w-m=3000 \
  --uav-initial-area-half-h-m=1500 \
  --uav-area-half-w-m=16000 \
  --uav-area-half-h-m=8000 \
  --uav-mission-target-scale=4 \
  --uav-speed-mps=25 \
  --xhaul-donor-unavailable-start=30 \
  --xhaul-donor-unavailable-stop=75 \
  --xhaul-healthy-rsrp-dbm=-110 \
  --enable-xhaul-channel-variation=1 \
  --xhaul-shadowing-stddev-db=6 \
  --xhaul-fading-stddev-db=4 \
  --channel-update-ms=100 \
  --channel-condition-update-ms=200"

# Build only the new example target. Configure first if this is a fresh build tree:
#   ./ns3 configure --build-profile=optimized --enable-examples
./ns3 build oran-nr-uav-xhaul-autonomy-example

# Scenario 1: UE + TN only with healthy terrestrial infrastructure.
# This is the clean semi-urban TN reference. No UAV cells are installed,
# no satellite monitor is enabled, and no artificial TN/donor-backhaul degradation is
# applied. The xhaul-autonomy-trace.csv file will contain only the header.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-only --run-label=clean ${COMMON_ARGS} ${TN_ONLY_ARGS}"

# Scenario 2: UE + TN + UAV with healthy terrestrial donor backhaul.
# UAVs are active cell nodes. The TN layer has 80 UE capacity while 100 UEs are
# present after 5 s, so UAV cells are expected to help with access capacity and
# coverage. The UAV-to-ground TN donor link is monitored using the best donor
# RSRP; the final comparison disables the hard donor-distance cutoff. Satellite
# backhaul is disabled.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=healthy-xhaul ${COMMON_ARGS} ${UAV_ARGS}"

# Scenario 3: UE + TN + UAV with donor-backhaul stress but no satellite fallback.
# This is the fair no-satellite baseline. When the TN donor/gateway path is
# unavailable, the Near-RT RIC switching xApp removes the UAV route to the core
# and blocks normal handover to the UAV. QoS should drop for UAV-served users
# during this interval.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=donor-unavailable-no-sat ${COMMON_ARGS} ${UAV_ARGS} ${NATURAL_MISSION_ARGS}"

# Scenario 4: UE + TN + UAV + satellite with a natural mission-period stress.
# Same TN+UAV deployment, but satellite backhaul monitoring is enabled. From
# 30 s onward, UAVs move toward underserved UE clusters with a realistic speed.
# During 30-75 s, the TN donor/gateway path is unavailable, representing that the UAV
# is outside the usable donor-link coverage region or the terrestrial donor path
# is blocked. No artificial RSRP penalty is applied; the trace records this as
# DonorUnavailableActive=1 and XhaulState=UNREACHABLE. If satellite SNR is
# healthy, the Near-RT RIC switching xApp switches the core path to
# UAV->SAT->GW->core.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --run-label=donor-unavailable-sat ${COMMON_ARGS} ${UAV_ARGS} --sat-backhaul-scenario=NTN-Suburban --enable-sat-backhaul-monitor=1 --enable-uav-switching-xapp=1 ${NATURAL_MISSION_ARGS}"
