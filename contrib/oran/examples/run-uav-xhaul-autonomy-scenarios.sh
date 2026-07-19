#!/usr/bin/env bash
set -euo pipefail

# Run this script from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
#
# Purpose:
#   Compare terrestrial, UAV-assisted, and satellite-assisted service. The
#   TN-only scenario is a clean semi-urban terrestrial reference with no
#   artificial disruption. The first TN+UAV scenario adds aerial cells under
#   healthy terrestrial donor backhaul. The second TN+UAV scenario uses the same natural
#   mission donor-backhaul degradation as the satellite case, but without satellite
#   fallback. The satellite-assisted scenario uses a natural mission period
#   starting at 15 s where UAVs move toward underserved UEs.
#   UAV-to-TN donor-backhaul degradation is driven by donor distance, stricter RSRP
#   thresholds, and stochastic channel variation:
#     1. TN only under healthy terrestrial infrastructure
#     2. TN + UAV with extra aerial access capacity and healthy TN donor backhaul
#     3. TN + UAV with natural donor-backhaul degradation and no satellite fallback
#     4. TN + UAV + satellite with terrestrial donor-backhaul degradation and a separate
#        simulated onboard UAV Near-RT RIC/autonomy xApp fallback
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
#   xhaul-autonomy-trace.csv    UAV donor-backhaul RSRP, xHaul-health state, active UAV RIC,
#                               control path, and UAV mode.
#   handover-trace.tr           Successful handover events.
#   handover-failure-trace.tr   Failed handover events.
#   tn-infrastructure-trace.csv TN degradation state and TN gNB TxPower.
#   ml-ho-dataset.csv           Candidate/decision records from the RIC logic module.
#   ns3-oran-lm.log             Verbose O-RAN logic-module INFO log.

COMMON_ARGS="--sim-time=40 \
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
  --xhaul-max-donor-distance-m=10000 \
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

NATURAL_MISSION_ARGS="--tn-degradation-start=-1 \
  --tn-degradation-stop=-1 \
  --tn-degradation-penalty-db=0 \
  --uav-control-start=15 \
  --uav-control-period=2 \
  --uav-underserved-rsrp-thresh-dbm=-105 \
  --uav-initial-area-half-w-m=3000 \
  --uav-initial-area-half-h-m=1500 \
  --uav-area-half-w-m=9000 \
  --uav-area-half-h-m=4500 \
  --uav-mission-target-scale=2.2 \
  --uav-speed-mps=220 \
  --xhaul-degradation-start=-1 \
  --xhaul-degradation-stop=-1 \
  --xhaul-degradation-penalty-db=0 \
  --xhaul-healthy-rsrp-dbm=-72 \
  --xhaul-degraded-rsrp-dbm=-82 \
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
# coverage. The UAV-to-ground TN donor link is monitored and should stay within
# the 10 km terrestrial donor-backhaul range. Satellite backhaul is disabled.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=healthy-xhaul ${COMMON_ARGS} ${UAV_ARGS}"

# Scenario 3: UE + TN + UAV with natural mission-period donor-backhaul stress, but no
# satellite fallback. This is the fair no-satellite degraded baseline. The UAVs
# follow the same mission behavior and donor-backhaul classification thresholds as the
# satellite case. When donor backhaul is degraded/unreachable, normal UE handover to the
# affected UAV cell is blocked because no satellite/onboard fallback is enabled.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --run-label=natural-xhaul-no-sat ${COMMON_ARGS} ${UAV_ARGS} ${NATURAL_MISSION_ARGS}"

# Scenario 4: UE + TN + UAV + satellite with a natural mission-period stress.
# Same TN+UAV deployment, but satellite backhaul monitoring is enabled. From
# 15 s onward, UAVs move toward underserved UE clusters with a moderated mission
# speed/area so UAV-to-UE access remains useful. The UAV-to-TN donor backhaul is
# not weakened with a hand-set dB penalty; donor distance, urban
# shadowing/fading variation, and donor-backhaul RSRP thresholds decide whether onboard
# UAV RIC control and satellite fallback are needed.
./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav-satellite --run-label=natural-xhaul-sat ${COMMON_ARGS} ${UAV_ARGS} --enable-sat-backhaul-monitor=1 --enable-onboard-uav-ric=1 ${NATURAL_MISSION_ARGS}"
