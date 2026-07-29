#!/usr/bin/env bash
set -euo pipefail

# Run this script from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
#
# To repeat the four main scenarios over several random seeds:
#   SEEDS="1 2 3 4" bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
#
# Purpose:
#   Compare terrestrial, UAV-assisted, and satellite-assisted service. The
#   TN-only scenario is a clean semi-urban terrestrial reference with no
#   artificial disruption. The TN+UAV scenario adds aerial cells under healthy
#   terrestrial donor backhaul. The distance-loss scenarios use a mission period
#   starting at 5 s where UAVs move toward underserved UEs. The UAV-to-TN donor
#   RSRP can then fall because of distance/path loss and channel variation. No
#   forced donor-unavailable time window or hand-set RSRP penalty is used.
#     1. TN only under healthy terrestrial infrastructure
#     2. TN + UAV with extra aerial access capacity and healthy TN donor backhaul
#     3. TN + UAV with distance-based donor-backhaul loss and no satellite fallback
#     4. TN + UAV + satellite with distance-based donor-backhaul loss and a
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

SEEDS="${SEEDS:-}"
# Keep HARQ disabled by default for this large mobility/satellite scenario.
# It can be enabled for focused debugging with ENABLE_HARQ_RETX=1, but it has
# caused allocator crashes in long mixed TN/UAV/satellite runs.
ENABLE_HARQ_RETX="${ENABLE_HARQ_RETX:-0}"

COMMON_ARGS="--sim-time=120 \
  --num-tn-gnbs=4 \
  --rlc-mode=am \
  --monitored-traffic=udp \
  --monitored-dl-rate-mbps=0.2 \
  --monitored-ul-rate-mbps=0.05 \
  --monitored-packet-size=1000 \
  --tn-tx-power-dbm=46 \
  --xhaul-tx-power-dbm=46 \
  --uav-tx-power-dbm=37 \
  --ue-tx-power-dbm=23 \
  --init-min-rsrp=-110 \
  --handover-min-rsrp-dbm=-110 \
  --enable-harq-retx=${ENABLE_HARQ_RETX} \
  --pdcp-discard-timer-ms=1000 \
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

NATURAL_MISSION_ARGS="--uav-control-start=5 \
  --uav-control-period=2 \
  --uav-underserved-rsrp-thresh-dbm=-110 \
  --uav-initial-area-half-w-m=3000 \
  --uav-initial-area-half-h-m=1500 \
  --uav-area-half-w-m=20000 \
  --uav-area-half-h-m=10000 \
  --uav-mission-target-scale=8 \
  --uav-speed-mps=25 \
  --xhaul-pathloss-exponent=5.6 \
  --xhaul-reference-distance-m=100 \
  --xhaul-healthy-rsrp-dbm=-110 \
  --xhaul-switch-to-sat-ttt-s=5 \
  --xhaul-switch-to-tn-ttt-s=5 \
  --enable-xhaul-channel-variation=1 \
  --xhaul-shadowing-stddev-db=6 \
  --xhaul-fading-stddev-db=4 \
  --channel-update-ms=100 \
  --channel-condition-update-ms=200"

# Build only the new example target. Configure first if this is a fresh build tree:
#   ./ns3 configure --build-profile=optimized --enable-examples
./ns3 build oran-nr-uav-xhaul-autonomy-example

run_ns3()
{
  local deployment_mode="$1"
  local run_label="$2"
  local seed="$3"
  local args="$4"
  local effective_label="${run_label}"
  local seed_arg=""

  if [[ -n "${seed}" ]]; then
    effective_label="${run_label}-seed${seed}"
    seed_arg="--RngRun=${seed}"
  fi

  echo
  echo "============================================================"
  echo "Running ${deployment_mode} / ${effective_label}"
  echo "============================================================"
  ./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=${deployment_mode} --run-label=${effective_label} ${seed_arg} ${args}"
}

run_four_main_scenarios()
{
  local seed="$1"

  # Scenario 1: UE + TN only with healthy terrestrial infrastructure.
  # This is the clean semi-urban TN reference. No UAV cells are installed,
  # no satellite monitor is enabled, and no artificial TN/donor-backhaul
  # degradation is applied. The xhaul-autonomy-trace.csv file will contain only
  # the header.
  run_ns3 "tn-only" "clean" "${seed}" "${COMMON_ARGS} ${TN_ONLY_ARGS}"

  # Scenario 2: UE + TN + UAV with healthy terrestrial donor backhaul.
  # UAVs are active cell nodes. The TN layer has 80 UE capacity while 100 UEs
  # are present after 5 s, so UAV cells are expected to help with access
  # capacity and coverage. Satellite backhaul is disabled.
  run_ns3 "tn-uav" "healthy-xhaul" "${seed}" "${COMMON_ARGS} ${UAV_ARGS}"

  # Scenario 3: UE + TN + UAV with distance-based donor-backhaul stress but no
  # satellite fallback. This is the fair no-satellite baseline. When UAV mission
  # movement makes the TN donor RSRP fall below the usable threshold, the UAV
  # route to the core becomes unavailable and normal UE handover to the UAV is
  # blocked.
  run_ns3 "tn-uav" "distance-loss-no-sat" "${seed}" "${COMMON_ARGS} ${UAV_ARGS} ${NATURAL_MISSION_ARGS}"

  # Scenario 4: UE + TN + UAV + satellite with the same distance-based donor
  # stress. If donor RSRP falls below the usable threshold and satellite SNR is
  # healthy, the Near-RT RIC switching xApp switches the core path to
  # UAV->SAT->GW->core.
  run_ns3 "tn-uav-satellite" "distance-loss-sat" "${seed}" "${COMMON_ARGS} ${UAV_ARGS} --sat-backhaul-scenario=NTN-Suburban --enable-sat-backhaul-monitor=1 --enable-uav-switching-xapp=1 ${NATURAL_MISSION_ARGS}"
}

if [[ -n "${SEEDS}" ]]; then
  for seed in ${SEEDS}; do
    run_four_main_scenarios "${seed}"
  done
else
  run_four_main_scenarios ""
fi
