#!/usr/bin/env bash
set -euo pipefail

# Run from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#
# Optional quick smoke test:
#   RUN_LIMIT=2 bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#
# Purpose:
#   Generate a supervised-learning dataset for a future UAV TN/satellite
#   switching xApp. Each run writes a unique --run-label so output folders do
#   not overwrite each other.

RUN_LIMIT="${RUN_LIMIT:-0}"
RUN_COUNT=0

COMMON_ARGS="--sim-time=120 \
  --num-tn-gnbs=4 \
  --rlc-mode=am \
  --monitored-traffic=udp \
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

UAV_BASE_ARGS="--num-uess1=50 \
  --num-ground-ues=50 \
  --ground-attach-delay=5 \
  --max-ues-tn=20 \
  --max-ues-ntn=10 \
  --num-ntn-gnbs=3"

MISSION_BASE_ARGS="--uav-control-start=30 \
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

run_one() {
  local label="$1"
  local deployment_mode="$2"
  local satellite_enabled="$3"
  local extra_args="$4"

  RUN_COUNT=$((RUN_COUNT + 1))
  if [[ "${RUN_LIMIT}" != "0" && "${RUN_COUNT}" -gt "${RUN_LIMIT}" ]]; then
    echo "[ai-sweep] RUN_LIMIT=${RUN_LIMIT} reached; stopping before ${label}"
    exit 0
  fi

  local sat_args=""
  if [[ "${satellite_enabled}" == "1" ]]; then
    sat_args="--sat-backhaul-scenario=NTN-Suburban --enable-sat-backhaul-monitor=1 --enable-uav-switching-xapp=1"
  fi

  echo
  echo "[ai-sweep] ${RUN_COUNT}: ${label}"
  ./ns3 run "oran-nr-uav-xhaul-autonomy-example \
    --deployment-mode=${deployment_mode} \
    --run-label=${label} \
    ${COMMON_ARGS} \
    ${UAV_BASE_ARGS} \
    ${MISSION_BASE_ARGS} \
    ${sat_args} \
    ${extra_args}"
}

run_pair() {
  local label="$1"
  local extra_args="$2"

  run_one "ai-${label}-no-sat" "tn-uav" "0" "${extra_args}"
  run_one "ai-${label}-sat" "tn-uav-satellite" "1" "${extra_args}"
}

echo "[ai-sweep] Building oran-nr-uav-xhaul-autonomy-example"
./ns3 build oran-nr-uav-xhaul-autonomy-example

# 1) Random-seed sweep. ns-3 accepts RngRun as a global command-line value.
for seed in 1 2 3 4 5 6 7 8 9 10; do
  run_pair "seed${seed}" "--RngRun=${seed}"
done

# 2) UAV mobility speed sweep.
for speed in 10 15 25 35; do
  run_pair "speed${speed}" "--uav-speed-mps=${speed}"
done

# 3) Donor outage timing sweep.
run_pair "outage30-75" "--xhaul-donor-unavailable-start=30 --xhaul-donor-unavailable-stop=75"
run_pair "outage40-90" "--xhaul-donor-unavailable-start=40 --xhaul-donor-unavailable-stop=90"
run_pair "outage60-105" "--xhaul-donor-unavailable-start=60 --xhaul-donor-unavailable-stop=105"

# 4) UE-load sweep.
run_pair "load50-50" "--num-uess1=50 --num-ground-ues=50"
run_pair "load60-60" "--num-uess1=60 --num-ground-ues=60"
run_pair "load70-70" "--num-uess1=70 --num-ground-ues=70"

# 5) Traffic-demand sweep.
run_pair "dl0p1" "--monitored-dl-rate-mbps=0.1"
run_pair "dl0p2" "--monitored-dl-rate-mbps=0.2"
run_pair "dl0p4" "--monitored-dl-rate-mbps=0.4"

# 6) Donor RSRP threshold sweep.
run_pair "rsrp-105" "--xhaul-healthy-rsrp-dbm=-105"
run_pair "rsrp-110" "--xhaul-healthy-rsrp-dbm=-110"
run_pair "rsrp-115" "--xhaul-healthy-rsrp-dbm=-115"

# 7) Channel-variation severity sweep.
run_pair "chan4-2" "--xhaul-shadowing-stddev-db=4 --xhaul-fading-stddev-db=2"
run_pair "chan6-4" "--xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4"
run_pair "chan8-6" "--xhaul-shadowing-stddev-db=8 --xhaul-fading-stddev-db=6"

# 8) Satellite channel scenario sweep. Satellite-only because no-satellite runs
# do not use the satellite channel model.
run_one "ai-sat-ntn-suburban" "tn-uav-satellite" "1" "--sat-backhaul-scenario=NTN-Suburban"
run_one "ai-sat-ntn-urban" "tn-uav-satellite" "1" "--sat-backhaul-scenario=NTN-Urban"

echo
echo "[ai-sweep] Completed ${RUN_COUNT} runs"
