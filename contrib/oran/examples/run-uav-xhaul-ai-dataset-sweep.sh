#!/usr/bin/env bash
set -euo pipefail

# Run from the repository root:
#   bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#
# Optional quick smoke test:
#   RUN_LIMIT=2 bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#
# Purpose:
#   Generate a supervised-learning dataset for the UAV TN/satellite switching
#   xApp using only the two degraded-donor comparison cases:
#     1) TN+UAV with no satellite fallback
#     2) TN+UAV+satellite with switching-xApp fallback
#
#   The default mode repeats these two cases over several random seeds and
#   three donor-loss models. This gives the AI xApp optimistic, nominal, and
#   stressed donor-backhaul examples without mixing in TN-only or healthy-
#   backhaul baselines.
#
# Examples:
#   SEEDS="1 2 3 4 5 6 7 8 9 10" bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#   LOSS_MODELS="fspl logdist logdist-fading" SEEDS="1 2" bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#   AI_RUN_TAG=ai-dataset-v1 LOSS_MODELS="fspl logdist logdist-fading" SEEDS="1 2" bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#   RUN_LIMIT=2 bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh
#   INCLUDE_PARAMETER_SWEEPS=1 bash contrib/oran/examples/run-uav-xhaul-ai-dataset-sweep.sh

RUN_LIMIT="${RUN_LIMIT:-0}"
SEEDS="${SEEDS:-1 2 3 4 5 6 7 8 9 10}"
LOSS_MODELS="${LOSS_MODELS:-fspl logdist logdist-fading}"
AI_RUN_TAG="${AI_RUN_TAG:-ai-dataset-v1}"
AI_OUTPUT_PARENT_DIR="${AI_OUTPUT_PARENT_DIR:-results/nr/tn-ntn/${AI_RUN_TAG}}"
INCLUDE_PARAMETER_SWEEPS="${INCLUDE_PARAMETER_SWEEPS:-0}"
ENABLE_HARQ_RETX="${ENABLE_HARQ_RETX:-0}"
RUN_COUNT=0

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

UAV_BASE_ARGS="--num-uess1=50 \
  --num-ground-ues=50 \
  --ground-attach-delay=5 \
  --max-ues-tn=20 \
  --max-ues-ntn=10 \
  --num-ntn-gnbs=3"

MISSION_BASE_ARGS="--uav-control-start=5 \
  --uav-control-period=2 \
  --uav-underserved-rsrp-thresh-dbm=-110 \
  --uav-initial-area-half-w-m=3000 \
  --uav-initial-area-half-h-m=1500 \
  --uav-area-half-w-m=20000 \
  --uav-area-half-h-m=10000 \
  --uav-mission-target-scale=4 \
  --uav-speed-mps=25 \
  --xhaul-reference-distance-m=100 \
  --xhaul-healthy-rsrp-dbm=-110 \
  --xhaul-switch-to-sat-ttt-s=5 \
  --xhaul-switch-to-tn-ttt-s=5 \
  --channel-update-ms=100 \
  --channel-condition-update-ms=200"

loss_model_args() {
  local model="$1"
  case "${model}" in
    fspl)
      echo "--xhaul-pathloss-exponent=2.0 --enable-xhaul-channel-variation=0 --xhaul-shadowing-stddev-db=0 --xhaul-fading-stddev-db=0"
      ;;
    logdist)
      echo "--xhaul-pathloss-exponent=4.8 --enable-xhaul-channel-variation=0 --xhaul-shadowing-stddev-db=0 --xhaul-fading-stddev-db=0"
      ;;
    logdist-fading)
      echo "--xhaul-pathloss-exponent=4.8 --enable-xhaul-channel-variation=1 --xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4"
      ;;
    *)
      echo "[ai-sweep] Unknown LOSS_MODEL '${model}'. Use: fspl logdist logdist-fading" >&2
      exit 2
      ;;
  esac
}

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
  mkdir -p "${AI_OUTPUT_PARENT_DIR}"
  ./ns3 run "oran-nr-uav-xhaul-autonomy-example \
    --deployment-mode=${deployment_mode} \
    --run-label=${label} \
    --output-parent-dir=${AI_OUTPUT_PARENT_DIR} \
    ${COMMON_ARGS} \
    ${UAV_BASE_ARGS} \
    ${MISSION_BASE_ARGS} \
    ${sat_args} \
    ${extra_args}"
}

run_pair() {
  local model="$1"
  local label="$2"
  local extra_args="$3"
  local model_args
  model_args="$(loss_model_args "${model}")"

  run_one "${AI_RUN_TAG}-${model}-${label}-no-sat" "tn-uav" "0" "${model_args} ${extra_args}"
  run_one "${AI_RUN_TAG}-${model}-${label}-sat" "tn-uav-satellite" "1" "${model_args} ${extra_args}"
}

echo "[ai-sweep] Building oran-nr-uav-xhaul-autonomy-example"
./ns3 build oran-nr-uav-xhaul-autonomy-example

# 1) Random-seed and donor-loss-model sweep. ns-3 accepts RngRun as a
# global command-line value.
for seed in ${SEEDS}; do
  for model in ${LOSS_MODELS}; do
    run_pair "${model}" "seed${seed}" "--RngRun=${seed}"
  done
done

if [[ "${INCLUDE_PARAMETER_SWEEPS}" != "1" ]]; then
  echo
  echo "[ai-sweep] Completed ${RUN_COUNT} seed-pair runs"
  exit 0
fi

# 2) UAV mobility speed sweep.
for speed in 10 15 25; do
  run_pair "logdist-fading" "speed${speed}" "--uav-speed-mps=${speed}"
done

# 3) Mission-start timing sweep. No forced donor outage is used; donor loss
# comes from UAV movement, distance/path loss, and channel variation.
run_pair "logdist-fading" "mission5" "--uav-control-start=5"
run_pair "logdist-fading" "mission30" "--uav-control-start=30"
run_pair "logdist-fading" "mission40" "--uav-control-start=40"
run_pair "logdist-fading" "mission60" "--uav-control-start=60"

# 4) UE-load sweep.
run_pair "logdist-fading" "load50-50" "--num-uess1=50 --num-ground-ues=50"
run_pair "logdist-fading" "load60-60" "--num-uess1=60 --num-ground-ues=60"
run_pair "logdist-fading" "load70-70" "--num-uess1=70 --num-ground-ues=70"

# 5) Traffic-demand sweep.
run_pair "logdist-fading" "dl0p1" "--monitored-dl-rate-mbps=0.1"
run_pair "logdist-fading" "dl0p2" "--monitored-dl-rate-mbps=0.2"
run_pair "logdist-fading" "dl0p4" "--monitored-dl-rate-mbps=0.4"

# 6) Donor RSRP threshold sweep.
run_pair "logdist-fading" "rsrp-110" "--xhaul-healthy-rsrp-dbm=-110"
run_pair "logdist-fading" "rsrp-120" "--xhaul-healthy-rsrp-dbm=-120"
run_pair "logdist-fading" "rsrp-125" "--xhaul-healthy-rsrp-dbm=-125"

# 7) Channel-variation severity sweep.
run_pair "logdist-fading" "chan4-2" "--xhaul-shadowing-stddev-db=4 --xhaul-fading-stddev-db=2"
run_pair "logdist-fading" "chan6-4" "--xhaul-shadowing-stddev-db=6 --xhaul-fading-stddev-db=4"
run_pair "logdist-fading" "chan8-6" "--xhaul-shadowing-stddev-db=8 --xhaul-fading-stddev-db=6"

# 8) Satellite channel scenario sweep. Satellite-only because no-satellite runs
# do not use the satellite channel model.
run_one "${AI_RUN_TAG}-sat-ntn-suburban" "tn-uav-satellite" "1" "--sat-backhaul-scenario=NTN-Suburban"
run_one "${AI_RUN_TAG}-sat-ntn-urban" "tn-uav-satellite" "1" "--sat-backhaul-scenario=NTN-Urban"

echo
echo "[ai-sweep] Completed ${RUN_COUNT} runs"
