#!/usr/bin/env bash
set -euo pipefail

# Clustered underserved-user coverage experiment.
#
# Runs the same physical scenario for three outside-UE demand levels:
#   20 UEs: light underserved demand
#   30 UEs: medium underserved demand
#   40 UEs: high underserved demand
#
# UAV capacity is set to 15 UEs per UAV, so three UAVs can serve up to
# 45 outside/underserved users.
#
# By default this script runs the rule-based controller first. Set
# RUN_AI=1 to also run the RF-AI switching xApp after the rule-based runs.

SIM_TIME="${SIM_TIME:-200}"
SEEDS="${SEEDS:-1}"
RUN_AI="${RUN_AI:-0}"
AI_MODEL="${AI_MODEL:-results/ai/uav-switching-xapp-ai-dataset-v2/uav_switching_xapp_model.onnx}"

EXE="${EXE:-}"
if [[ -z "${EXE}" ]]; then
  OPT_EXE="build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized"
  DBG_EXE="build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-debug"
  if [[ -x "${OPT_EXE}" && -x "${DBG_EXE}" ]]; then
    if [[ "${DBG_EXE}" -nt "${OPT_EXE}" ]]; then
      EXE="${DBG_EXE}"
    else
      EXE="${OPT_EXE}"
    fi
  elif [[ -x "${OPT_EXE}" ]]; then
    EXE="${OPT_EXE}"
  else
    EXE="${DBG_EXE}"
  fi
fi

COMMON_ARGS="\
--deployment-mode=tn-uav-satellite \
--sim-time=${SIM_TIME} \
--num-uess1=80 \
--ground-attach-delay=5 \
--num-tn-gnbs=4 \
--num-ntn-gnbs=3 \
--max-ues-tn=20 \
--max-ues-ntn=15 \
--split-ue-placement=1 \
--clustered-ues2-placement=1 \
--ues2-cluster-radius-m=500 \
--ues2-cluster-offset-m=3500 \
--ue-area-half-w-m=5000 \
--ue-area-half-h-m=5000 \
--uav-area-half-w-m=5000 \
--uav-area-half-h-m=5000 \
--uav-initial-area-half-w-m=3000 \
--uav-initial-area-half-h-m=1500 \
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
--enable-harq-retx=0 \
--pdcp-discard-timer-ms=1000 \
--enable-startup-ping=0 \
--enable-flow-monitor=1 \
--enable-rsrp-trace=1 \
--enable-position-trace=1 \
--enable-handover-trace=1 \
--enable-handover-failure-trace=1 \
--enable-decision-csv=1 \
--enable-oran-info-log=1 \
--sat-backhaul-scenario=NTN-Suburban \
--enable-sat-backhaul-monitor=1 \
--enable-uav-switching-xapp=1 \
--uav-control-start=5 \
--uav-control-period=1 \
--uav-underserved-rsrp-thresh-dbm=-100 \
--uav-mission-target-scale=1 \
--uav-speed-mps=35 \
--xhaul-pathloss-exponent=5.6 \
--xhaul-reference-distance-m=100 \
--xhaul-healthy-rsrp-dbm=-100 \
--xhaul-switch-to-sat-ttt-s=5 \
--xhaul-switch-to-tn-ttt-s=5 \
--enable-xhaul-channel-variation=1 \
--xhaul-shadowing-stddev-db=6 \
--xhaul-fading-stddev-db=4 \
--channel-update-ms=100 \
--channel-condition-update-ms=200"

run_case() {
  local controller="$1"
  local ues2="$2"
  local seed="$3"
  local label="$4"
  local ai_args=""

  if [[ "${controller}" == "rf-ai" ]]; then
    ai_args="--enable-ai-uav-switching-xapp=1 --uav-switching-ai-model=${AI_MODEL}"
  fi

  echo "[clustered-coverage] controller=${controller} seed=${seed} ues2=${ues2} sim=${SIM_TIME}s label=${label}"
  "${EXE}" ${COMMON_ARGS} \
    --RngRun="${seed}" \
    --num-ground-ues="${ues2}" \
    --run-label="${label}" \
    ${ai_args}
}

for seed in ${SEEDS}; do
  for ues2 in 20 30 40; do
    run_case "rule" "${ues2}" "${seed}" "rule-clustered-80tn-${ues2}out-seed${seed}-${SIM_TIME}s"
  done
done

if [[ "${RUN_AI}" == "1" ]]; then
  if [[ ! -f "${AI_MODEL}" ]]; then
    echo "[clustered-coverage] AI model not found: ${AI_MODEL}" >&2
    exit 1
  fi

  for seed in ${SEEDS}; do
    for ues2 in 20 30 40; do
      run_case "rf-ai" "${ues2}" "${seed}" "rf-ai-clustered-80tn-${ues2}out-seed${seed}-${SIM_TIME}s"
    done
  done
fi
