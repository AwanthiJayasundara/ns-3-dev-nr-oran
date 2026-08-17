#!/usr/bin/env bash
set -euo pipefail

# Clustered underserved-user coverage experiment.
#
# Runs the same physical scenario for three outside-UE demand levels:
#   60 central + 20 outside = 80 total UEs
#   60 central + 30 outside = 90 total UEs
#   60 central + 40 outside = 100 total UEs
#
# UAV capacity is set to 15 UEs per UAV, so three UAVs can serve up to
# 45 outside/underserved users.
#
# The default is the compact 90 s TN+UAV ISAC-reactive development profile.
# RUN_HOTSPOT_RF is intentionally distinct from the legacy xHaul ONNX switch;
# the executable currently rejects RUN_HOTSPOT_RF=1 until RF inference exists.

SIM_TIME="${SIM_TIME:-90}"
SEEDS="${SEEDS:-1}"
JOBS="${JOBS:-1}"
UES2_LOADS="${UES2_LOADS:-20 30 40}"
SCENARIO_PROFILE="${SCENARIO_PROFILE:-compact}"
EXPERIMENT_TAG="${EXPERIMENT_TAG:-main}"
RUN_HOTSPOT_RF="${RUN_HOTSPOT_RF:-0}"
ENABLE_ISAC_SENSING="${ENABLE_ISAC_SENSING:-1}"
ENABLE_UAV_REPOSITIONING="${ENABLE_UAV_REPOSITIONING:-1}"
ENABLE_NS3_LOG="${ENABLE_NS3_LOG:-0}"
ENABLE_NR_HELPER_INFO_LOG="${ENABLE_NR_HELPER_INFO_LOG:-0}"
METHODS="${METHODS:-}"
ALLOW_EXISTING_RESULTS="${ALLOW_EXISTING_RESULTS:-0}"
NUM_TN_GNBS="${NUM_TN_GNBS:-4}"
NUM_UAVS="${NUM_UAVS:-3}"
MAX_UES_TN="${MAX_UES_TN:-20}"
MAX_UES_UAV="${MAX_UES_UAV:-15}"
TN_TX_POWER_DBM="${TN_TX_POWER_DBM:-46}"
UAV_TX_POWER_DBM="${UAV_TX_POWER_DBM:-37}"
UE_TX_POWER_DBM="${UE_TX_POWER_DBM:-23}"
UNDERSERVED_RSRP_DBM="${UNDERSERVED_RSRP_DBM:--100}"
UAV_SPEED_MPS="${UAV_SPEED_MPS:-15}"
UAV_CONTROL_PERIOD_S="${UAV_CONTROL_PERIOD_S:-1}"
UNDERSERVED_PERSISTENCE_S="${UNDERSERVED_PERSISTENCE_S:-6}"
UNKNOWN_TIMEOUT_S="${UNKNOWN_TIMEOUT_S:-6}"
RSRP_MAX_AGE_S="${RSRP_MAX_AGE_S:-5}"
HANDOVER_TTT_S="${HANDOVER_TTT_S:-2}"
POSITION_TRACE_INTERVAL_S="${POSITION_TRACE_INTERVAL_S:-2}"
ISAC_TX_POWER_DBM="${ISAC_TX_POWER_DBM:-30}"
ISAC_RCS_M2="${ISAC_RCS_M2:-1}"
ISAC_SYSTEM_LOSS_LINEAR="${ISAC_SYSTEM_LOSS_LINEAR:-1}"
ISAC_DETECTION_MIDPOINT_DB="${ISAC_DETECTION_MIDPOINT_DB:--15}"
HOTSPOT_RF_MODEL="${HOTSPOT_RF_MODEL:-results/ai/hotspot-rf/hotspot_rf.onnx}"
OUTPUT_ROOT="${OUTPUT_ROOT:-results/nr/tn-ntn}"
OUTPUT_PARENT_DIR="${OUTPUT_PARENT_DIR:-${OUTPUT_ROOT}/${EXPERIMENT_TAG}}"
DB_DIR="${DB_DIR:-${OUTPUT_PARENT_DIR}/db}"
RUNNER_LOG_DIR="${RUNNER_LOG_DIR:-${OUTPUT_PARENT_DIR}/runner-logs}"

if ! [[ "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[clustered-coverage] JOBS must be a positive integer; got '${JOBS}'" >&2
  exit 1
fi
for bool_name in RUN_HOTSPOT_RF ENABLE_ISAC_SENSING ENABLE_UAV_REPOSITIONING ENABLE_NS3_LOG ENABLE_NR_HELPER_INFO_LOG ALLOW_EXISTING_RESULTS; do
  bool_value="${!bool_name}"
  if [[ "${bool_value}" != "0" && "${bool_value}" != "1" ]]; then
    echo "[clustered-coverage] ${bool_name} must be 0 or 1; got '${bool_value}'" >&2
    exit 1
  fi
done
if ! [[ "${EXPERIMENT_TAG}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "[clustered-coverage] EXPERIMENT_TAG may contain only letters, digits, dot, underscore, and hyphen" >&2
  exit 1
fi
for requested_method in ${METHODS}; do
  case "${requested_method}" in
    static|oracle-reactive|isac-reactive|isac-rf-predictive) ;;
    *)
      echo "[clustered-coverage] unsupported method '${requested_method}'" >&2
      echo "[clustered-coverage] allowed methods: static oracle-reactive isac-reactive isac-rf-predictive" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${DB_DIR}" "${RUNNER_LOG_DIR}"

case "${SCENARIO_PROFILE}" in
  compact)
    GEOMETRY_ARGS="\
--tn-area-half-w-m=100 \
--tn-area-half-h-m=100 \
--ues2-cluster-radius-m=100 \
--ues2-cluster-offset-m=350 \
--ue-area-half-w-m=1000 \
--ue-area-half-h-m=1000 \
--uav-area-half-w-m=1000 \
--uav-area-half-h-m=1000 \
--uav-initial-area-half-w-m=100 \
--uav-initial-area-half-h-m=100"
    PROFILE_CONTROL_START_S=12
    ;;
  large)
    GEOMETRY_ARGS="\
--tn-area-half-w-m=2500 \
--tn-area-half-h-m=1200 \
--ues2-cluster-radius-m=500 \
--ues2-cluster-offset-m=3500 \
--ue-area-half-w-m=5000 \
--ue-area-half-h-m=5000 \
--uav-area-half-w-m=5000 \
--uav-area-half-h-m=5000 \
--uav-initial-area-half-w-m=3000 \
--uav-initial-area-half-h-m=1500"
    PROFILE_CONTROL_START_S=5
    ;;
  *)
    echo "[clustered-coverage] SCENARIO_PROFILE must be compact or large; got '${SCENARIO_PROFILE}'" >&2
    exit 1
    ;;
esac
UAV_CONTROL_START_S="${UAV_CONTROL_START_S:-${PROFILE_CONTROL_START_S}}"

EXE="${EXE:-}"
if [[ -z "${EXE}" ]]; then
  SOURCE_FILE="contrib/oran/examples/oran-nr-uav-xhaul-autonomy-example.cc"
  SERVER_OPT_EXE="build-optimized/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized"
  OPT_EXE="build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized"
  DBG_EXE="build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-debug"
  if [[ -x "${SERVER_OPT_EXE}" && "${SERVER_OPT_EXE}" -nt "${SOURCE_FILE}" ]]; then
    EXE="${SERVER_OPT_EXE}"
  elif [[ -x "${OPT_EXE}" && "${OPT_EXE}" -nt "${SOURCE_FILE}" &&
          -x "${DBG_EXE}" && "${DBG_EXE}" -nt "${SOURCE_FILE}" ]]; then
    if [[ "${DBG_EXE}" -nt "${OPT_EXE}" ]]; then
      EXE="${DBG_EXE}"
    else
      EXE="${OPT_EXE}"
    fi
  elif [[ -x "${OPT_EXE}" && "${OPT_EXE}" -nt "${SOURCE_FILE}" ]]; then
    EXE="${OPT_EXE}"
  elif [[ -x "${DBG_EXE}" && "${DBG_EXE}" -nt "${SOURCE_FILE}" ]]; then
    EXE="${DBG_EXE}"
  else
    echo "[clustered-coverage] no up-to-date executable found; rebuild the example first" >&2
    exit 1
  fi
fi

COMMON_ARGS="\
--deployment-mode=tn-uav \
--sim-time=${SIM_TIME} \
--output-parent-dir=${OUTPUT_PARENT_DIR} \
--num-uess1=60 \
--ground-attach-delay=5 \
--num-tn-gnbs=${NUM_TN_GNBS} \
--num-ntn-gnbs=${NUM_UAVS} \
--max-ues-tn=${MAX_UES_TN} \
--max-ues-ntn=${MAX_UES_UAV} \
--split-ue-placement=1 \
--clustered-ues2-placement=1 \
--uav-target-ues2-only=0 \
${GEOMETRY_ARGS} \
--rlc-mode=am \
--monitored-traffic=udp \
--monitored-dl-rate-mbps=0.2 \
--monitored-ul-rate-mbps=0.05 \
--monitored-packet-size=1000 \
--tn-tx-power-dbm=${TN_TX_POWER_DBM} \
--uav-tx-power-dbm=${UAV_TX_POWER_DBM} \
--ue-tx-power-dbm=${UE_TX_POWER_DBM} \
--init-min-rsrp=-110 \
--handover-min-rsrp-dbm=-110 \
--enable-harq-retx=0 \
--pdcp-discard-timer-ms=1000 \
--enable-startup-ping=0 \
--enable-flow-monitor=1 \
--enable-rsrp-trace=1 \
--enable-position-trace=1 \
--position-trace-interval=${POSITION_TRACE_INTERVAL_S} \
--enable-handover-trace=1 \
--enable-handover-failure-trace=1 \
--enable-decision-csv=0 \
--enable-oran-info-log=${ENABLE_NS3_LOG} \
--enable-nr-helper-info-log=${ENABLE_NR_HELPER_INFO_LOG} \
--enable-setup-prints=0 \
--enable-progress=0 \
--enable-oran-app-loss-reports=0 \
--enable-oran-cell-load-reports=0 \
--enable-fh-control=0 \
--rem-mode=0 \
--enable-sat-backhaul-monitor=0 \
--enable-uav-switching-xapp=0 \
--uav-control-start=${UAV_CONTROL_START_S} \
--uav-control-period=${UAV_CONTROL_PERIOD_S} \
--uav-underserved-rsrp-thresh-dbm=${UNDERSERVED_RSRP_DBM} \
--uav-underserved-persistence-s=${UNDERSERVED_PERSISTENCE_S} \
--uav-unknown-timeout-s=${UNKNOWN_TIMEOUT_S} \
--uav-rsrp-measurement-max-age-s=${RSRP_MAX_AGE_S} \
--handover-ttt-s=${HANDOVER_TTT_S} \
--isac-sensing-frequency-hz=4.0e9 \
--isac-sensing-tx-power-dbm=${ISAC_TX_POWER_DBM} \
--isac-target-rcs-m2=${ISAC_RCS_M2} \
--isac-sensing-system-loss-linear=${ISAC_SYSTEM_LOSS_LINEAR} \
--isac-detection-midpoint-db=${ISAC_DETECTION_MIDPOINT_DB} \
--uav-access-scenario=UMa \
--hotspot-rf-model=${HOTSPOT_RF_MODEL} \
--uav-mission-target-scale=1 \
--uav-speed-mps=${UAV_SPEED_MPS} \
--ues1-speed-mps=5 \
--ues2-speed-mps=4 \
--channel-update-ms=100 \
--channel-condition-update-ms=200"

run_case() {
  local controller="$1"
  local ues2="$2"
  local seed="$3"
  local label="$4"
  local db_file="${DB_DIR}/${label}.db"
  local console_log="${RUNNER_LOG_DIR}/${label}.console.log"
  local method_repositioning
  local method_sensing
  local method_rf

  case "${controller}" in
    static)
      method_repositioning=0
      method_sensing=0
      method_rf=0
      ;;
    oracle-reactive)
      method_repositioning=1
      method_sensing=0
      method_rf=0
      ;;
    isac-reactive)
      method_repositioning=1
      method_sensing=1
      method_rf=0
      ;;
    isac-rf-predictive)
      method_repositioning=1
      method_sensing=1
      method_rf=1
      ;;
    *)
      echo "[clustered-coverage] internal error: unsupported method '${controller}'" >&2
      return 1
      ;;
  esac

  if [[ -e "${db_file}" && "${ALLOW_EXISTING_RESULTS}" != "1" ]]; then
    echo "[clustered-coverage] refusing to overwrite existing database: ${db_file}" >&2
    echo "[clustered-coverage] choose a new EXPERIMENT_TAG, or set ALLOW_EXISTING_RESULTS=1 to overwrite it intentionally" >&2
    return 1
  fi
  echo "[clustered-coverage] controller=${controller} seed=${seed} ues2=${ues2} sim=${SIM_TIME}s label=${label}"
  "${EXE}" ${COMMON_ARGS} \
    --enable-uav-repositioning="${method_repositioning}" \
    --enable-isac-sensing="${method_sensing}" \
    --enable-hotspot-rf="${method_rf}" \
    --RngRun="${seed}" \
    --num-ground-ues="${ues2}" \
    --db-file="${db_file}" \
    --run-label="${label}" 2>&1 | tee "${console_log}"
}

run_seed() {
  local controller="$1"
  local seed="$2"
  local label_prefix="$3"

  for ues2 in ${UES2_LOADS}; do
    run_case "${controller}" \
      "${ues2}" \
      "${seed}" \
      "${label_prefix}-clustered-60central-${ues2}out-seed${seed}-${SIM_TIME}s"
  done
}

run_seed_batches() {
  local controller="$1"
  local label_prefix="$2"
  local active=0

  for seed in ${SEEDS}; do
    run_seed "${controller}" "${seed}" "${label_prefix}" &
    active=$((active + 1))

    if (( active >= JOBS )); then
      if ! wait -n; then
        echo "[clustered-coverage] a ${controller} seed batch failed" >&2
        return 1
      fi
      active=$((active - 1))
    fi
  done

  while (( active > 0 )); do
    if ! wait -n; then
      echo "[clustered-coverage] a ${controller} seed batch failed" >&2
      return 1
    fi
    active=$((active - 1))
  done
}

if [[ -n "${METHODS}" ]]; then
  for METHOD in ${METHODS}; do
    run_seed_batches "${METHOD}" "${EXPERIMENT_TAG}-${SCENARIO_PROFILE}-${METHOD}"
  done
else
  if [[ "${ENABLE_UAV_REPOSITIONING}" == "0" ]]; then
    METHOD="static"
  elif [[ "${RUN_HOTSPOT_RF}" == "1" ]]; then
    METHOD="isac-rf-predictive"
  elif [[ "${ENABLE_ISAC_SENSING}" == "1" ]]; then
    METHOD="isac-reactive"
  else
    METHOD="oracle-reactive"
  fi
  run_seed_batches "${METHOD}" "${EXPERIMENT_TAG}-${SCENARIO_PROFILE}-${METHOD}"
fi
