#!/usr/bin/env bash
# Run the frozen v8 100-UE reviewer pilot.  This file only runs simulations when
# invoked explicitly; sourcing it exposes run_reviewer_arm for the campaign.
# Its fixed-association, capacity-stratified design supports backhaul-switching
# and QoS evidence.  Handover-success and UE ping-pong claims are not applicable.

set -euo pipefail

PILOT_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PILOT_SCRIPT_DIR
PILOT_REPO_ROOT="$(cd "${PILOT_SCRIPT_DIR}/../../.." && pwd -P)"
readonly PILOT_REPO_ROOT
readonly PILOT_EXAMPLE="${PILOT_REPO_ROOT}/contrib/oran/examples/oran-nr-uav-xhaul-autonomy-example.cc"
readonly PILOT_HELPER="${PILOT_REPO_ROOT}/src/nr/helper/nr-hybrid-sat-epc-helper.cc"
readonly PILOT_EXECUTABLE_DEFAULT="${PILOT_REPO_ROOT}/build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized"
readonly PILOT_NR_LIBRARY="${PILOT_REPO_ROOT}/build/lib/libns3-dev-nr-optimized.so"
readonly PILOT_MODEL="${PILOT_REPO_ROOT}/results/ai/uav-sustained-outage-v1/sustained_outage_rf_v1.onnx"
readonly PILOT_SCHEMA="${PILOT_REPO_ROOT}/results/ai/uav-sustained-outage-v1/input_output_schema_v1.json"
readonly PILOT_MODEL_SHA256="2f9bb76fe6d161cf8d4ab6ff5031647f45fa7cb6916d60463b4cffadeca1207f"
readonly PILOT_SCHEMA_SHA256="7602c73891c528d2972cbf078032d9f42c4ef6de591f45bdcb2b3ccb9af2dbb9"
readonly PILOT_VALIDATOR="${PILOT_SCRIPT_DIR}/validate-uav-reviewer-pilot-v8.py"
readonly PILOT_RUNNER="${PILOT_SCRIPT_DIR}/run-uav-reviewer-pilot-v8.sh"
readonly PILOT_CAMPAIGN_RUNNER="${PILOT_SCRIPT_DIR}/run-uav-reviewer-campaign-v8.sh"
readonly PILOT_CMAKE_CACHE="${PILOT_REPO_ROOT}/cmake-cache/CMakeCache.txt"
readonly PILOT_ROUTE_CONTRACT_REPORT="${PILOT_REPO_ROOT}/results/nr/tn-ntn/route-contract-v20/route-contract.json"
readonly PILOT_ROUTE_CONTRACT_MANIFEST="${PILOT_REPO_ROOT}/results/nr/tn-ntn/route-contract-v20/meta/frozen-inputs.sha256"
readonly PILOT_ROUTE_CONTRACT_REPORT_SHA256="8d48891d4f749bd2113a939b510b81d03bfe008aa4bbdcb07c60d160bfcc69a3"
readonly PILOT_ROUTE_CONTRACT_MANIFEST_SHA256="6c647780637e63b03845f0169bf5a24b1071fcb821d6027730ec7ad01d10193b"
readonly PILOT_ROUTE_BINARY_SHA256="4bb117a93f2e99855a75ebc5b18a9156f1e89f3c1986d240a925b66e0c546aff"
readonly PILOT_ROUTE_NR_LIBRARY_SHA256="20c771dccdbdd76b1793acf6ea985c1fcb8c6f5e2497c861cf722140d9566f24"
readonly PILOT_ROUTE_EXAMPLE_SHA256="be567402e35db83226d44c139519bc7d1364bf2e07f5a26b6504640430907e4e"
readonly PILOT_ROUTE_HELPER_SHA256="3e4d810761120eb084061a9b38351e8f06e8bd4fbcbe75adf764c35c3894aaaf"

# Frozen reviewer design.  Capacity and placement stay explicit so the first
# rule-only arm can verify UAV exposure before any broader campaign is allowed.
# S1 attaches first; TN aggregate capacity 40 therefore forces at least 10 of
# the 50 monitored UEs toward UAV cells when all monitored UEs attach.
readonly PILOT_MONITORED_UES=50
readonly PILOT_BACKGROUND_UES=50
readonly PILOT_EXPECTED_UES=$((PILOT_MONITORED_UES + PILOT_BACKGROUND_UES))
readonly PILOT_TN_GNBS=4
readonly PILOT_UAV_GNBS=3
readonly PILOT_MAX_UES_PER_TN_GNB=10
readonly PILOT_MAX_UES_PER_UAV_GNB=30
readonly PILOT_TN_TOTAL_CAPACITY=$((PILOT_TN_GNBS * PILOT_MAX_UES_PER_TN_GNB))
readonly PILOT_UAV_TOTAL_CAPACITY=$((PILOT_UAV_GNBS * PILOT_MAX_UES_PER_UAV_GNB))
readonly PILOT_EXPECTED_MONITORED_TN_UES=40
readonly PILOT_EXPECTED_MONITORED_UAV_UES=10
readonly PILOT_GROUND_ATTACH_DELAY_S=5
readonly PILOT_INITIAL_ATTACH_STAGGER_S=0.25
readonly PILOT_MONITORED_DL_START_S=6
readonly PILOT_MONITORED_UL_START_S=6.5
readonly PILOT_CAPACITY_STRATIFIED_UE_PLACEMENT=1
readonly PILOT_UE_ANCHOR_CLUSTER_RADIUS_M=25
readonly PILOT_UE_AREA_HALF_W_M=10000
readonly PILOT_UE_AREA_HALF_H_M=5000
readonly PILOT_UAV_INITIAL_AREA_HALF_W_M=1000
readonly PILOT_UAV_INITIAL_AREA_HALF_H_M=800
readonly PILOT_UAV_AREA_HALF_W_M=3000
readonly PILOT_UAV_AREA_HALF_H_M=1500
readonly PILOT_UAV_SPEED_MPS=0
readonly PILOT_MOBILITY_UPDATE_MS=200
readonly PILOT_XHAUL_PATHLOSS_EXPONENT=4.5

pilot_die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 2
}

PILOT_LAST_EXIT_CODE=0

pilot_sha256() {
    sha256sum -- "$1" | awk '{print $1}'
}

pilot_python() {
    local candidate
    for candidate in \
        "${PILOT_PYTHON:-}" \
        "${PILOT_REPO_ROOT}/../ns-3-dev-nr-oran/.venv-uav-ai/bin/python" \
        "${PILOT_REPO_ROOT}/.venv-uav-ai/bin/python" \
        "${PILOT_REPO_ROOT}/.venv/bin/python" \
        /usr/bin/python3; do
        if [[ -n "${candidate}" && -x "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done
    pilot_die "no Python 3 interpreter found"
}

pilot_assert_frozen_inputs() {
    local executable=$1
    local metadata_python
    [[ -x "${executable}" ]] || pilot_die \
        "optimized example binary is missing: ${executable}; build the single target first"
    [[ -f "${PILOT_EXAMPLE}" ]] || pilot_die "missing source: ${PILOT_EXAMPLE}"
    [[ -f "${PILOT_HELPER}" ]] || pilot_die "missing helper source: ${PILOT_HELPER}"
    [[ -f "${PILOT_NR_LIBRARY}" ]] || pilot_die "missing NR shared library: ${PILOT_NR_LIBRARY}"
    [[ -f "${PILOT_MODEL}" ]] || pilot_die "missing frozen model: ${PILOT_MODEL}"
    [[ -f "${PILOT_SCHEMA}" ]] || pilot_die "missing frozen schema: ${PILOT_SCHEMA}"
    [[ -f "${PILOT_VALIDATOR}" ]] || pilot_die "missing validator: ${PILOT_VALIDATOR}"
    [[ -f "${PILOT_CAMPAIGN_RUNNER}" ]] || pilot_die \
        "missing campaign runner: ${PILOT_CAMPAIGN_RUNNER}"
    [[ -f "${PILOT_CMAKE_CACHE}" ]] || pilot_die "missing CMake cache: ${PILOT_CMAKE_CACHE}"
    [[ -f "${PILOT_ROUTE_CONTRACT_REPORT}" ]] || pilot_die \
        "missing route-contract report: ${PILOT_ROUTE_CONTRACT_REPORT}"
    [[ -f "${PILOT_ROUTE_CONTRACT_MANIFEST}" ]] || pilot_die \
        "missing route-contract manifest: ${PILOT_ROUTE_CONTRACT_MANIFEST}"
    [[ "$(pilot_sha256 "${PILOT_MODEL}")" == "${PILOT_MODEL_SHA256}" ]] || pilot_die \
        "model SHA-256 does not match the frozen protocol"
    [[ "$(pilot_sha256 "${PILOT_SCHEMA}")" == "${PILOT_SCHEMA_SHA256}" ]] || pilot_die \
        "input/output schema SHA-256 does not match the frozen protocol"
    [[ "$(pilot_sha256 "${PILOT_ROUTE_CONTRACT_REPORT}")" == \
        "${PILOT_ROUTE_CONTRACT_REPORT_SHA256}" ]] || pilot_die \
        "route-contract-v20 report SHA-256 does not match the frozen protocol"
    [[ "$(pilot_sha256 "${PILOT_ROUTE_CONTRACT_MANIFEST}")" == \
        "${PILOT_ROUTE_CONTRACT_MANIFEST_SHA256}" ]] || pilot_die \
        "route-contract-v20 manifest SHA-256 does not match the frozen protocol"
    [[ "$(pilot_sha256 "${executable}")" == "${PILOT_ROUTE_BINARY_SHA256}" ]] || pilot_die \
        "executable SHA-256 does not match route-contract-v20"
    [[ "$(pilot_sha256 "${PILOT_NR_LIBRARY}")" == \
        "${PILOT_ROUTE_NR_LIBRARY_SHA256}" ]] || pilot_die \
        "NR shared-library SHA-256 does not match route-contract-v20"
    [[ "$(pilot_sha256 "${PILOT_EXAMPLE}")" == "${PILOT_ROUTE_EXAMPLE_SHA256}" ]] || pilot_die \
        "example source SHA-256 does not match route-contract-v20"
    [[ "$(pilot_sha256 "${PILOT_HELPER}")" == "${PILOT_ROUTE_HELPER_SHA256}" ]] || pilot_die \
        "route helper source SHA-256 does not match route-contract-v20"
    command -v /usr/bin/time >/dev/null || pilot_die "/usr/bin/time is required"
    ldd "${executable}" | grep -Fq -- "=> ${PILOT_NR_LIBRARY} " || pilot_die \
        "executable is not loading the frozen NR shared library: ${PILOT_NR_LIBRARY}"

    metadata_python=$(pilot_python)
    "${metadata_python}" - "${PILOT_ROUTE_CONTRACT_REPORT}" \
        "${PILOT_ROUTE_CONTRACT_MANIFEST}" "${executable}" "${PILOT_NR_LIBRARY}" \
        "${PILOT_EXAMPLE}" "${PILOT_HELPER}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

report_path, manifest_path, *required_paths = map(Path, sys.argv[1:])
report = json.loads(report_path.read_text(encoding="utf-8"))
checks = report.get("checks")
if (
    not report.get("passed")
    or not isinstance(checks, dict)
    or not checks
    or not all(checks.values())
):
    raise SystemExit(f"route/data-plane contract is not PASS: {report_path}")

frozen = {}
for line in manifest_path.read_text(encoding="utf-8").splitlines():
    if not line.strip():
        continue
    digest, filename = line.split(maxsplit=1)
    frozen[str(Path(filename).resolve())] = digest

for path in required_paths:
    resolved = str(path.resolve())
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if frozen.get(resolved) != digest:
        raise SystemExit(
            f"route-contract identity mismatch for {resolved}: "
            f"contract={frozen.get(resolved)!r} current={digest}"
        )
PY
}

pilot_common_args() {
    local rng_run=$1
    local sim_time_s=${2:-120}
    PILOT_COMMON_ARGS=(
        "--sim-time=${sim_time_s}"
        "--stop-tail=2"
        "--RngSeed=1"
        "--RngRun=${rng_run}"
        "--num-uess1=${PILOT_MONITORED_UES}"
        "--num-ground-ues=${PILOT_BACKGROUND_UES}"
        "--ground-attach-delay=${PILOT_GROUND_ATTACH_DELAY_S}"
        "--initial-attach-stagger-s=${PILOT_INITIAL_ATTACH_STAGGER_S}"
        "--num-tn-gnbs=${PILOT_TN_GNBS}"
        "--num-ntn-gnbs=${PILOT_UAV_GNBS}"
        "--max-ues-tn=${PILOT_MAX_UES_PER_TN_GNB}"
        "--max-ues-ntn=${PILOT_MAX_UES_PER_UAV_GNB}"
        "--use-oran=1"
        "--use-rsrp-lm=0"
        "--use-onnx-lm=0"
        "--use-torch-lm=0"
        "--lm-query-interval=2"
        "--e2-send-interval=2"
        "--tx-delay=0.1"
        "--handover-algorithm=ns3::NrNoOpHandoverAlgorithm"
        "--rlc-mode=um"
        "--monitored-traffic=udp"
        "--monitored-dl-rate-mbps=0.2"
        "--monitored-ul-rate-mbps=0.05"
        "--monitored-packet-size=1000"
        "--monitored-dl-start-s=${PILOT_MONITORED_DL_START_S}"
        "--monitored-ul-start-s=${PILOT_MONITORED_UL_START_S}"
        "--tn-tx-power-dbm=46"
        "--xhaul-tx-power-dbm=46"
        "--uav-tx-power-dbm=37"
        "--ue-tx-power-dbm=23"
        "--init-min-rsrp=-110"
        "--handover-min-rsrp-dbm=-110"
        "--enable-harq-retx=1"
        "--enable-fading=1"
        "--channel-update-ms=100"
        "--channel-condition-update-ms=200"
        "--pdcp-discard-timer-ms=1000"
        "--enable-startup-ping=0"
        "--enable-dedicated-qos-flows=1"
        "--enable-flow-monitor=1"
        "--enable-rsrp-trace=1"
        "--enable-position-trace=1"
        "--position-trace-interval=1"
        "--enable-handover-trace=1"
        "--enable-handover-failure-trace=1"
        "--enable-decision-csv=1"
        "--enable-oran-info-log=1"
        "--sat-backhaul-scenario=NTN-Suburban"
        "--sat-backhaul-min-snr-db=0"
        "--sat-backhaul-log-step-ms=500"
        "--uav-control-start=5"
        "--uav-control-period=2"
        "--uav-underserved-rsrp-thresh-dbm=-110"
        "--ue-area-half-w-m=${PILOT_UE_AREA_HALF_W_M}"
        "--ue-area-half-h-m=${PILOT_UE_AREA_HALF_H_M}"
        "--capacity-stratified-ue-placement=${PILOT_CAPACITY_STRATIFIED_UE_PLACEMENT}"
        "--ue-anchor-cluster-radius-m=${PILOT_UE_ANCHOR_CLUSTER_RADIUS_M}"
        "--uav-initial-area-half-w-m=${PILOT_UAV_INITIAL_AREA_HALF_W_M}"
        "--uav-initial-area-half-h-m=${PILOT_UAV_INITIAL_AREA_HALF_H_M}"
        "--uav-area-half-w-m=${PILOT_UAV_AREA_HALF_W_M}"
        "--uav-area-half-h-m=${PILOT_UAV_AREA_HALF_H_M}"
        "--uav-mission-target-scale=1"
        "--closed-loop-uav-targeting=0"
        "--uav-speed-mps=${PILOT_UAV_SPEED_MPS}"
        "--mobility-update-ms=${PILOT_MOBILITY_UPDATE_MS}"
        "--xhaul-pathloss-exponent=${PILOT_XHAUL_PATHLOSS_EXPONENT}"
        "--xhaul-reference-distance-m=100"
        "--xhaul-healthy-rsrp-dbm=-100"
        "--enable-xhaul-channel-variation=1"
        "--xhaul-shadowing-stddev-db=6"
        "--xhaul-fading-stddev-db=4"
        "--xhaul-shadowing-stream=7301"
        "--xhaul-fading-stream=7302"
        "--xhaul-degradation-start=30"
        "--xhaul-degradation-stop=75"
        "--xhaul-degradation-penalty-db=35"
        "--xhaul-degradation-ramp-s=10"
        "--xhaul-donor-unavailable-start=-1"
        "--xhaul-donor-unavailable-stop=-1"
        "--uav-outage-predictor-threshold=0.5"
        "--uav-outage-predictor-consecutive=2"
        "--uav-outage-predictor-min-dwell-s=6"
        "--uav-outage-predictor-max-feature-gap-s=3"
        "--xhaul-route-update-delay-s=0.05"
    )
}

pilot_policy_args() {
    local policy=$1
    case "${policy}" in
        rule)
            PILOT_POLICY_ARGS=(
                "--deployment-mode=tn-uav-satellite"
                "--enable-sat-backhaul-monitor=1"
                "--enable-uav-switching-xapp=1"
                "--enable-ai-uav-switching-xapp=0"
                "--uav-outage-predictor-mode=off"
                "--xhaul-switch-to-sat-ttt-s=5"
                "--xhaul-switch-to-tn-ttt-s=5"
                "--backhaul-aware-handover=1"
            )
            ;;
        predictor)
            PILOT_POLICY_ARGS=(
                "--deployment-mode=tn-uav-satellite"
                "--enable-sat-backhaul-monitor=1"
                "--enable-uav-switching-xapp=1"
                "--enable-ai-uav-switching-xapp=0"
                "--uav-outage-predictor-mode=active"
                "--uav-outage-predictor-model=${PILOT_MODEL}"
                "--uav-outage-predictor-model-sha256=${PILOT_MODEL_SHA256}"
                "--xhaul-switch-to-sat-ttt-s=5"
                "--xhaul-switch-to-tn-ttt-s=5"
                "--backhaul-aware-handover=1"
            )
            ;;
        predictor-shadow)
            PILOT_POLICY_ARGS=(
                "--deployment-mode=tn-uav-satellite"
                "--enable-sat-backhaul-monitor=1"
                "--enable-uav-switching-xapp=1"
                "--enable-ai-uav-switching-xapp=0"
                "--uav-outage-predictor-mode=shadow"
                "--uav-outage-predictor-model=${PILOT_MODEL}"
                "--uav-outage-predictor-model-sha256=${PILOT_MODEL_SHA256}"
                "--xhaul-switch-to-sat-ttt-s=5"
                "--xhaul-switch-to-tn-ttt-s=5"
                "--backhaul-aware-handover=1"
            )
            ;;
        fast-rule)
            PILOT_POLICY_ARGS=(
                "--deployment-mode=tn-uav-satellite"
                "--enable-sat-backhaul-monitor=1"
                "--enable-uav-switching-xapp=1"
                "--enable-ai-uav-switching-xapp=0"
                "--uav-outage-predictor-mode=off"
                "--xhaul-switch-to-sat-ttt-s=0"
                "--xhaul-switch-to-tn-ttt-s=5"
                "--backhaul-aware-handover=1"
            )
            ;;
        no-satellite)
            PILOT_POLICY_ARGS=(
                "--deployment-mode=tn-uav"
                "--enable-sat-backhaul-monitor=0"
                "--enable-uav-switching-xapp=1"
                "--enable-ai-uav-switching-xapp=0"
                "--uav-outage-predictor-mode=off"
                "--xhaul-switch-to-sat-ttt-s=5"
                "--xhaul-switch-to-tn-ttt-s=5"
                "--backhaul-aware-handover=1"
            )
            ;;
        *) pilot_die "unknown policy '${policy}'" ;;
    esac
}

pilot_result_dir() {
    local output_root=$1
    local run_label=$2
    local policy=$3
    local deployment_mode=tn-uav-satellite
    if [[ "${policy}" == "no-satellite" ]]; then
        deployment_mode=tn-uav
    fi
    printf '%s/raw/%s_ueS1_%s_ueS2_%s_tnGnb_%s_ntnGnb_%s_tnCap_%s_ntnCap_%s_hyst_2_%s\n' \
        "${output_root}" "${deployment_mode}" \
        "${PILOT_MONITORED_UES}" "${PILOT_BACKGROUND_UES}" \
        "${PILOT_TN_GNBS}" "${PILOT_UAV_GNBS}" \
        "${PILOT_MAX_UES_PER_TN_GNB}" "${PILOT_MAX_UES_PER_UAV_GNB}" \
        "${run_label}"
}

pilot_print_command() {
    local policy=$1
    local rng_run=$2
    local output_root=$3
    local scope=${4:-campaign}
    local sim_time_s=${5:-120}
    local executable=${PILOT_EXECUTABLE:-${PILOT_EXECUTABLE_DEFAULT}}
    local run_label="reviewer_${scope}_${policy}_run${rng_run}"
    local db_path="${output_root}/meta/${run_label}/oran-repository.db"
    local arg
    pilot_common_args "${rng_run}" "${sim_time_s}"
    pilot_policy_args "${policy}"
    printf '%q ' "${executable}"
    for arg in "${PILOT_COMMON_ARGS[@]}" "${PILOT_POLICY_ARGS[@]}" \
        "--run-label=${run_label}" "--output-parent-dir=${output_root}/raw" "--db-file=${db_path}"; do
        printf '%q ' "${arg}"
    done
    printf '\n'
}

run_reviewer_arm() {
    local policy=$1
    local rng_run=$2
    local output_root=$3
    local scope=${4:-campaign}
    local sim_time_s=${5:-120}
    local executable=${PILOT_EXECUTABLE:-${PILOT_EXECUTABLE_DEFAULT}}
    local metadata_python
    local run_label="reviewer_${scope}_${policy}_run${rng_run}"
    local meta_dir="${output_root}/meta/${run_label}"
    local result_dir
    local db_path="${meta_dir}/oran-repository.db"
    local start_epoch end_epoch exit_code=0
    local -a command_args

    metadata_python=$(pilot_python)
    result_dir=$(pilot_result_dir "${output_root}" "${run_label}" "${policy}")
    pilot_assert_frozen_inputs "${executable}"
    [[ "${output_root}" = /* ]] || pilot_die "output root must be absolute: ${output_root}"
    [[ ! -e "${meta_dir}" ]] || pilot_die "refusing to overwrite metadata: ${meta_dir}"
    [[ ! -e "${result_dir}" ]] || pilot_die "refusing to overwrite raw result: ${result_dir}"

    mkdir -p -- "${output_root}/meta" "${output_root}/raw"
    mkdir -- "${meta_dir}"
    pilot_common_args "${rng_run}" "${sim_time_s}"
    pilot_policy_args "${policy}"
    command_args=(
        "${PILOT_COMMON_ARGS[@]}"
        "${PILOT_POLICY_ARGS[@]}"
        "--run-label=${run_label}"
        "--output-parent-dir=${output_root}/raw"
        "--db-file=${db_path}"
    )

    printf '%s\0' "${PILOT_COMMON_ARGS[@]}" > "${meta_dir}/matched-config.argv0"
    printf '%s\0' "${PILOT_POLICY_ARGS[@]}" > "${meta_dir}/policy-config.argv0"
    printf '%s\0' "${command_args[@]}" > "${meta_dir}/full-config.argv0"
    sha256sum -- "${meta_dir}/matched-config.argv0" > "${meta_dir}/matched-config.sha256"
    sha256sum -- "${meta_dir}/policy-config.argv0" > "${meta_dir}/policy-config.sha256"
    sha256sum -- "${meta_dir}/full-config.argv0" > "${meta_dir}/full-config.sha256"
    {
        printf '%q ' "${executable}"
        printf '%q ' "${command_args[@]}"
        printf '\n'
    } > "${meta_dir}/command.sh.txt"

    git -C "${PILOT_REPO_ROOT}" status --porcelain=v1 --untracked-files=all \
        > "${meta_dir}/git-status-porcelain.txt"
    git -C "${PILOT_REPO_ROOT}" diff --binary --no-ext-diff HEAD -- \
        > "${meta_dir}/git-tracked-code.diff"
    cp -- "${PILOT_CMAKE_CACHE}" "${meta_dir}/cmake-cache.txt"
    {
        printf 'captured_utc=%s\n' "$(date --utc --iso-8601=seconds)"
        printf 'uname='; uname -a
        printf '\ncompiler_version:\n'; c++ --version
        printf '\ncmake_version:\n'; cmake --version
        printf '\nexecutable_file:\n'; file -- "${executable}"
        printf '\nloaded_libraries:\n'; ldd "${executable}"
        if command -v readelf >/dev/null; then
            printf '\nelf_build_id:\n'; readelf -n "${executable}"
        fi
        printf '\nselected_cmake_cache_entries:\n'
        grep -E '^(CMAKE_BUILD_TYPE|CMAKE_CXX_COMPILER|CMAKE_CXX_FLAGS|NS3_)' \
            "${PILOT_CMAKE_CACHE}" || true
    } > "${meta_dir}/build-provenance.txt" 2>&1
    sha256sum -- \
        "${executable}" \
        "${PILOT_NR_LIBRARY}" \
        "${PILOT_EXAMPLE}" \
        "${PILOT_HELPER}" \
        "${PILOT_MODEL}" \
        "${PILOT_SCHEMA}" \
        "${PILOT_RUNNER}" \
        "${PILOT_VALIDATOR}" \
        "${PILOT_CAMPAIGN_RUNNER}" \
        "${PILOT_ROUTE_CONTRACT_REPORT}" \
        "${PILOT_ROUTE_CONTRACT_MANIFEST}" \
        "${PILOT_CMAKE_CACHE}" \
        "${meta_dir}/matched-config.argv0" \
        "${meta_dir}/policy-config.argv0" \
        "${meta_dir}/full-config.argv0" \
        "${meta_dir}/command.sh.txt" \
        "${meta_dir}/git-status-porcelain.txt" \
        "${meta_dir}/git-tracked-code.diff" \
        "${meta_dir}/cmake-cache.txt" \
        "${meta_dir}/build-provenance.txt" \
        > "${meta_dir}/frozen-inputs.sha256"

    "${metadata_python}" - "${meta_dir}/run.json" "${policy}" "${rng_run}" \
        "${scope}" "${PILOT_REPO_ROOT}" "${executable}" "${PILOT_EXAMPLE}" \
        "${PILOT_HELPER}" "${PILOT_NR_LIBRARY}" "${PILOT_MODEL}" "${PILOT_SCHEMA}" \
        "${PILOT_RUNNER}" "${PILOT_VALIDATOR}" "${PILOT_CAMPAIGN_RUNNER}" \
        "${PILOT_ROUTE_CONTRACT_REPORT}" "${PILOT_ROUTE_CONTRACT_MANIFEST}" \
        "${PILOT_CMAKE_CACHE}" "${result_dir}" "${db_path}" "${run_label}" \
        "$(cut -d' ' -f1 "${meta_dir}/matched-config.sha256")" \
        "$(cut -d' ' -f1 "${meta_dir}/policy-config.sha256")" \
        "$(cut -d' ' -f1 "${meta_dir}/full-config.sha256")" "${meta_dir}" \
        "${PILOT_EXPECTED_UES}" "${PILOT_MONITORED_UES}" "${PILOT_BACKGROUND_UES}" \
        "${PILOT_TN_GNBS}" "${PILOT_UAV_GNBS}" \
        "${PILOT_MAX_UES_PER_TN_GNB}" "${PILOT_MAX_UES_PER_UAV_GNB}" \
        "${PILOT_TN_TOTAL_CAPACITY}" "${PILOT_UAV_TOTAL_CAPACITY}" \
        "${PILOT_GROUND_ATTACH_DELAY_S}" "${PILOT_INITIAL_ATTACH_STAGGER_S}" \
        "${PILOT_CAPACITY_STRATIFIED_UE_PLACEMENT}" \
        "${PILOT_UE_ANCHOR_CLUSTER_RADIUS_M}" "${PILOT_UE_AREA_HALF_W_M}" \
        "${PILOT_UE_AREA_HALF_H_M}" "${PILOT_UAV_INITIAL_AREA_HALF_W_M}" \
        "${PILOT_UAV_INITIAL_AREA_HALF_H_M}" "${PILOT_UAV_AREA_HALF_W_M}" \
        "${PILOT_UAV_AREA_HALF_H_M}" "${PILOT_UAV_SPEED_MPS}" \
        "${PILOT_MOBILITY_UPDATE_MS}" "${PILOT_XHAUL_PATHLOSS_EXPONENT}" \
        "${PILOT_EXPECTED_MONITORED_TN_UES}" \
        "${PILOT_EXPECTED_MONITORED_UAV_UES}" \
        "${PILOT_MONITORED_DL_START_S}" "${PILOT_MONITORED_UL_START_S}" \
        "${sim_time_s}" <<'PY'
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

(out, policy, rng_run, scope, repo, executable, source, helper, nr_library,
 model, schema, runner, validator, campaign_runner, route_report,
 route_manifest, cmake_cache, result_dir, db_path, run_label, matched_hash,
 policy_hash, full_hash, meta_dir, expected_ues, monitored_ues, background_ues,
 tn_gnbs, uav_gnbs, max_ues_tn, max_ues_uav, tn_total_capacity,
 uav_total_capacity, ground_attach_delay_s, initial_attach_stagger_s,
 capacity_stratified_ue_placement, ue_cluster_radius_m, ue_area_half_w_m,
 ue_area_half_h_m,
 uav_initial_area_half_w_m, uav_initial_area_half_h_m, uav_area_half_w_m,
 uav_area_half_h_m, uav_speed_mps, mobility_update_ms, xhaul_pathloss_exponent,
 expected_monitored_tn_ues, expected_monitored_uav_ues, monitored_dl_start_s,
 monitored_ul_start_s, sim_time_s) = sys.argv[1:]

def sha256(filename):
    return hashlib.sha256(Path(filename).read_bytes()).hexdigest()

def git(*args):
    return subprocess.check_output(["git", "-C", repo, *args], text=True).strip()

meta = Path(meta_dir)
route_contract = json.loads(Path(route_report).read_text(encoding="utf-8"))
route_contract_checks = route_contract.get("checks", {})
payload = {
    "protocol_version": "uav-reviewer-v8",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "policy": policy,
    "scope": scope,
    "rng_seed": 1,
    "rng_run": int(rng_run),
    "run_label": run_label,
    "repo_root": repo,
    "git_head": git("rev-parse", "HEAD"),
    "git_status_porcelain": (meta / "git-status-porcelain.txt").read_text(
        encoding="utf-8"
    ),
    "source_path": source,
    "source_sha256": sha256(source),
    "helper_path": helper,
    "helper_sha256": sha256(helper),
    "binary_path": executable,
    "binary_sha256": sha256(executable),
    "nr_library_path": nr_library,
    "nr_library_sha256": sha256(nr_library),
    "loaded_nr_library_verified": True,
    "model_path": model,
    "model_sha256": sha256(model),
    "schema_path": schema,
    "schema_sha256": sha256(schema),
    "runner_path": runner,
    "runner_sha256": sha256(runner),
    "validator_path": validator,
    "validator_sha256": sha256(validator),
    "campaign_runner_path": campaign_runner,
    "campaign_runner_sha256": sha256(campaign_runner),
    "route_contract_report_path": route_report,
    "route_contract_report_sha256": sha256(route_report),
    "route_contract_manifest_path": route_manifest,
    "route_contract_manifest_sha256": sha256(route_manifest),
    "route_contract_passed": bool(route_contract.get("passed")),
    "route_contract_all_checks_passed": bool(route_contract_checks)
    and all(route_contract_checks.values()),
    "route_contract_checks": route_contract_checks,
    "route_contract_identity": "route-contract-v20",
    "cmake_cache_path": cmake_cache,
    "cmake_cache_sha256": sha256(cmake_cache),
    "cmake_cache_archive_path": str((meta / "cmake-cache.txt").resolve()),
    "cmake_cache_archive_sha256": sha256(meta / "cmake-cache.txt"),
    "git_diff_path": str((meta / "git-tracked-code.diff").resolve()),
    "git_diff_sha256": sha256(meta / "git-tracked-code.diff"),
    "git_status_path": str((meta / "git-status-porcelain.txt").resolve()),
    "git_status_sha256": sha256(meta / "git-status-porcelain.txt"),
    "build_provenance_path": str((meta / "build-provenance.txt").resolve()),
    "build_provenance_sha256": sha256(meta / "build-provenance.txt"),
    "frozen_inputs_manifest_path": str((meta / "frozen-inputs.sha256").resolve()),
    "frozen_inputs_manifest_sha256": sha256(meta / "frozen-inputs.sha256"),
    "matched_config_path": str((meta / "matched-config.argv0").resolve()),
    "matched_config_sha256": matched_hash,
    "policy_config_path": str((meta / "policy-config.argv0").resolve()),
    "policy_config_sha256": policy_hash,
    "full_config_path": str((meta / "full-config.argv0").resolve()),
    "full_config_sha256": full_hash,
    "result_dir": result_dir,
    "database_path": db_path,
    "sim_time_s": float(sim_time_s),
    "drain_time_s": 2.0,
    "degradation_start_s": 30.0,
    "degradation_stop_s": 75.0,
    "degradation_penalty_db": 35.0,
    "degradation_ramp_s": 10.0,
    "xhaul_shadowing_stream": 7301,
    "xhaul_fading_stream": 7302,
    "radio_contract": {
        "rlc_mode": "um",
        "harq_retransmissions_enabled": True,
        "dedicated_qos_flows_enabled": True,
        "use_rsrp_lm": False,
        "fixed_initial_association": True,
    },
    "fixed_initial_association": True,
    "mobility_lm": "noop",
    "initial_attach_stagger_s": float(initial_attach_stagger_s),
    "monitored_dl_start_s": float(monitored_dl_start_s),
    "monitored_ul_start_s": float(monitored_ul_start_s),
    "mobility_update_ms": float(mobility_update_ms),
    "ue_mobility_contract": {
        "description": "stationary UAV anchors with bounded local UE mobility",
        "monitored_ue_speed_mps": 5.0,
        "background_ue_speed_mps": 4.0,
        "uav_speed_mps": float(uav_speed_mps),
        "update_interval_ms": float(mobility_update_ms),
        "cluster_half_width_m": float(ue_cluster_radius_m),
        "cluster_half_height_m": float(ue_cluster_radius_m),
    },
    "ue_placement": {
        "mode": "capacity-stratified-clustered",
        "capacity_stratified": bool(int(capacity_stratified_ue_placement)),
        "cluster_radius_m": float(ue_cluster_radius_m),
        "global_area_half_w_m": float(ue_area_half_w_m),
        "global_area_half_h_m": float(ue_area_half_h_m),
    },
    "frozen_design": {
        "monitored_ues": int(monitored_ues),
        "background_ues": int(background_ues),
        "tn_gnbs": int(tn_gnbs),
        "uav_gnbs": int(uav_gnbs),
        "max_ues_per_tn_gnb": int(max_ues_tn),
        "max_ues_per_uav_gnb": int(max_ues_uav),
        "tn_total_capacity": int(tn_total_capacity),
        "uav_total_capacity": int(uav_total_capacity),
        "expected_monitored_tn_ues": int(expected_monitored_tn_ues),
        "expected_monitored_uav_ues": int(expected_monitored_uav_ues),
        "ground_attach_delay_s": float(ground_attach_delay_s),
        "initial_attach_stagger_s": float(initial_attach_stagger_s),
        "monitored_dl_start_s": float(monitored_dl_start_s),
        "monitored_ul_start_s": float(monitored_ul_start_s),
        "deterministic_access_anchors": True,
        "explicit_initial_anchor_attachment": True,
        "ue_placement_mode": "capacity-stratified-clustered",
        "capacity_stratified_ue_placement": bool(
            int(capacity_stratified_ue_placement)
        ),
        "ue_cluster_radius_m": float(ue_cluster_radius_m),
        "ue_area_half_w_m": float(ue_area_half_w_m),
        "ue_area_half_h_m": float(ue_area_half_h_m),
        "uav_initial_area_half_w_m": float(uav_initial_area_half_w_m),
        "uav_initial_area_half_h_m": float(uav_initial_area_half_h_m),
        "uav_area_half_w_m": float(uav_area_half_w_m),
        "uav_area_half_h_m": float(uav_area_half_h_m),
        "uav_speed_mps": float(uav_speed_mps),
        "mobility_update_ms": float(mobility_update_ms),
        "monitored_ue_speed_mps": 5.0,
        "background_ue_speed_mps": 4.0,
        "access_mobility_description": "stationary UAV anchors with bounded local UE mobility",
        "xhaul_pathloss_exponent": float(xhaul_pathloss_exponent),
        "capacity_stratified_exposure": True,
    },
    "evidence_scope": "backhaul switching and monitored-UE QoS under fixed initial association",
    "claim_scope": {
        "backhaul_switching": "in_scope",
        "monitored_ue_qos": "in_scope",
        "handover_success": "not-applicable",
        "ue_pingpong": "not-applicable",
    },
    "deployment_mode": "tn-uav" if policy == "no-satellite" else "tn-uav-satellite",
    "expected_ues": int(expected_ues),
    "monitored_ues": int(monitored_ues),
    "monitored_packet_size_bytes": 1000,
    "service_thresholds": {
        "dl_min_rate_mbps": 0.18,
        "ul_min_rate_mbps": 0.045,
        "max_delay_s": 0.30,
        "min_pdr": 0.90
    },
}
Path(out).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
PY

    start_epoch=$(date +%s)
    printf '%s\n' "$(date --utc --iso-8601=seconds)" > "${meta_dir}/started-utc.txt"
    set +e
    (
        cd "${PILOT_REPO_ROOT}"
        OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/time -v \
            -o "${meta_dir}/time.txt" \
            "${executable}" "${command_args[@]}" \
            > "${meta_dir}/stdout.log" 2> "${meta_dir}/stderr.log"
    )
    exit_code=$?
    set -e
    end_epoch=$(date +%s)
    printf '%s\n' "$(date --utc --iso-8601=seconds)" > "${meta_dir}/finished-utc.txt"
    "${metadata_python}" - "${meta_dir}/process-status.json" "${exit_code}" \
        "$((end_epoch - start_epoch))" "${result_dir}" <<'PY'
import json, sys
from pathlib import Path
out, code, wall, result_dir = sys.argv[1:]
payload = {
    "exit_code": int(code),
    "wall_clock_seconds": int(wall),
    "result_dir_exists": Path(result_dir).is_dir(),
}
Path(out).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
PY

    if [[ -d "${result_dir}" ]]; then
        find "${result_dir}" -type f -print0 | sort -z | xargs -0 -r sha256sum \
            > "${meta_dir}/raw-artifacts.sha256"
    else
        : > "${meta_dir}/raw-artifacts.sha256"
    fi
    printf 'policy=%s RngRun=%s exit=%s wall_s=%s metadata=%s\n' \
        "${policy}" "${rng_run}" "${exit_code}" "$((end_epoch - start_epoch))" "${meta_dir}"
    PILOT_LAST_EXIT_CODE=${exit_code}
    return 0
}

pilot_assert_accepted_rule_validation() {
    local validation_report=$1
    local rule_meta=$2
    local validation_python
    validation_python=$(pilot_python)
    [[ -f "${validation_report}" ]] || pilot_die \
        "accepted rule validation is missing: ${validation_report}"
    "${validation_python}" - "${validation_report}" "${rule_meta}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1]).resolve()
expected_meta_dir = Path(sys.argv[2]).resolve()
report = json.loads(report_path.read_text(encoding="utf-8"))
runs = report.get("runs", [])
if report.get("errors"):
    raise SystemExit(f"rule validation contains errors: {report.get('errors')!r}")
if report.get("run_count") != 1 or len(runs) != 1:
    raise SystemExit("rule validation must contain exactly one run")
run = runs[0]
metadata = run.get("metadata", {})
if Path(run.get("metadata_dir", "")).resolve() != expected_meta_dir:
    raise SystemExit("rule validation metadata directory is not the required rule arm")
expected_identity = {
    "policy": "rule",
    "scope": "pilot",
    "rng_seed": 1,
    "rng_run": 6,
}
observed_identity = {key: metadata.get(key) for key in expected_identity}
if observed_identity != expected_identity:
    raise SystemExit(
        f"rule validation identity mismatch: expected={expected_identity!r} "
        f"observed={observed_identity!r}"
    )
if run.get("gates_passed") is not True:
    raise SystemExit("rule run gates did not pass")

# A one-arm report deliberately fails the pair-level complete_policy_group gate,
# so report.overall_passed is not used here.  Campaign execution still requires
# the later two-arm pilot report to pass overall.
print(f"rule_validation_acceptance=PASS json={report_path}")
PY
}

pilot_main() {
    local stage=${1:-${PILOT_STAGE:-rule-only}}
    local output_root
    if [[ "${stage}" == "smoke-rule" ]]; then
        output_root=${PILOT_SMOKE_OUTPUT_ROOT:-${PILOT_REPO_ROOT}/results/nr/tn-ntn/reviewer-smoke-v8}
    else
        output_root=${PILOT_OUTPUT_ROOT:-${PILOT_REPO_ROOT}/results/nr/tn-ntn/reviewer-pilot-v8}
    fi
    local validation_python
    local rule_meta="${output_root}/meta/reviewer_pilot_rule_run6"
    local predictor_meta="${output_root}/meta/reviewer_pilot_predictor_run6"
    local smoke_meta="${output_root}/meta/reviewer_smoke_rule_run6"
    local rule_validation="${output_root}/rule-validation.json"
    local pilot_validation="${output_root}/pilot-validation.json"
    local smoke_validation="${output_root}/smoke-validation.json"
    local validator_status=0
    local overall=0

    if [[ "${DRY_RUN:-0}" == "1" ]]; then
        case "${stage}" in
            smoke-rule) pilot_print_command rule 6 "${output_root}" smoke 22 ;;
            rule-only) pilot_print_command rule 6 "${output_root}" pilot ;;
            predictor-only) pilot_print_command predictor 6 "${output_root}" pilot ;;
            shadow-only) pilot_print_command predictor-shadow 6 "${output_root}" pilot ;;
            pair)
                pilot_print_command rule 6 "${output_root}" pilot
                pilot_print_command predictor 6 "${output_root}" pilot
                ;;
            *) pilot_die "unknown pilot stage '${stage}'" ;;
        esac
        return 0
    fi

    case "${stage}" in
        smoke-rule)
            [[ ! -e "${output_root}" ]] || pilot_die \
                "smoke stage requires a fresh output root; refusing to reuse ${output_root}"
            run_reviewer_arm rule 6 "${output_root}" smoke 22
            overall=${PILOT_LAST_EXIT_CODE}
            if [[ "${overall}" -ne 0 ]]; then
                return "${overall}"
            fi
            validation_python=$(pilot_python)
            "${validation_python}" "${PILOT_VALIDATOR}" \
                --metadata-dir "${smoke_meta}" \
                --output "${smoke_validation}" || overall=$?
            if [[ "${overall}" -eq 0 ]]; then
                printf 'Smoke validation PASS: %s\n' "${smoke_validation}"
            fi
            return "${overall}"
            ;;
        rule-only)
            [[ ! -e "${rule_validation}" ]] || pilot_die \
                "refusing to overwrite rule validation: ${rule_validation}"
            run_reviewer_arm rule 6 "${output_root}" pilot
            overall=${PILOT_LAST_EXIT_CODE}
            if [[ "${overall}" -ne 0 ]]; then
                return "${overall}"
            fi
            validation_python=$(pilot_python)
            set +e
            "${validation_python}" "${PILOT_VALIDATOR}" \
                --metadata-dir "${rule_meta}" \
                --output "${rule_validation}"
            validator_status=$?
            set -e
            if [[ "${validator_status}" -ne 0 && "${validator_status}" -ne 1 ]]; then
                pilot_die "single-arm validator failed with exit ${validator_status}"
            fi
            pilot_assert_accepted_rule_validation "${rule_validation}" "${rule_meta}"
            printf 'Single-arm validator exit=%s; accepted runs[0].gates_passed while the pair design gate is incomplete.\n' \
                "${validator_status}"
            printf 'Rule-only timing gate complete. Inspect its route/exposure gates before running:\n'
            printf '  PILOT_OUTPUT_ROOT=%q %q predictor-only\n' \
                "${output_root}" "${PILOT_RUNNER}"
            return 0
            ;;
        predictor-only)
            pilot_assert_accepted_rule_validation "${rule_validation}" "${rule_meta}"
            [[ ! -e "${pilot_validation}" ]] || pilot_die \
                "refusing to overwrite pilot validation: ${pilot_validation}"
            run_reviewer_arm predictor 6 "${output_root}" pilot
            overall=${PILOT_LAST_EXIT_CODE}
            ;;
        shadow-only)
            pilot_assert_accepted_rule_validation "${rule_validation}" "${rule_meta}"
            run_reviewer_arm predictor-shadow 6 "${output_root}" pilot
            return "${PILOT_LAST_EXIT_CODE}"
            ;;
        pair)
            [[ ! -e "${pilot_validation}" ]] || pilot_die \
                "refusing to overwrite pilot validation: ${pilot_validation}"
            run_reviewer_arm rule 6 "${output_root}" pilot
            if [[ "${PILOT_LAST_EXIT_CODE}" -ne 0 ]]; then
                return "${PILOT_LAST_EXIT_CODE}"
            fi
            run_reviewer_arm predictor 6 "${output_root}" pilot
            if [[ "${PILOT_LAST_EXIT_CODE}" -ne 0 ]]; then
                overall=${PILOT_LAST_EXIT_CODE}
            fi
            ;;
        *) pilot_die "unknown pilot stage '${stage}'" ;;
    esac

    if [[ "${VALIDATE_AFTER_RUN:-1}" == "1" && "${overall}" -eq 0 ]]; then
        validation_python=$(pilot_python)
        "${validation_python}" "${PILOT_VALIDATOR}" \
            --metadata-dir "${rule_meta}" \
            --metadata-dir "${predictor_meta}" \
            --output "${pilot_validation}" || overall=$?
    fi
    return "${overall}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    pilot_main "$@"
fi

