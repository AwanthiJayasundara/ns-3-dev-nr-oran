#!/usr/bin/env bash
# Dedicated Gate-2 integration harness for O-RAN RSRP UE mobility.  It is
# intentionally separate from the fixed-association reviewer-v7 backhaul run.
# This script does not train or deploy an ONNX mobility model.

set -euo pipefail

MOBILITY_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly MOBILITY_SCRIPT_DIR
MOBILITY_REPO_ROOT="$(cd "${MOBILITY_SCRIPT_DIR}/../../.." && pwd -P)"
readonly MOBILITY_REPO_ROOT
readonly MOBILITY_SOURCE="${MOBILITY_SCRIPT_DIR}/oran-nr-uav-xhaul-autonomy-example.cc"
readonly MOBILITY_RSRP_LM_SOURCE="${MOBILITY_REPO_ROOT}/contrib/oran/model/oran-lm-nr-2-nr-rsrp-handover-with-tn-ntn.cc"
readonly MOBILITY_HELPER="${MOBILITY_REPO_ROOT}/src/nr/helper/nr-hybrid-sat-epc-helper.cc"
readonly MOBILITY_RUNNER="${MOBILITY_SCRIPT_DIR}/run-uav-reviewer-mobility.sh"
readonly MOBILITY_VALIDATOR="${MOBILITY_SCRIPT_DIR}/validate-uav-reviewer-mobility.py"
readonly MOBILITY_MANIFEST="${MOBILITY_REPO_ROOT}/audit/reviewer_evidence/mobility_corridors_v2.csv"
readonly MOBILITY_STREAM_MAP="${MOBILITY_REPO_ROOT}/audit/reviewer_evidence/mobility_stream_map_v2.json"
readonly MOBILITY_MANIFEST_SHA256="7ad1a91c68c14dd50d597f2149f402bc480fac1de27c734f893029731d41d25d"
readonly MOBILITY_STREAM_MAP_SHA256="f70e6f3429fffb2c2b4e98d82395fc6c6afc3d5083ef1e43855dd86a9d47f868"
readonly MOBILITY_EXECUTABLE_DEFAULT="${MOBILITY_REPO_ROOT}/build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized"
readonly MOBILITY_NR_LIBRARY="${MOBILITY_REPO_ROOT}/build/lib/libns3-dev-nr-optimized.so"
readonly MOBILITY_ORAN_LIBRARY="${MOBILITY_REPO_ROOT}/build/lib/libns3-dev-oran-optimized.so"
readonly MOBILITY_CMAKE_CACHE="${MOBILITY_REPO_ROOT}/cmake-cache/CMakeCache.txt"
readonly MOBILITY_OUTPUT_ROOT_DEFAULT="${MOBILITY_REPO_ROOT}/results/nr/tn-ntn/reviewer-mobility-integration-v2"

mobility_die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 2
}

mobility_sha256() {
    sha256sum -- "$1" | awk '{print $1}'
}

mobility_python() {
    local candidate
    for candidate in \
        "${MOBILITY_PYTHON:-}" \
        "${MOBILITY_REPO_ROOT}/../ns-3-dev-nr-oran/.venv-uav-ai/bin/python" \
        "${MOBILITY_REPO_ROOT}/.venv-uav-ai/bin/python" \
        "${MOBILITY_REPO_ROOT}/.venv/bin/python" \
        /usr/bin/python3; do
        if [[ -n "${candidate}" && -x "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done
    mobility_die "no Python 3 interpreter found"
}

mobility_assert_inputs() {
    local executable=$1
    local required
    for required in \
        "${MOBILITY_SOURCE}" \
        "${MOBILITY_RSRP_LM_SOURCE}" \
        "${MOBILITY_HELPER}" \
        "${MOBILITY_RUNNER}" \
        "${MOBILITY_VALIDATOR}" \
        "${MOBILITY_MANIFEST}" \
        "${MOBILITY_STREAM_MAP}" \
        "${MOBILITY_NR_LIBRARY}" \
        "${MOBILITY_ORAN_LIBRARY}" \
        "${MOBILITY_CMAKE_CACHE}"; do
        [[ -f "${required}" ]] || mobility_die "missing required input: ${required}"
    done
    [[ -x "${executable}" ]] || mobility_die "missing optimized executable: ${executable}"
    [[ "$(mobility_sha256 "${MOBILITY_MANIFEST}")" == "${MOBILITY_MANIFEST_SHA256}" ]] || \
        mobility_die "mobility manifest SHA-256 differs from the registered Gate-2 input"
    [[ "$(mobility_sha256 "${MOBILITY_STREAM_MAP}")" == "${MOBILITY_STREAM_MAP_SHA256}" ]] || \
        mobility_die "stream-map SHA-256 differs from the registered Gate-2 input"
    [[ "${executable}" -nt "${MOBILITY_SOURCE}" ]] || mobility_die \
        "executable predates the mobility source; build the single example target first"
    local dependencies
    dependencies=$(ldd "${executable}")
    for required in "${MOBILITY_NR_LIBRARY}" "${MOBILITY_ORAN_LIBRARY}"; do
        [[ "${dependencies}" == *"=> ${required} "* ]] || mobility_die \
            "executable is not loading the expected shared library: ${required}"
    done
    command -v /usr/bin/time >/dev/null || mobility_die "/usr/bin/time is required"
}

mobility_common_args() {
    local rsrp_type=ns3::OranLmNr2NrRsrpHandoverWithTnNtn
    MOBILITY_ARGS=(
        "--sim-time=30"
        "--stop-tail=2"
        "--RngSeed=1"
        "--RngRun=6"
        "--num-uess1=50"
        "--num-ground-ues=50"
        "--num-tn-gnbs=4"
        "--num-ntn-gnbs=3"
        "--max-ues-tn=20"
        "--max-ues-ntn=30"
        "--capacity-stratified-ue-placement=1"
        "--monitored-initial-tn-ues=40"
        "--ue-anchor-cluster-radius-m=100"
        "--ue-mobility-mode=manifest"
        "--ue-mobility-manifest=${MOBILITY_MANIFEST}"
        "--freeze-uav-access-positions=1"
        "--mobility-update-ms=200"
        "--ground-attach-delay=5"
        "--initial-attach-stagger-s=0.25"
        "--use-oran=1"
        "--use-rsrp-lm=1"
        "--use-onnx-lm=0"
        "--use-torch-lm=0"
        "--handover-algorithm=ns3::NrNoOpHandoverAlgorithm"
        "--lm-query-interval=2"
        "--e2-send-interval=2"
        "--tx-delay=0.1"
        "--hysteresis-db=2"
        "--handover-min-rsrp-dbm=-110"
        "--${rsrp_type}::Warmup=2s"
        "--${rsrp_type}::MinHoInterval=2s"
        "--${rsrp_type}::HoAttemptTimeout=2s"
        "--${rsrp_type}::TimeToTrigger=2s"
        "--${rsrp_type}::LowRsrpRecheck=2s"
        "--${rsrp_type}::TnMinRsrpDbm=-110"
        "--${rsrp_type}::NtnEnterMarginDb=3"
        "--${rsrp_type}::TnReturnMarginDb=5"
        "--rlc-mode=um"
        "--enable-harq-retx=0"
        "--monitored-traffic=udp"
        "--monitored-dl-rate-mbps=0.2"
        "--monitored-ul-rate-mbps=0.05"
        "--monitored-packet-size=1000"
        "--monitored-dl-start-s=6"
        "--monitored-ul-start-s=6.5"
        "--enable-dedicated-qos-flows=1"
        "--enable-startup-ping=0"
        "--enable-fading=1"
        "--channel-update-ms=100"
        "--channel-condition-update-ms=200"
        "--pdcp-discard-timer-ms=1000"
        "--tn-tx-power-dbm=46"
        "--uav-tx-power-dbm=37"
        "--ue-tx-power-dbm=23"
        "--init-min-rsrp=-110"
        "--enable-flow-monitor=1"
        "--enable-rsrp-trace=1"
        "--enable-position-trace=1"
        "--position-trace-interval=1"
        "--enable-handover-trace=1"
        "--enable-handover-failure-trace=1"
        "--enable-decision-csv=1"
        "--enable-oran-info-log=1"
        "--deployment-mode=tn-uav-satellite"
        "--enable-sat-backhaul-monitor=1"
        "--enable-uav-switching-xapp=0"
        "--enable-ai-uav-switching-xapp=0"
        "--uav-outage-predictor-mode=off"
        "--enable-xhaul-channel-variation=0"
        "--xhaul-pathloss-exponent=2"
        "--xhaul-reference-distance-m=100"
        "--xhaul-tx-power-dbm=46"
        "--xhaul-healthy-rsrp-dbm=-100"
        "--xhaul-degradation-start=-1"
        "--xhaul-donor-unavailable-start=-1"
        "--backhaul-aware-handover=0"
        "--closed-loop-uav-targeting=0"
        "--uav-initial-area-half-w-m=1000"
        "--uav-initial-area-half-h-m=800"
        "--uav-area-half-w-m=3000"
        "--uav-area-half-h-m=1500"
        "--xhaul-shadowing-stream=7301"
        "--xhaul-fading-stream=7302"
    )
}

mobility_result_dir() {
    local output_root=$1
    local run_label=$2
    printf '%s/raw/tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_30_hyst_2_%s\n' \
        "${output_root}" "${run_label}"
}

mobility_print_command() {
    local executable=${MOBILITY_EXECUTABLE:-${MOBILITY_EXECUTABLE_DEFAULT}}
    local output_root=${MOBILITY_OUTPUT_ROOT:-${MOBILITY_OUTPUT_ROOT_DEFAULT}}
    local run_label=reviewer_mobility_integration_rule_run6
    local meta_dir="${output_root}/meta/${run_label}"
    local arg
    mobility_common_args
    printf '%q ' "${executable}"
    for arg in "${MOBILITY_ARGS[@]}" \
        "--run-label=${run_label}" \
        "--output-parent-dir=${output_root}/raw" \
        "--db-file=${meta_dir}/oran-repository.db"; do
        printf '%q ' "${arg}"
    done
    printf '\n'
}

mobility_run_gate2() {
    local executable=${MOBILITY_EXECUTABLE:-${MOBILITY_EXECUTABLE_DEFAULT}}
    local output_root=${MOBILITY_OUTPUT_ROOT:-${MOBILITY_OUTPUT_ROOT_DEFAULT}}
    local run_label=reviewer_mobility_integration_rule_run6
    local meta_dir="${output_root}/meta/${run_label}"
    local result_dir
    local db_path="${meta_dir}/oran-repository.db"
    local metadata_python
    local start_epoch end_epoch exit_code=0 validation_code=0
    local -a command_args

    [[ "${output_root}" = /* ]] || mobility_die "output root must be absolute"
    if pgrep -f -- '/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized ' >/dev/null; then
        mobility_die "another UAV autonomy simulation is running; Gate 2 requires isolated execution"
    fi
    mobility_assert_inputs "${executable}"
    metadata_python=$(mobility_python)
    result_dir=$(mobility_result_dir "${output_root}" "${run_label}")
    [[ ! -e "${meta_dir}" ]] || mobility_die "refusing to overwrite metadata: ${meta_dir}"
    [[ ! -e "${result_dir}" ]] || mobility_die "refusing to overwrite raw result: ${result_dir}"

    mkdir -p -- "${output_root}/meta" "${output_root}/raw"
    mkdir -- "${meta_dir}"
    mobility_common_args
    command_args=(
        "${MOBILITY_ARGS[@]}"
        "--run-label=${run_label}"
        "--output-parent-dir=${output_root}/raw"
        "--db-file=${db_path}"
    )
    printf '%s\0' "${MOBILITY_ARGS[@]}" > "${meta_dir}/matched-config.argv0"
    : > "${meta_dir}/policy-config.argv0"
    printf '%s\0' "${command_args[@]}" > "${meta_dir}/full-config.argv0"
    sha256sum -- "${meta_dir}/matched-config.argv0" > "${meta_dir}/matched-config.sha256"
    sha256sum -- "${meta_dir}/policy-config.argv0" > "${meta_dir}/policy-config.sha256"
    sha256sum -- "${meta_dir}/full-config.argv0" > "${meta_dir}/full-config.sha256"
    {
        printf '%q ' "${executable}"
        printf '%q ' "${command_args[@]}"
        printf '\n'
    } > "${meta_dir}/command.sh.txt"

    git -C "${MOBILITY_REPO_ROOT}" status --porcelain=v1 --untracked-files=all \
        > "${meta_dir}/git-status-porcelain.txt"
    git -C "${MOBILITY_REPO_ROOT}" diff --binary --no-ext-diff HEAD -- \
        > "${meta_dir}/git-tracked-code.diff"
    cp -- "${MOBILITY_CMAKE_CACHE}" "${meta_dir}/cmake-cache.txt"
    {
        printf 'captured_utc=%s\n' "$(date --utc --iso-8601=seconds)"
        printf 'uname='; uname -a
        printf '\ncompiler_version:\n'; c++ --version
        printf '\nexecutable_file:\n'; file -- "${executable}"
        printf '\nloaded_libraries:\n'; ldd "${executable}"
    } > "${meta_dir}/build-provenance.txt" 2>&1
    sha256sum -- \
        "${executable}" "${MOBILITY_NR_LIBRARY}" "${MOBILITY_ORAN_LIBRARY}" "${MOBILITY_SOURCE}" \
        "${MOBILITY_RSRP_LM_SOURCE}" "${MOBILITY_HELPER}" "${MOBILITY_RUNNER}" \
        "${MOBILITY_VALIDATOR}" "${MOBILITY_MANIFEST}" "${MOBILITY_STREAM_MAP}" \
        "${MOBILITY_CMAKE_CACHE}" "${meta_dir}/matched-config.argv0" \
        "${meta_dir}/policy-config.argv0" "${meta_dir}/full-config.argv0" \
        "${meta_dir}/command.sh.txt" "${meta_dir}/git-status-porcelain.txt" \
        "${meta_dir}/git-tracked-code.diff" "${meta_dir}/cmake-cache.txt" \
        "${meta_dir}/build-provenance.txt" > "${meta_dir}/frozen-inputs.sha256"

    "${metadata_python}" - "${meta_dir}/run.json" "${MOBILITY_REPO_ROOT}" \
        "${executable}" "${MOBILITY_NR_LIBRARY}" "${MOBILITY_ORAN_LIBRARY}" "${MOBILITY_SOURCE}" \
        "${MOBILITY_RSRP_LM_SOURCE}" "${MOBILITY_HELPER}" "${MOBILITY_RUNNER}" \
        "${MOBILITY_VALIDATOR}" "${MOBILITY_MANIFEST}" "${MOBILITY_STREAM_MAP}" \
        "${MOBILITY_CMAKE_CACHE}" "${result_dir}" "${db_path}" "${meta_dir}" <<'PY'
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

(out, repo, executable, nr_library, oran_library, source, rsrp_lm_source, helper, runner,
 validator, manifest, stream_map, cmake_cache, result_dir, db_path,
 meta_dir) = sys.argv[1:]

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def git(*args):
    return subprocess.check_output(["git", "-C", repo, *args], text=True).strip()

meta = Path(meta_dir)
payload = {
    "protocol_version": "uav-reviewer-mobility-gate2-v2",
    "status": "started; integration only",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "policy": "rsrp-rule",
    "scope": "mobility-integration",
    "rng_seed": 1,
    "rng_run": 6,
    "repo_root": repo,
    "git_head": git("rev-parse", "HEAD"),
    "git_status_porcelain": (meta / "git-status-porcelain.txt").read_text(),
    "binary_path": executable,
    "binary_sha256": sha256(executable),
    "nr_library_path": nr_library,
    "nr_library_sha256": sha256(nr_library),
    "oran_library_path": oran_library,
    "oran_library_sha256": sha256(oran_library),
    "source_path": source,
    "source_sha256": sha256(source),
    "rsrp_lm_source_path": rsrp_lm_source,
    "rsrp_lm_source_sha256": sha256(rsrp_lm_source),
    "route_helper_path": helper,
    "route_helper_sha256": sha256(helper),
    "runner_path": runner,
    "runner_sha256": sha256(runner),
    "validator_path": validator,
    "validator_sha256": sha256(validator),
    "trajectory_manifest_path": manifest,
    "trajectory_manifest_sha256": sha256(manifest),
    "stream_map_path": stream_map,
    "stream_map_sha256": sha256(stream_map),
    "cmake_cache_path": cmake_cache,
    "cmake_cache_sha256": sha256(cmake_cache),
    "result_dir": result_dir,
    "database_path": db_path,
    "matched_config_sha256": (meta / "matched-config.sha256").read_text().split()[0],
    "policy_config_sha256": (meta / "policy-config.sha256").read_text().split()[0],
    "full_config_sha256": (meta / "full-config.sha256").read_text().split()[0],
    "mobility_policy": "OranLmNr2NrRsrpHandoverWithTnNtn",
    "mobility_model_path": None,
    "mobility_model_sha256": None,
    "backhaul_policy": "disabled/frozen healthy",
    "backhaul_model_path": None,
    "backhaul_model_sha256": None,
    "native_handover_algorithm": "ns3::NrNoOpHandoverAlgorithm",
    "sim_time_s": 30.0,
    "drain_time_s": 2.0,
    "population": {"monitored": 50, "background": 50},
    "access_nodes": {"tn": 4, "uav": 3},
    "initial_monitored_exposure": {"tn": 40, "uav": 10},
    "admission_caps": {"tn_per_cell": 20, "uav_per_cell": 30},
    "registered_opportunities": 4,
    "radio_contract": {
        "rlc_mode": "um",
        "harq_retransmissions_enabled": False,
        "fading_enabled": True,
        "dedicated_qos_flows_enabled": True
    },
    "controller_contract": {
        "warmup_s": 2.0,
        "min_ho_interval_s": 2.0,
        "attempt_timeout_s": 2.0,
        "time_to_trigger_s": 2.0,
        "low_rsrp_recheck_s": 2.0,
        "hysteresis_db": 2.0,
        "minimum_target_rsrp_dbm": -110.0,
        "tn_min_rsrp_dbm": -110.0,
        "ntn_enter_margin_db": 3.0,
        "tn_return_margin_db": 5.0
    },
    "claim_scope": "integration/crash and trace reconciliation only; not a manuscript result"
}
Path(out).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
PY

    start_epoch=$(date +%s)
    date --utc --iso-8601=seconds > "${meta_dir}/started-utc.txt"
    set +e
    (
        cd "${MOBILITY_REPO_ROOT}"
        OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/time -v \
            -o "${meta_dir}/time.txt" \
            "${executable}" "${command_args[@]}" \
            > "${meta_dir}/stdout.log" 2> "${meta_dir}/stderr.log"
    )
    exit_code=$?
    set -e
    end_epoch=$(date +%s)
    date --utc --iso-8601=seconds > "${meta_dir}/finished-utc.txt"
    "${metadata_python}" - "${meta_dir}/process-status.json" "${exit_code}" \
        "$((end_epoch - start_epoch))" "${result_dir}" <<'PY'
import json, sys
from pathlib import Path
out, code, wall, result_dir = sys.argv[1:]
Path(out).write_text(json.dumps({
    "exit_code": int(code),
    "wall_clock_seconds": int(wall),
    "result_dir_exists": Path(result_dir).is_dir()
}, indent=2, sort_keys=True) + "\n")
PY
    if [[ -d "${result_dir}" ]]; then
        find "${result_dir}" -type f -print0 | sort -z | xargs -0 -r sha256sum \
            > "${meta_dir}/raw-artifacts.sha256"
    else
        : > "${meta_dir}/raw-artifacts.sha256"
    fi

    set +e
    "${metadata_python}" "${MOBILITY_VALIDATOR}" \
        --metadata-dir "${meta_dir}" \
        --output "${output_root}/gate2-validation.json"
    validation_code=$?
    set -e
    printf 'Gate2 process_exit=%s validation_exit=%s wall_s=%s metadata=%s\n' \
        "${exit_code}" "${validation_code}" "$((end_epoch - start_epoch))" "${meta_dir}"
    if [[ ${exit_code} -ne 0 || ${validation_code} -ne 0 ]]; then
        return 1
    fi
}

mobility_main() {
    local stage=${1:-integration-rule}
    case "${stage}" in
        dry-run)
            mobility_print_command
            ;;
        integration-rule)
            if [[ "${DRY_RUN:-0}" == "1" ]]; then
                mobility_print_command
            else
                mobility_run_gate2
            fi
            ;;
        *)
            mobility_die "usage: $0 {dry-run|integration-rule}"
            ;;
    esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    mobility_main "$@"
fi
