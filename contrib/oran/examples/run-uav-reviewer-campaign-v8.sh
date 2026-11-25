#!/usr/bin/env bash
# Plan, and only with RUN_FULL=1 execute, the frozen v8 four-policy reviewer campaign.

set -euo pipefail

CAMPAIGN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly CAMPAIGN_SCRIPT_DIR
# shellcheck source=contrib/oran/examples/run-uav-reviewer-pilot-v8.sh
source "${CAMPAIGN_SCRIPT_DIR}/run-uav-reviewer-pilot-v8.sh"

campaign_argv_sha256() {
    printf '%s\0' "$@" | sha256sum | awk '{print $1}'
}

campaign_assert_pilot_evidence() {
    local pilot_validation=$1
    local executable=$2
    local validation_python
    local matched_hash rule_policy_hash predictor_policy_hash

    pilot_assert_frozen_inputs "${executable}"
    pilot_common_args 6
    matched_hash=$(campaign_argv_sha256 "${PILOT_COMMON_ARGS[@]}")
    pilot_policy_args rule
    rule_policy_hash=$(campaign_argv_sha256 "${PILOT_POLICY_ARGS[@]}")
    pilot_policy_args predictor
    predictor_policy_hash=$(campaign_argv_sha256 "${PILOT_POLICY_ARGS[@]}")
    validation_python=$(pilot_python)

    "${validation_python}" - "${pilot_validation}" "${matched_hash}" \
        "${rule_policy_hash}" "${predictor_policy_hash}" "${executable}" \
        "${PILOT_EXAMPLE}" "${PILOT_HELPER}" "${PILOT_NR_LIBRARY}" \
        "${PILOT_MODEL}" "${PILOT_SCHEMA}" "${PILOT_RUNNER}" \
        "${PILOT_VALIDATOR}" "${PILOT_CAMPAIGN_RUNNER}" \
        "${PILOT_ROUTE_CONTRACT_REPORT}" "${PILOT_ROUTE_CONTRACT_MANIFEST}" \
        "${PILOT_CMAKE_CACHE}" "${PILOT_EXPECTED_UES}" \
        "${PILOT_MONITORED_UES}" "${PILOT_BACKGROUND_UES}" \
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
        "${PILOT_MONITORED_DL_START_S}" "${PILOT_MONITORED_UL_START_S}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

(
    report_arg, matched_hash, rule_policy_hash, predictor_policy_hash,
    executable, source, helper, nr_library, model, schema, runner, validator,
    campaign_runner, route_report, route_manifest, cmake_cache, expected_ues,
    monitored_ues, background_ues, tn_gnbs, uav_gnbs, max_ues_tn,
    max_ues_uav, tn_total_capacity, uav_total_capacity,
    ground_attach_delay_s, initial_attach_stagger_s,
    capacity_stratified_ue_placement, ue_cluster_radius_m, ue_area_half_w_m,
    ue_area_half_h_m,
    uav_initial_area_half_w_m, uav_initial_area_half_h_m, uav_area_half_w_m,
    uav_area_half_h_m, uav_speed_mps, mobility_update_ms, xhaul_pathloss_exponent,
    expected_monitored_tn_ues, expected_monitored_uav_ues,
    monitored_dl_start_s, monitored_ul_start_s,
) = sys.argv[1:]

def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

report_path = Path(report_arg).resolve()
report = json.loads(report_path.read_text(encoding="utf-8"))
if report.get("overall_passed") is not True or report.get("errors"):
    raise SystemExit("two-arm pilot validation did not pass overall")
runs = report.get("runs", [])
if report.get("run_count") != 2 or len(runs) != 2:
    raise SystemExit("pilot validation must contain exactly two runs")
runs_by_policy = {run.get("metadata", {}).get("policy"): run for run in runs}
if set(runs_by_policy) != {"rule", "predictor"}:
    raise SystemExit("pilot validation must contain exactly rule and predictor")

artifact_fields = (
    ("source_path", "source_sha256", source),
    ("helper_path", "helper_sha256", helper),
    ("binary_path", "binary_sha256", executable),
    ("nr_library_path", "nr_library_sha256", nr_library),
    ("model_path", "model_sha256", model),
    ("schema_path", "schema_sha256", schema),
    ("runner_path", "runner_sha256", runner),
    ("validator_path", "validator_sha256", validator),
    ("campaign_runner_path", "campaign_runner_sha256", campaign_runner),
    ("route_contract_report_path", "route_contract_report_sha256", route_report),
    ("route_contract_manifest_path", "route_contract_manifest_sha256", route_manifest),
    ("cmake_cache_path", "cmake_cache_sha256", cmake_cache),
)
archived_fields = (
    ("matched_config_path", "matched_config_sha256"),
    ("policy_config_path", "policy_config_sha256"),
    ("full_config_path", "full_config_sha256"),
    ("git_status_path", "git_status_sha256"),
    ("git_diff_path", "git_diff_sha256"),
    ("build_provenance_path", "build_provenance_sha256"),
    ("cmake_cache_archive_path", "cmake_cache_archive_sha256"),
    ("frozen_inputs_manifest_path", "frozen_inputs_manifest_sha256"),
)
expected_design = {
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
}
expected_policy_hashes = {
    "rule": rule_policy_hash,
    "predictor": predictor_policy_hash,
}

for policy, run in runs_by_policy.items():
    if run.get("gates_passed") is not True:
        raise SystemExit(f"pilot run gates failed for policy={policy}")
    metadata = run.get("metadata", {})
    expected_identity = {
        "scope": "pilot",
        "rng_seed": 1,
        "rng_run": 6,
        "protocol_version": "uav-reviewer-v8",
    }
    for field, expected in expected_identity.items():
        if metadata.get(field) != expected:
            raise SystemExit(
                f"pilot {policy} identity mismatch for {field}: "
                f"expected={expected!r} observed={metadata.get(field)!r}"
            )
    for path_field, hash_field, current_arg in artifact_fields:
        current = Path(current_arg).resolve()
        if Path(metadata.get(path_field, "")).resolve() != current:
            raise SystemExit(f"pilot {policy} path mismatch for {path_field}")
        current_hash = digest(current)
        if metadata.get(hash_field) != current_hash:
            raise SystemExit(f"pilot {policy} current hash mismatch for {hash_field}")
    for path_field, hash_field in archived_fields:
        archived = Path(metadata.get(path_field, ""))
        if not archived.is_file() or metadata.get(hash_field) != digest(archived):
            raise SystemExit(f"pilot {policy} archived evidence mismatch for {path_field}")
    if metadata.get("matched_config_sha256") != matched_hash:
        raise SystemExit(f"pilot {policy} does not match the current common configuration")
    if metadata.get("policy_config_sha256") != expected_policy_hashes[policy]:
        raise SystemExit(f"pilot {policy} does not match the current policy configuration")
    if metadata.get("radio_contract") != {
        "rlc_mode": "um",
        "harq_retransmissions_enabled": True,
        "dedicated_qos_flows_enabled": True,
        "use_rsrp_lm": False,
        "fixed_initial_association": True,
    }:
        raise SystemExit(f"pilot {policy} radio contract mismatch")
    if metadata.get("fixed_initial_association") is not True:
        raise SystemExit(f"pilot {policy} is not fixed initial association")
    if metadata.get("mobility_lm") != "noop":
        raise SystemExit(f"pilot {policy} mobility LM is not noop")
    if metadata.get("initial_attach_stagger_s") != float(initial_attach_stagger_s):
        raise SystemExit(f"pilot {policy} initial attach stagger mismatch")
    if metadata.get("monitored_dl_start_s") != float(monitored_dl_start_s):
        raise SystemExit(f"pilot {policy} monitored DL start mismatch")
    if metadata.get("monitored_ul_start_s") != float(monitored_ul_start_s):
        raise SystemExit(f"pilot {policy} monitored UL start mismatch")
    if metadata.get("mobility_update_ms") != float(mobility_update_ms):
        raise SystemExit(f"pilot {policy} mobility update interval mismatch")
    expected_ue_mobility_contract = {
        "description": "stationary UAV anchors with bounded local UE mobility",
        "monitored_ue_speed_mps": 5.0,
        "background_ue_speed_mps": 4.0,
        "uav_speed_mps": float(uav_speed_mps),
        "update_interval_ms": float(mobility_update_ms),
        "cluster_half_width_m": float(ue_cluster_radius_m),
        "cluster_half_height_m": float(ue_cluster_radius_m),
    }
    if metadata.get("ue_mobility_contract") != expected_ue_mobility_contract:
        raise SystemExit(f"pilot {policy} UE mobility contract mismatch")
    if metadata.get("ue_placement") != {
        "mode": "capacity-stratified-clustered",
        "capacity_stratified": True,
        "cluster_radius_m": float(ue_cluster_radius_m),
        "global_area_half_w_m": float(ue_area_half_w_m),
        "global_area_half_h_m": float(ue_area_half_h_m),
    }:
        raise SystemExit(f"pilot {policy} clustered placement metadata mismatch")
    if metadata.get("evidence_scope") != (
        "backhaul switching and monitored-UE QoS under fixed initial association"
    ):
        raise SystemExit(f"pilot {policy} evidence scope mismatch")
    claim_scope = metadata.get("claim_scope", {})
    if claim_scope.get("handover_success") != "not-applicable":
        raise SystemExit(f"pilot {policy} handover claim scope mismatch")
    if claim_scope.get("ue_pingpong") != "not-applicable":
        raise SystemExit(f"pilot {policy} ping-pong claim scope mismatch")
    if metadata.get("frozen_design") != expected_design:
        raise SystemExit(f"pilot {policy} frozen capacity/placement design mismatch")
    if metadata.get("expected_ues") != int(expected_ues):
        raise SystemExit(f"pilot {policy} expected UE count mismatch")
    if metadata.get("route_contract_identity") != "route-contract-v20":
        raise SystemExit(f"pilot {policy} route-contract identity mismatch")
    if metadata.get("route_contract_passed") is not True:
        raise SystemExit(f"pilot {policy} route contract is not PASS")
    if metadata.get("route_contract_all_checks_passed") is not True:
        raise SystemExit(f"pilot {policy} route-contract checks are not all PASS")

print(f"campaign_pilot_gate=PASS json={report_path}")
PY
}

campaign_assert_completion() {
    local campaign_validation=$1
    local validation_python
    validation_python=$(pilot_python)
    "${validation_python}" - "${campaign_validation}" <<'PY'
import collections
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1]).resolve()
report = json.loads(report_path.read_text(encoding="utf-8"))
if report.get("overall_passed") is not True or report.get("errors"):
    raise SystemExit("campaign validation did not pass overall")
runs = report.get("runs", [])
if report.get("run_count") != 20 or len(runs) != 20:
    raise SystemExit("campaign completion requires exactly 20 validated runs")
policies = {"rule", "predictor", "fast-rule", "no-satellite"}
expected = {(1, rng_run, policy) for rng_run in range(7, 12) for policy in policies}
observed = [
    (
        run.get("metadata", {}).get("rng_seed"),
        run.get("metadata", {}).get("rng_run"),
        run.get("metadata", {}).get("policy"),
    )
    for run in runs
]
counts = collections.Counter(observed)
if set(counts) != expected or any(count != 1 for count in counts.values()):
    raise SystemExit(
        "campaign completion requires one exact four-policy group for each RngRun 7..11"
    )
if any(run.get("metadata", {}).get("scope") != "campaign" for run in runs):
    raise SystemExit("campaign completion report contains a non-campaign run")
if any(run.get("gates_passed") is not True for run in runs):
    raise SystemExit("campaign completion report contains a failed run gate")
print(f"campaign_completion=PASS groups=5 runs=20 json={report_path}")
PY
}

campaign_main() {
    local output_root=${CAMPAIGN_OUTPUT_ROOT:-${PILOT_REPO_ROOT}/results/nr/tn-ntn/reviewer-campaign-v8}
    local pilot_validation=${PILOT_VALIDATION_JSON:-${PILOT_REPO_ROOT}/results/nr/tn-ntn/reviewer-pilot-v8/pilot-validation.json}
    local campaign_validation="${output_root}/campaign-validation.json"
    local executable=${PILOT_EXECUTABLE:-${PILOT_EXECUTABLE_DEFAULT}}
    local validation_python
    local rng_run policy meta_dir
    local -a policies=(rule predictor fast-rule no-satellite)
    local -a runs=(7 8 9 10 11)
    local -a validation_args=()

    printf 'Frozen campaign: RngSeed=1; RngRun=7..11; 100 UEs; policies=%s\n' \
        "${policies[*]}"
    printf 'Evidence scope: fixed-association backhaul switching/QoS; handover and ping-pong are N/A.\n'
    printf 'Output root: %s\n' "${output_root}"
    for rng_run in "${runs[@]}"; do
        for policy in "${policies[@]}"; do
            pilot_print_command "${policy}" "${rng_run}" "${output_root}" campaign
        done
    done

    if [[ "${RUN_FULL:-0}" != "1" ]]; then
        printf 'DRY RUN ONLY. Set RUN_FULL=1 after the pilot validation passes to execute.\n'
        return 0
    fi

    validation_python=$(pilot_python)
    [[ -f "${pilot_validation}" ]] || pilot_die \
        "pilot validation is required before the full campaign: ${pilot_validation}"
    campaign_assert_pilot_evidence "${pilot_validation}" "${executable}"
    [[ ! -e "${output_root}" ]] || pilot_die \
        "full campaign requires a fresh output root; refusing to reuse ${output_root}"

    for rng_run in "${runs[@]}"; do
        for policy in "${policies[@]}"; do
            run_reviewer_arm "${policy}" "${rng_run}" "${output_root}" campaign
            if [[ "${PILOT_LAST_EXIT_CODE}" -ne 0 ]]; then
                printf 'Simulation failed for policy=%s RngRun=%s; campaign stopped.\n' \
                    "${policy}" "${rng_run}" >&2
                return "${PILOT_LAST_EXIT_CODE}"
            fi
            meta_dir="${output_root}/meta/reviewer_campaign_${policy}_run${rng_run}"
            validation_args+=(--metadata-dir "${meta_dir}")
        done
    done

    "${validation_python}" "${PILOT_VALIDATOR}" \
        "${validation_args[@]}" \
        --output "${campaign_validation}"
    campaign_assert_completion "${campaign_validation}"
}

campaign_main "$@"

