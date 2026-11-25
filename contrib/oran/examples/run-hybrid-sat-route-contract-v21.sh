#!/usr/bin/env bash
set -euo pipefail

readonly CONTRACT_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly CONTRACT_REPO_ROOT="$(cd "${CONTRACT_SCRIPT_DIR}/../../.." && pwd -P)"
readonly CONTRACT_BINARY="${CONTRACT_REPO_ROOT}/build/contrib/oran/examples/ns3-dev-oran-nr-uav-xhaul-autonomy-example-optimized"
readonly CONTRACT_NR_LIBRARY="${CONTRACT_REPO_ROOT}/build/lib/libns3-dev-nr-optimized.so"
readonly CONTRACT_ROOT="${CONTRACT_REPO_ROOT}/results/nr/tn-ntn/route-contract-v21"
readonly CONTRACT_LABEL="route-contract-run9002-final-v21"
readonly CONTRACT_RESULT="${CONTRACT_ROOT}/raw/tn-uav-satellite_ueS1_1_ueS2_0_tnGnb_1_ntnGnb_1_tnCap_20_ntnCap_1_hyst_2_${CONTRACT_LABEL}"

[[ -x "${CONTRACT_BINARY}" ]] || { printf 'missing binary: %s\n' "${CONTRACT_BINARY}" >&2; exit 2; }
[[ -f "${CONTRACT_NR_LIBRARY}" ]] || { printf 'missing NR library: %s\n' "${CONTRACT_NR_LIBRARY}" >&2; exit 2; }
[[ ! -e "${CONTRACT_ROOT}" ]] || { printf 'refusing to overwrite: %s\n' "${CONTRACT_ROOT}" >&2; exit 2; }
mkdir -p "${CONTRACT_ROOT}/raw" "${CONTRACT_ROOT}/meta"

args=(
  --deployment-mode=tn-uav-satellite --sim-time=50 --stop-tail=2
  --RngSeed=1 --RngRun=9002
  --num-uess1=1 --num-ground-ues=0 --num-tn-gnbs=1 --num-ntn-gnbs=1
  --max-ues-tn=20 --max-ues-ntn=1 --ground-attach-delay=5
  --use-oran=1 --use-rsrp-lm=1 --use-onnx-lm=0 --use-torch-lm=0
  --lm-query-interval=1 --e2-send-interval=1 --tx-delay=0.1
  --handover-algorithm=ns3::NrNoOpHandoverAlgorithm --rlc-mode=um
  --monitored-traffic=udp --monitored-dl-rate-mbps=0.2
  --monitored-ul-rate-mbps=0.05 --monitored-packet-size=1000
  --tn-tx-power-dbm=-100 --xhaul-tx-power-dbm=46 --uav-tx-power-dbm=46
  --ue-tx-power-dbm=23 --init-min-rsrp=-200 --handover-min-rsrp-dbm=-200
  --enable-harq-retx=1 --enable-fading=1 --enable-startup-ping=0
  --enable-dedicated-qos-flows=1 --enable-flow-monitor=1 --enable-rsrp-trace=1
  --enable-position-trace=1 --position-trace-interval=1
  --enable-handover-trace=1 --enable-handover-failure-trace=1
  --enable-decision-csv=1 --enable-oran-info-log=1
  --sat-backhaul-scenario=NTN-Suburban --sat-backhaul-min-snr-db=0
  --sat-backhaul-log-step-ms=500 --enable-sat-backhaul-monitor=1
  --uav-control-start=5 --uav-control-period=1 --closed-loop-uav-targeting=0
  --uav-speed-mps=0 --tn-area-half-w-m=100 --tn-area-half-h-m=100
  --ue-area-half-w-m=100 --ue-area-half-h-m=100
  --uav-initial-area-half-w-m=100 --uav-initial-area-half-h-m=100
  --uav-area-half-w-m=100 --uav-area-half-h-m=100
  --xhaul-pathloss-exponent=2 --xhaul-reference-distance-m=100
  --xhaul-healthy-rsrp-dbm=-100 --enable-xhaul-channel-variation=0
  --xhaul-shadowing-stream=7301 --xhaul-fading-stream=7302
  --xhaul-degradation-start=15 --xhaul-degradation-stop=35
  --xhaul-degradation-penalty-db=100 --xhaul-degradation-ramp-s=0
  --xhaul-donor-unavailable-start=-1 --xhaul-donor-unavailable-stop=-1
  --xhaul-switch-to-sat-ttt-s=5 --xhaul-switch-to-tn-ttt-s=5
  --enable-uav-switching-xapp=1 --enable-ai-uav-switching-xapp=0
  --uav-outage-predictor-mode=off --xhaul-route-update-delay-s=0.05
  --backhaul-aware-handover=1 --run-label="${CONTRACT_LABEL}"
  --output-parent-dir="${CONTRACT_ROOT}/raw"
  --db-file="${CONTRACT_ROOT}/meta/oran-repository.db"
)

{
  printf '%q ' "${CONTRACT_BINARY}" "${args[@]}"
  printf '\n'
} > "${CONTRACT_ROOT}/meta/command.sh.txt"
sha256sum "${CONTRACT_BINARY}" \
  "${CONTRACT_NR_LIBRARY}" \
  "${CONTRACT_REPO_ROOT}/contrib/oran/examples/oran-nr-uav-xhaul-autonomy-example.cc" \
  "${CONTRACT_REPO_ROOT}/src/nr/helper/nr-hybrid-sat-epc-helper.cc" \
  "${CONTRACT_REPO_ROOT}/contrib/oran/examples/run-hybrid-sat-route-contract-v21.sh" \
  "${CONTRACT_REPO_ROOT}/contrib/oran/examples/validate-hybrid-sat-route-contract.py" \
  > "${CONTRACT_ROOT}/meta/frozen-inputs.sha256"

(
  cd "${CONTRACT_REPO_ROOT}"
  NS_LOG='HybridSatEpcHelper=level_info|prefix_time' \
    OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/time -v \
    -o "${CONTRACT_ROOT}/meta/time.txt" \
    "${CONTRACT_BINARY}" "${args[@]}" \
    > "${CONTRACT_ROOT}/meta/stdout.log" \
    2> "${CONTRACT_ROOT}/meta/stderr.log"
)

python3 "${CONTRACT_SCRIPT_DIR}/validate-hybrid-sat-route-contract.py" \
  "${CONTRACT_RESULT}" --output "${CONTRACT_ROOT}/route-contract.json"
find "${CONTRACT_RESULT}" -type f -print0 | sort -z | xargs -0 sha256sum \
  > "${CONTRACT_ROOT}/meta/raw-artifacts.sha256"
