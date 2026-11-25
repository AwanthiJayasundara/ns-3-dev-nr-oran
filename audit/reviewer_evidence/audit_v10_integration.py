#!/usr/bin/env python3
"""Recompute and plot completed v10 integration diagnostics without editing the sealed run."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import re
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RUN_ROOT = ROOT / "results/nr/tn-ntn/reviewer-integration-pair-v10"
VALIDATOR = ROOT / "contrib/oran/examples/validate-uav-reviewer-integration-pair-v10.py"
SEALED_VALIDATION = "4e0220ea7015ca8b520d1c373eee756e6c66fbbae59f498f775014bca354bf42"
RIC_START_S = 2.5  # Start time in the v10 frozen scenario source.
ROUTE_LEVEL = {"NO_BACKHAUL_AVAILABLE": 0, "TN_DIRECT": 1, "SATELLITE_FALLBACK": 2}


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def verified_raw_files(run_path, manifest):
    records = {}
    for line in manifest.read_text().splitlines():
        expected, filename = line.split(maxsplit=1)
        file = run_path / Path(filename).name
        if file.name in records or digest(file) != expected:
            raise ValueError(f"Failed raw artifact identity: {file}")
        records[file.name] = expected
    if set(records) != {path.name for path in run_path.iterdir() if path.is_file()}:
        raise ValueError(f"Raw manifest and directory differ: {run_path}")
    return records


def csv_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise ValueError(f"No rows for {path}")
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def packet_pairs(path):
    events = {"TX": {}, "RX": {}}
    for row in csv_rows(path):
        key = (int(row["UE"]), row["Dir"], int(row["Sequence"]))
        side = events[row["Event"]]
        time = float(row["Time"])
        sent = float(row["SenderTimestampSec"])
        size = int(row["PacketBytes"])
        if key in side or not math.isfinite(time) or not math.isfinite(sent) or size <= 0:
            raise ValueError(f"Invalid or duplicate packet event: {row}")
        side[key] = (time, sent, size)
    tx, rx = events["TX"], events["RX"]
    if not rx.keys() <= tx.keys():
        raise ValueError("Orphan RX key")
    for key, (time, sent, _) in tx.items():
        if abs(time - sent) > 1e-6:
            raise ValueError(f"TX timestamp discrepancy: {key}")
        if key in rx and (abs(sent - rx[key][1]) > 1e-6 or rx[key][0] < sent):
            raise ValueError(f"RX timestamp discrepancy: {key}")
    return tx, rx


def serving_cells(raw):
    identities = {
        int(row["IMSI"]): int(row["UE"])
        for row in csv_rows(raw / "ue-identity-map.csv")
        if row["Group"] == "S1"
    }
    cells = defaultdict(set)
    with (raw / "rsrp-trace.tr").open() as stream:
        fields = stream.readline().split()
        imsi, cell, serving = [fields.index(name) for name in ("IMSI", "CellId", "Serving")]
        for line in stream:
            row = line.split()
            if row[serving] == "1" and int(row[imsi]) in identities:
                cells[identities[int(row[imsi])]].add(int(row[cell]))
    if len(identities) != len(cells) or any(len(values) != 1 for values in cells.values()):
        raise ValueError("Monitored serving cells are incomplete or changed")
    return {ue: next(iter(values)) for ue, values in cells.items()}


def packet_summary(tx, rx, cells, lower, upper, inclusive):
    by_group = defaultdict(list)
    for key, (time, _, _) in tx.items():
        if time < lower or time > upper or (time == upper and not inclusive):
            continue
        ue, direction, _ = key
        cell = cells[ue]
        for label in ("all", "TN" if cell <= 4 else "UAV", f"cell_{cell}"):
            by_group[label, direction].append(key)
            by_group[label, "ALL"].append(key)
    rows = []
    for (cohort, direction), keys in sorted(by_group.items()):
        delays = [rx[key][0] - tx[key][0] for key in keys if key in rx]
        rows.append({
            "cohort": cohort, "direction": direction, "tx": len(keys), "rx": len(delays),
            "lost": len(keys) - len(delays), "pdr": len(delays) / len(keys),
            "mean_delivered_delay_ms": 1000 * math.fsum(delays) / len(delays) if delays else None,
            "max_delivered_delay_ms": 1000 * max(delays) if delays else None,
        })
    return rows


def event_rows(events, switching, policy):
    linked = {
        (int(row["uav_cell_id"]), float(row["apply_time_s"])): row
        for row in switching["linked_satellite_policy_events"]
    }
    result = []
    for row in events:
        key = (int(row["UavCellId"]), float(row["ApplyTime"]))
        link = linked.get(key, {})
        result.append({
            "policy": policy, "cell": key[0], "decision_s": row["DecisionTime"],
            "apply_s": key[1], "actuation_s": row["ActuationDelaySec"], "event": row["EventType"],
            "previous_route": row["PreviousRouteMode"], "actual_route": row["ActualRouteMode"],
            "guard": row["GuardResult"], "linked_outage": link.get("episode_id", ""),
            "raw_onset_to_apply_s": link.get("apply_minus_raw_onset_s", ""),
            "route_interruption_s": link.get("route_interruption_s", ""),
        })
    return result


def save_figures(output, events, bins, duration, start, stop):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    colors = {"rule": "#235F98", "predictor": "#AE4B17"}
    styles = {"rule": "-", "predictor": "--"}
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    figure, panels = plt.subplots(3, 1, figsize=(7.5, 6.2), sharex=True, layout="constrained")
    for cell, axis in zip((5, 6, 7), panels):
        axis.axvspan(start, stop, color="#777777", alpha=0.12)
        axis.axvline(14.5, color="#777777", linewidth=0.8, linestyle=":")
        for policy in colors:
            series = [row for row in events if row["cell"] == cell and row["policy"] == policy]
            times = [10, *(row["apply_s"] for row in series), duration]
            levels = [1, *(ROUTE_LEVEL[row["actual_route"]] for row in series)]
            axis.step(times, [*levels, levels[-1]], where="post", color=colors[policy],
                      linestyle=styles[policy], linewidth=1.8)
        axis.set_yticks([0, 1, 2], ["Unavailable", "Terrestrial", "Satellite"])
        axis.set_ylim(-0.22, 2.22)
        axis.set_ylabel(f"UAV cell {cell}")
    panels[-1].set_xlabel("Simulation time (s)")
    panels[-1].set_xlim(10, duration)
    panels[0].legend(handles=[Line2D([], [], color=colors[p], linestyle=styles[p], label=p.title())
                              for p in colors], loc="upper right")
    figure.suptitle("v10 actual backhaul routes, one matched integration run")
    for suffix in ("pdf", "png"):
        metadata = {"CreationDate": None, "ModDate": None} if suffix == "pdf" else None
        figure.savefig(output / f"v10_backhaul_timeline.{suffix}", dpi=200, metadata=metadata)
    plt.close(figure)

    figure, panels = plt.subplots(2, 1, figsize=(7.5, 4.8), sharex=True, layout="constrained")
    for cohort, axis in zip(("TN", "UAV"), panels):
        axis.axvspan(start, stop, color="#777777", alpha=0.12)
        for policy in colors:
            series = [row for row in bins if row["cohort"] == cohort and row["policy"] == policy]
            axis.stairs([row["pdr"] for row in series],
                        [*(row["bin_start_s"] for row in series), duration],
                        color=colors[policy], linestyle=styles[policy], linewidth=1.8,
                        label=policy.title(), baseline=None)
        axis.set_ylabel(f"{cohort}-served PDR")
        axis.set_ylim(-0.025, 1.06)
        axis.grid(axis="y", alpha=0.25)
    panels[0].legend(loc="lower right")
    panels[-1].set_xlabel("Application transmit time (s)")
    panels[-1].set_xlim(10, duration)
    figure.suptitle("v10 packet delivery by initial serving cohort (1 s TX bins)")
    for suffix in ("pdf", "png"):
        metadata = {"CreationDate": None, "ModDate": None} if suffix == "pdf" else None
        figure.savefig(output / f"v10_packet_delivery.{suffix}", dpi=200, metadata=metadata)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("v10_integration"))
    args = parser.parse_args()
    validation_path = RUN_ROOT / "integration-validation.json"
    if digest(validation_path) != SEALED_VALIDATION:
        raise ValueError("Sealed v10 validation hash mismatch")
    validation = json.loads(validation_path.read_text())
    if any(digest(VALIDATOR) != arm["metadata"]["validator_sha256"] for arm in validation["runs"]):
        raise ValueError("Frozen validator identity mismatch")
    spec = importlib.util.spec_from_file_location("frozen_v10_validator", VALIDATOR)
    frozen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(frozen)
    manifest_rows, metrics, event_data, bins = [], [], [], []
    adjudication, offered = {}, {}
    for arm in validation["runs"]:
        metadata = arm["metadata"]
        policy = metadata["policy"]
        label = metadata["run_label"]
        meta = RUN_ROOT / "meta" / label
        raw = RUN_ROOT / "raw" / Path(metadata["result_dir"]).name
        raw_manifest = meta / "raw-artifacts.sha256"
        raw_hashes = verified_raw_files(raw, raw_manifest)
        status = json.loads((meta / "process-status.json").read_text())
        if status["exit_code"] != 0:
            raise ValueError(f"Simulation failed: {label}")
        frozen_config = meta / "matched-config.argv0"
        if digest(frozen_config) != metadata["matched_config_sha256"]:
            raise ValueError(f"Matched config identity mismatch: {label}")
        con = dict(item.split("=", 1) for item in frozen_config.read_text().split("\0") if item)
        duration = float(con["--sim-time"])
        cadence = float(con["--lm-query-interval"])
        start, stop = map(float, (con["--xhaul-degradation-start"], con["--xhaul-degradation-stop"]))
        first = RIC_START_S + cadence
        query_count = math.ceil((duration - first) / cadence)
        last = first + (query_count - 1) * cadence
        ai, xhaul = raw / "uav-switching-ai-trace.csv", raw / "xhaul-autonomy-trace.csv"
        feature_gate = frozen.validate_online_feature_construction(
            ai, xhaul, expected_cadence_s=cadence, expected_first_epoch_s=first,
            expected_last_epoch_s=last,
        )
        model = ROOT / "results/ai/uav-sustained-outage-v1/sustained_outage_rf_v1.onnx"
        if any(digest(path) != metadata[key] for path, key in (
            (model, "model_sha256"),
            (RUN_ROOT / "replay-source" / Path(metadata["source_path"]).name, "source_sha256"),
            (RUN_ROOT / "replay-source" / Path(metadata["binary_path"]).name, "binary_sha256"),
        )):
            raise ValueError("Model, archived source, or archived executable identity mismatch")
        onnx_gate, onnx = frozen.validate_onnx_rows(ai, model, 1e-5, required=policy == "predictor")
        events = csv_rows(raw / "uav-switching-events.csv")
        switching = frozen.switching_metrics(raw / "uav-switching-events.csv", xhaul, start, stop, policy)
        event_data.extend(event_rows(events, switching, policy))
        tx, rx = packet_pairs(raw / "monitored-packet-events.csv")
        coverage = frozen.service_coverage_metrics(raw / "rsrp-trace.tr", raw / "uav-switching-events.csv",
                                                    xhaul, metadata["expected_ues"], -110.0, duration,
                                                    start, stop)
        cohort = frozen.packet_cohort_metrics(raw / "monitored-packet-events.csv", metadata["monitored_ues"],
                                              duration, start, stop)
        cells = serving_cells(raw)
        adjudication[policy] = {
            "original_failed_gates": [gate["name"] for gate in arm["gates"] if not gate["passed"]],
            "duration_derived_feature_gate": feature_gate, "onnx_gate": onnx_gate, "onnx": onnx,
            "served_tn_ues": sum(cell <= 4 for cell in cells.values()),
            "served_uav_ues": sum(cell >= 5 for cell in cells.values()),
            "coverage_degradation": coverage["windows"]["degradation"],
            "service_satisfaction_degradation": cohort["service_satisfaction"]["windows"]["degradation"],
            "raw_packet_counts": {"tx": len(tx), "rx": len(rx)},
            "linked_satellite_policy_events": switching["linked_satellite_policy_events"],
        }
        offered[policy] = tx
        for window, lower, upper, inclusive in (
            ("pre_degradation", 10, start, False), ("degradation", start, stop, False),
            ("post_degradation", stop, duration, True), ("full", 10, duration, True),
        ):
            summaries = packet_summary(tx, rx, cells, lower, upper, inclusive)
            original_window = arm["metrics"]["packet_cohort_qos"]["windows"][window]
            if cohort["windows"][window] != original_window:
                raise ValueError(f"Replayed window differs from sealed validation: {policy} {window}")
            for row in summaries:
                if row["cohort"] == "all":
                    original = original_window[row["direction"]]
                    if row["tx"] != original["tx_attempts"] or row["rx"] != original["rx_packets"]:
                        raise ValueError(f"Raw counts disagree with the sealed validation: {policy} {window}")
                metrics.append({"rng_seed": metadata["rng_seed"], "rng_run": metadata["rng_run"],
                                "policy": policy, "window": window, **row})
        for lower in range(10, int(duration)):
            upper = lower + 1
            for row in packet_summary(tx, rx, cells, lower, upper, upper == duration):
                if row["direction"] == "ALL" and row["cohort"] in {"TN", "UAV"}:
                    bins.append({"policy": policy, "bin_start_s": lower, "bin_stop_s": upper, **row})
        peak_rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", (meta / "time.txt").read_text())
        manifest_rows.append({
            "rng_seed": metadata["rng_seed"], "rng_run": metadata["rng_run"], "policy": policy,
            "exit_code": status["exit_code"], "wall_clock_seconds": status["wall_clock_seconds"],
            "peak_rss_kib": int(peak_rss.group(1)),
            "started_utc": (meta / "started-utc.txt").read_text().strip(),
            "finished_utc": (meta / "finished-utc.txt").read_text().strip(),
            "run_validation_passed": arm["gates_passed"], "handover_success_rate": "NA",
            "ue_pingpong_rate": "NA", "raw_path": str(raw.relative_to(ROOT)),
            "metadata_path": str(meta.relative_to(ROOT)),
            "expanded_command_path": str((meta / "command.sh.txt").relative_to(ROOT)),
            "expanded_command_sha256": digest(meta / "command.sh.txt"),
            "process_status_sha256": digest(meta / "process-status.json"),
            "raw_manifest_sha256": digest(raw_manifest), "raw_files_verified": len(raw_hashes),
            "raw_packets_sha256": raw_hashes["monitored-packet-events.csv"],
            "raw_xhaul_sha256": raw_hashes["xhaul-autonomy-trace.csv"],
            "raw_actions_sha256": raw_hashes["uav-switching-events.csv"],
            "raw_ai_sha256": raw_hashes["uav-switching-ai-trace.csv"],
            "metadata_sha256": digest(meta / "run.json"),
            **{key: metadata[key] for key in (
                "git_head", "git_diff_sha256", "source_sha256", "binary_sha256", "nr_library_sha256",
                "helper_sha256", "runner_sha256", "validator_sha256", "matched_config_sha256",
                "policy_config_sha256", "full_config_sha256", "frozen_inputs_manifest_sha256",
                "route_contract_report_sha256", "route_contract_manifest_sha256", "schema_sha256",
            )},
            "frozen_model_sha256": metadata["model_sha256"],
            "deployed_model_sha256": metadata["model_sha256"] if policy == "predictor" else "NA",
        })
    if offered["rule"] != offered["predictor"]:
        raise ValueError("Offered application attempts do not match")
    indexed = {(row["policy"], row["window"], row["cohort"], row["direction"]): row for row in metrics}
    pairs = []
    for row in metrics:
        if row["policy"] != "rule":
            continue
        candidate = indexed[("predictor", row["window"], row["cohort"], row["direction"])]
        pairs.append({
            "rng_seed": row["rng_seed"], "rng_run": row["rng_run"], "window": row["window"],
            "cohort": row["cohort"], "direction": row["direction"], "offered_tx_each": row["tx"],
            "rule_rx": row["rx"], "predictor_rx": candidate["rx"], "rule_pdr": row["pdr"],
            "predictor_pdr": candidate["pdr"], "pdr_delta_pp": 100 * (candidate["pdr"] - row["pdr"]),
            "rule_mean_delivered_delay_ms": row["mean_delivered_delay_ms"],
            "predictor_mean_delivered_delay_ms": candidate["mean_delivered_delay_ms"],
        })
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "matched_run_manifest.csv", manifest_rows)
    write_csv(args.output / "packet_metrics_per_seed.csv", metrics)
    write_csv(args.output / "paired_packet_differences.csv", pairs)
    write_csv(args.output / "route_events.csv", event_data)
    write_csv(args.output / "packet_delivery_1s.csv", bins)
    summary = {
        "original_validation_sha256": SEALED_VALIDATION,
        "original_overall_passed": validation["overall_passed"],
        "independent_offered_packet_matching_passed": True,
        "original_pair_matched_input_gates": validation["comparisons"][0]["gates"],
        "runs": adjudication,
    }
    (args.output / "trace_adjudication.json").write_text(json.dumps(summary, indent=2) + "\n")
    save_figures(args.output, event_data, bins, duration, start, stop)
    print(f"Wrote diagnostic exports to {args.output}")


if __name__ == "__main__":
    main()
