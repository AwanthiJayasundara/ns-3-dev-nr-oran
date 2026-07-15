#!/usr/bin/env python3
"""Summarize NR handover failures and O-RAN LM handover rejections.

Usage:
  python3 contrib/oran/examples/analyze-uav-xhaul-ho-failures.py \
    results/nr/tn-ntn/<run-folder>
"""

from __future__ import annotations

import csv
import re
import sys
from collections import Counter
from pathlib import Path


def count_lines(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open(errors="ignore") as f:
        return sum(1 for line in f if line.strip())


def parse_lm_failures(path: Path) -> tuple[Counter[str], Counter[str], list[dict[str, str]]]:
    by_reason: Counter[str] = Counter()
    by_ue: Counter[str] = Counter()
    rows: list[dict[str, str]] = []

    if not path.exists():
        return by_reason, by_ue, rows

    pattern = re.compile(
        r"LM (?P<reason>HO_FAIL_[A-Z0-9_]+)"
        r".*?UE=(?P<ue>\d+)"
        r".*?currCell=(?P<curr>\d+)"
        r".*?candCell=(?P<cand>\d+)"
        r".*?candRsrp=(?P<rsrp>-?\d+(?:\.\d+)?)"
    )

    with path.open(errors="ignore") as f:
        for line in f:
            match = pattern.search(line)
            if not match:
                continue

            row = match.groupdict()
            by_reason[row["reason"]] += 1
            by_ue[row["ue"]] += 1
            rows.append(row)

    return by_reason, by_ue, rows


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__.strip())
        return 2

    run_dir = Path(sys.argv[1])
    if not run_dir.is_dir():
        print(f"Run folder not found: {run_dir}")
        return 1

    rrc_failure_file = run_dir / "handover-failure-trace.tr"
    lm_log_file = run_dir / "ns3-oran-lm.log"

    rrc_failures = count_lines(rrc_failure_file)
    lm_by_reason, lm_by_ue, rows = parse_lm_failures(lm_log_file)
    lm_total = sum(lm_by_reason.values())

    print(f"Run folder: {run_dir}")
    print(f"NR/RRC executed handover failures: {rrc_failures}")
    print(f"O-RAN LM rejected handover candidates: {lm_total}")

    if lm_by_reason:
        print("\nO-RAN LM failures by reason:")
        for reason, count in lm_by_reason.most_common():
            print(f"  {reason}: {count}")

    if lm_by_ue:
        print("\nTop UEs with LM rejected handover candidates:")
        for ue, count in lm_by_ue.most_common(10):
            print(f"  UE {ue}: {count}")

    out_file = run_dir / "lm-ho-failure-summary.csv"
    with out_file.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["reason", "ue", "curr", "cand", "rsrp"])
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nWrote: {out_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
