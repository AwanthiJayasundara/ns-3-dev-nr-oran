#!/usr/bin/env python3
"""
SOP vs target secrecy rate R_sec, with option to use only handover moments.

SOP definition:
  P_out(R_sec) = Pr{ C_s < R_sec }

subset modes:
  - all  : use all samples (default)  -> your original SOP
  - ho   : use only samples where handover==True  -> shows ONNX impact better
  - noho : use only samples where handover==False

Combine TN+NTN:
  - concat: concatenate samples (weighted by row count)
  - equal : compute SOP per domain and average 50/50

Optional time filtering: --t-min / --t-max on first token (time).
Expected log format (label,value pairs):
  time,UE,id,cell,<cellId>,...,secrecy,<val>,outage,<val>
"""

from pathlib import Path
import argparse
import math
import numpy as np
import matplotlib.pyplot as plt

# Use consistent LaTeX-compatible style
plt.style.use('./latex_style.mplstyle')

plt.rcParams.update({
    'font.size': 20,         # base font size for all text
    'axes.titlesize': 20,    # title
    'axes.labelsize': 20,    # x and y labels
    'xtick.labelsize': 20,   # x tick labels
    'ytick.labelsize': 20,   # y tick labels
    'legend.fontsize': 20,   # legend
})


def _safe_float(x: str):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
        return None
    except Exception:
        return None


def _safe_int(x: str):
    try:
        return int(float(x))
    except Exception:
        return None


def parse_rows(filepath: Path, domain: str, t_min=None, t_max=None):
    """
    Parse file into rows with fields:
      time, domain, ue_id, cell, secrecy
    """
    rows = []
    with filepath.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            toks = [t.strip() for t in line.split(",") if t.strip() != ""]
            if len(toks) < 6:
                continue

            # header skip
            if toks[0].lower() == "time":
                continue

            # time
            t = _safe_float(toks[0])
            if t is None:
                continue

            if (t_min is not None) and (t < t_min):
                continue
            if (t_max is not None) and (t > t_max):
                continue

            # type (optional filter: keep UE only if present)
            if len(toks) > 1 and toks[1].lower() != "ue":
                continue

            # ue id
            if len(toks) < 3:
                continue
            ue_id = _safe_int(toks[2])
            if ue_id is None:
                continue

            # parse label/value pairs starting from index 3
            kv = {}
            i = 3
            while i + 1 < len(toks):
                kv[toks[i].lower()] = toks[i + 1]
                i += 2

            cell = _safe_int(kv.get("cell", ""))
            cs = _safe_float(kv.get("secrecy", ""))

            if cell is None or cs is None:
                continue

            if cs < 0:
                cs = 0.0

            rows.append(
                {"time": t, "domain": domain, "id": ue_id, "cell": cell, "secrecy": cs}
            )

    return rows


def mark_handovers(rows):
    """
    Mark handover=True if cell changes for same (domain, ue_id) between consecutive samples.
    """
    rows_sorted = sorted(rows, key=lambda r: (round(r["time"], 6), r["domain"], r["id"]))
    last_cell = {}

    for r in rows_sorted:
        key = (r["domain"], r["id"])
        prev = last_cell.get(key, None)
        cur = r["cell"]
        r["handover"] = (prev is not None) and (cur != prev)
        last_cell[key] = cur

    return rows_sorted


def sop_curve(cs: np.ndarray, rsec: np.ndarray) -> np.ndarray:
    if cs.size == 0:
        return np.full_like(rsec, np.nan, dtype=float)
    return (cs[:, None] < rsec[None, :]).mean(axis=0)


def combine_domains_sop(cs_tn: np.ndarray, cs_ntn: np.ndarray, rsec: np.ndarray, mode: str) -> np.ndarray:
    mode = mode.lower().strip()
    if mode == "concat":
        cs = np.concatenate([cs_tn, cs_ntn]) if (cs_tn.size or cs_ntn.size) else np.array([], dtype=float)
        return sop_curve(cs, rsec)

    if mode == "equal":
        curves = []
        if cs_tn.size:
            curves.append(sop_curve(cs_tn, rsec))
        if cs_ntn.size:
            curves.append(sop_curve(cs_ntn, rsec))
        if not curves:
            return np.full_like(rsec, np.nan, dtype=float)
        return np.mean(np.vstack(curves), axis=0)

    raise ValueError(f"Unknown combine mode: {mode}")


def extract_cs_by_subset(rows_sorted, domain: str, subset: str) -> np.ndarray:
    subset = subset.lower().strip()
    if subset == "all":
        vals = [r["secrecy"] for r in rows_sorted if r["domain"] == domain]
    elif subset == "ho":
        vals = [r["secrecy"] for r in rows_sorted if r["domain"] == domain and r["handover"]]
    elif subset == "noho":
        vals = [r["secrecy"] for r in rows_sorted if r["domain"] == domain and (not r["handover"])]
    else:
        raise ValueError("subset must be one of: all, ho, noho")
    return np.array(vals, dtype=float)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--wo-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withoutsecrecylm/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--wo-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withoutsecrecylm/tn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--w-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--w-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/tn/50_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--onnx-ntn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/ntn/50_ues/secrecy-sinr-vs-time.txt"))
    parser.add_argument("--onnx-tn", type=Path,
                        default=Path("results/nr/tn-ntn/withsecrecylm/onnx/tn/50_ues/secrecy-sinr-vs-time.txt"))

    parser.add_argument("--out-dir", type=Path, default=Path("results/nr/tn-ntn"))
    parser.add_argument("--t-min", type=float, default=None)
    parser.add_argument("--t-max", type=float, default=None)

    parser.add_argument("--combine", choices=["concat", "equal"], default="concat")
    parser.add_argument("--subset", choices=["all", "ho", "noho"], default="all",
                        help="Use all samples, only handover samples, or only non-handover samples.")

    parser.add_argument("--rsec-max", type=float, default=None)
    parser.add_argument("--rsec-points", type=int, default=101)

    args = parser.parse_args()

    # --- parse + mark HO for each scenario ---
    def load_scenario(tn_path: Path, ntn_path: Path):
        rows = []
        rows += parse_rows(tn_path, "tn", args.t_min, args.t_max)
        rows += parse_rows(ntn_path, "ntn", args.t_min, args.t_max)
        rows = mark_handovers(rows)
        cs_tn = extract_cs_by_subset(rows, "tn", args.subset)
        cs_ntn = extract_cs_by_subset(rows, "ntn", args.subset)
        return cs_tn, cs_ntn

    cs_wo_tn, cs_wo_ntn = load_scenario(args.wo_tn, args.wo_ntn)
    cs_w_tn, cs_w_ntn = load_scenario(args.w_tn, args.w_ntn)
    cs_onnx_tn, cs_onnx_ntn = load_scenario(args.onnx_tn, args.onnx_ntn)

    # choose x-axis range
    all_cs = np.concatenate([cs_wo_tn, cs_wo_ntn, cs_w_tn, cs_w_ntn, cs_onnx_tn, cs_onnx_ntn]) \
        if (cs_wo_tn.size + cs_wo_ntn.size + cs_w_tn.size + cs_w_ntn.size + cs_onnx_tn.size + cs_onnx_ntn.size) \
        else np.array([0.0])

    if args.rsec_max is None:
        p95 = np.percentile(all_cs, 95) if all_cs.size else 0.5
        rsec_max = min(2.0, max(0.5, float(p95)))
    else:
        rsec_max = float(args.rsec_max)

    rsec = np.linspace(0.0, rsec_max, args.rsec_points)

    # SOP curves
    p_wo = combine_domains_sop(cs_wo_tn, cs_wo_ntn, rsec, args.combine)
    p_w = combine_domains_sop(cs_w_tn, cs_w_ntn, rsec, args.combine)
    p_onnx = combine_domains_sop(cs_onnx_tn, cs_onnx_ntn, rsec, args.combine)

    # plot
    plt.figure(figsize=(7.2, 4.4))
    plt.plot(rsec, p_wo, marker="o", markersize=3, linewidth=1.6, label="Without secrecy (baseline)")
    plt.plot(rsec, p_w, marker="o", markersize=3, linewidth=1.6, label="With secrecy xApp")
    plt.plot(rsec, p_onnx, marker="o", markersize=3, linewidth=1.6, label="Secrecy + ONNX ML gate")

    plt.xlabel(r"Target secrecy rate $R_{\mathrm{sec}}$ (bits/s/Hz)")
    ylabel = r"Secrecy outage probability"
    if args.subset == "ho":
        ylabel = r"SOP at HO ${\mathrm{out}}^{\mathrm{HO}}$"
    elif args.subset == "noho":
        ylabel = r"SOP (no HO) ${\mathrm{out}}^{\neg \mathrm{HO}}$"
    plt.ylabel(ylabel)

    plt.ylim(-0.02, 1.02)
    plt.xlim(0.0, rsec_max)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    tw = ""
    if (args.t_min is not None) or (args.t_max is not None):
        tw = f"_t{args.t_min if args.t_min is not None else 'min'}-{args.t_max if args.t_max is not None else 'max'}"

    out_png = args.out_dir / f"sop_vs_rsec_{args.subset}_{args.combine}{tw}.png"
    out_pdf = args.out_dir / f"sop_vs_rsec_{args.subset}_{args.combine}{tw}.pdf"
    plt.savefig(out_png, dpi=200)
    plt.savefig(out_pdf)
    print(f"Saved: {out_png}")
    print(f"Saved: {out_pdf}")

    # print counts (very important to interpret HO-subset)
    def pr(name, a, b):
        print(f"{name}: TN={a.size}, NTN={b.size}, total={a.size+b.size}")

    print(f"subset={args.subset}, combine={args.combine}")
    pr("wo", cs_wo_tn, cs_wo_ntn)
    pr("w", cs_w_tn, cs_w_ntn)
    pr("onnx", cs_onnx_tn, cs_onnx_ntn)

    # quick operating point
    r0 = 0.1
    def p0(cs_tn, cs_ntn):
        if args.combine == "concat":
            cs = np.concatenate([cs_tn, cs_ntn]) if (cs_tn.size or cs_ntn.size) else np.array([], dtype=float)
            return float((cs < r0).mean()) if cs.size else float("nan")
        else:
            vals = []
            if cs_tn.size: vals.append(float((cs_tn < r0).mean()))
            if cs_ntn.size: vals.append(float((cs_ntn < r0).mean()))
            return float(np.mean(vals)) if vals else float("nan")

    print(f"P_out(R_sec=0.1): wo={p0(cs_wo_tn, cs_wo_ntn):.4f}, "
          f"w={p0(cs_w_tn, cs_w_ntn):.4f}, "
          f"onnx={p0(cs_onnx_tn, cs_onnx_ntn):.4f}")


if __name__ == "__main__":
    main()
