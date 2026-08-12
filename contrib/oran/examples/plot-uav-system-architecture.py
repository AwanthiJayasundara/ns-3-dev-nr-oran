#!/usr/bin/env python3
"""Generate system architecture figure for UAV TN-NTN O-RAN service continuity."""

from __future__ import annotations

from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch


OUT_DIR = Path("docs/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def add_box(ax, xy, w, h, text, fc, ec="#1f2937", fontsize=10, lw=1.3, r=0.05):
    box = patches.FancyBboxPatch(
        xy,
        w,
        h,
        boxstyle=f"round,pad=0.018,rounding_size={r}",
        facecolor=fc,
        edgecolor=ec,
        linewidth=lw,
    )
    ax.add_patch(box)
    ax.text(xy[0] + w / 2, xy[1] + h / 2, text, ha="center", va="center", fontsize=fontsize)
    return box


def add_arrow(ax, start, end, color="#1f2937", lw=1.6, ls="-", label=None, label_offset=(0, 0), rad=0.0):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=lw,
        linestyle=ls,
        color=color,
        connectionstyle=f"arc3,rad={rad}",
    )
    ax.add_patch(arrow)
    if label:
        mx = (start[0] + end[0]) / 2 + label_offset[0]
        my = (start[1] + end[1]) / 2 + label_offset[1]
        ax.text(mx, my, label, fontsize=8.5, ha="center", va="center", color=color,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.82, pad=1.2))
    return arrow


def add_hex(ax, center, radius, color, alpha=0.28):
    hexagon = patches.RegularPolygon(center, numVertices=6, radius=radius, orientation=0.52,
                                     facecolor=color, edgecolor="none", alpha=alpha)
    ax.add_patch(hexagon)
    return hexagon


def add_gnb(ax, x, y, label):
    ax.plot([x, x], [y, y + 0.42], color="#1d4ed8", lw=2.0)
    ax.plot([x - 0.18, x, x + 0.18], [y, y + 0.42, y], color="#1d4ed8", lw=1.4)
    ax.plot([x - 0.28, x + 0.28], [y + 0.42, y + 0.42], color="#1d4ed8", lw=1.4)
    ax.text(x, y - 0.12, label, ha="center", va="top", fontsize=8.5,
            bbox=dict(facecolor="white", edgecolor="#cbd5e1", boxstyle="round,pad=0.18"))


def add_uav(ax, x, y, label):
    body = patches.Ellipse((x, y), 0.58, 0.23, facecolor="#111827", edgecolor="black", lw=0.8)
    ax.add_patch(body)
    for dx, dy in [(-0.43, 0.18), (0.43, 0.18), (-0.43, -0.18), (0.43, -0.18)]:
        ax.plot([x, x + dx], [y, y + dy], color="#111827", lw=1.2)
        ax.add_patch(patches.Circle((x + dx, y + dy), 0.12, facecolor="#64748b", edgecolor="black", lw=0.6))
    ax.text(x, y + 0.48, label, ha="center", va="bottom", fontsize=8.5,
            bbox=dict(facecolor="white", edgecolor="#cbd5e1", boxstyle="round,pad=0.18"))


def main() -> None:
    fig, ax = plt.subplots(figsize=(12.6, 6.6))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis("off")

    # Coverage regions
    for x, y, c in [(1.5, 4.0, "#bbf7d0"), (3.5, 4.2, "#bfdbfe"), (6.3, 4.35, "#fde68a"), (8.4, 4.4, "#fde68a")]:
        add_hex(ax, (x, y), 1.35, c, alpha=0.34)
    ax.text(1.15, 5.45, "TN service region", fontsize=10, color="#166534", weight="bold")
    ax.text(6.1, 5.75, "Underserved UE clusters", fontsize=10, color="#92400e", weight="bold")

    # Network nodes
    add_gnb(ax, 1.45, 3.6, "TN gNB 1")
    add_gnb(ax, 3.35, 3.85, "TN gNB 2")
    add_gnb(ax, 9.4, 3.95, "TN donor gNB")
    add_uav(ax, 6.45, 4.65, "UAV-gNB 1")
    add_uav(ax, 8.25, 4.9, "UAV-gNB 2")

    # UEs
    for i, (x, y) in enumerate([(0.9, 3.1), (1.25, 2.75), (1.75, 2.9), (2.1, 3.18)]):
        ax.add_patch(patches.Circle((x, y), 0.09, facecolor="#60a5fa", edgecolor="#1e3a8a", lw=0.5))
    ax.text(1.45, 2.35, "TN-covered UEs", ha="center", fontsize=8.5)
    for x, y in [(5.9, 3.75), (6.25, 3.45), (6.65, 3.55), (7.0, 3.9), (8.1, 3.8), (8.45, 3.55), (8.82, 3.9)]:
        ax.add_patch(patches.Circle((x, y), 0.09, facecolor="#a78bfa", edgecolor="#4c1d95", lw=0.5))
    ax.text(7.35, 3.05, "UEs served by UAV access links", ha="center", fontsize=8.5)

    # Core and satellite path
    core = add_box(ax, (10.65, 0.65), 2.1, 0.75, "5G Core\\n/N3 user plane", "#d1fae5", fontsize=9.5)
    gw = add_box(ax, (11.25, 5.05), 1.75, 0.62, "Satellite\\nGateway", "#e0f2fe", fontsize=9.2)
    sat = add_box(ax, (11.7, 6.75), 1.35, 0.55, "Satellite", "#eef2ff", fontsize=9.5)

    # O-RAN control platform
    cloud = add_box(ax, (3.95, 0.55), 3.45, 1.85, "O-Cloud / MEC", "#e0f2fe", fontsize=10)
    ric = add_box(ax, (4.25, 1.35), 2.85, 0.45, "Near-RT RIC", "#ffffff", fontsize=9.5)
    add_box(ax, (4.15, 0.78), 1.35, 0.42, "Switching xApp\\n(TN/NTN)", "#fef3c7", fontsize=7.8)
    add_box(ax, (5.7, 0.78), 1.35, 0.42, "Handover xApp", "#dcfce7", fontsize=8.2)
    add_box(ax, (2.95, 0.78), 0.82, 0.42, "SMO", "#ccfbf1", fontsize=8.5)

    # User-plane / backhaul routes
    add_arrow(ax, (6.45, 4.45), (9.25, 4.15), color="#2563eb", lw=2.2,
              label="TN wireless backhaul", label_offset=(0.0, 0.32), rad=-0.05)
    add_arrow(ax, (9.4, 3.8), (10.8, 1.42), color="#2563eb", lw=2.0,
              label="TN transport", label_offset=(0.35, -0.05))
    add_arrow(ax, (8.35, 5.08), (11.85, 6.75), color="#0ea5e9", lw=2.3,
              label="SAT fallback", label_offset=(0.15, 0.25), rad=0.08)
    add_arrow(ax, (12.35, 6.75), (12.25, 5.67), color="#0ea5e9", lw=2.0,
              label="Feeder link", label_offset=(0.55, 0.0))
    add_arrow(ax, (12.1, 5.05), (11.65, 1.42), color="#0ea5e9", lw=2.0,
              label="GW to core", label_offset=(0.63, -0.1))

    # Access links
    add_arrow(ax, (6.15, 3.75), (6.4, 4.48), color="#7c3aed", lw=1.5, ls="--",
              label="UAV access", label_offset=(-0.45, 0.0))
    add_arrow(ax, (8.45, 3.8), (8.25, 4.72), color="#7c3aed", lw=1.5, ls="--")

    # E2/control and xApp decision flow
    add_arrow(ax, (5.7, 2.4), (6.28, 4.42), color="#111827", lw=1.5, ls=(0, (3, 3)),
              label="E2 reports/commands", label_offset=(-0.15, 0.05))
    add_arrow(ax, (5.55, 2.4), (3.35, 3.85), color="#111827", lw=1.3, ls=(0, (3, 3)))
    add_arrow(ax, (5.55, 2.4), (9.35, 3.9), color="#111827", lw=1.3, ls=(0, (3, 3)))
    add_arrow(ax, (5.0, 1.2), (5.0, 0.78), color="#92400e", lw=1.2, label="route mode", label_offset=(0.55, 0.0))
    add_arrow(ax, (6.35, 1.2), (6.35, 0.78), color="#166534", lw=1.2, label="UE HO", label_offset=(0.45, 0.0))

    # Degradation annotation
    ax.add_patch(patches.FancyArrowPatch((9.0, 5.35), (8.0, 5.05), arrowstyle="-|>", mutation_scale=16,
                                         linewidth=2.0, color="#ef4444", connectionstyle="arc3,rad=-0.2"))
    ax.text(7.7, 5.55, "TN backhaul may degrade\\nwith UAV distance/fading", ha="center", fontsize=8.8,
            bbox=dict(facecolor="#fee2e2", edgecolor="#ef4444", boxstyle="round,pad=0.25"))

    # Legend
    legend_x, legend_y = 0.55, 6.75
    add_box(ax, (legend_x, legend_y), 4.6, 0.8,
            "Solid blue: user-plane backhaul route    Dashed black: O-RAN E2 control\\n"
            "Switching xApp selects TN direct, satellite fallback, or no backhaul",
            "#ffffff", ec="#cbd5e1", fontsize=8.4, lw=1.0)

    ax.text(7.0, 7.75, "UAV TN-NTN O-RAN System Architecture", ha="center", va="top",
            fontsize=15, weight="bold")

    for ext in ("png", "pdf"):
        fig.savefig(OUT_DIR / f"uav_tn_ntn_oran_system_architecture.{ext}", dpi=300)
    plt.close(fig)
    print("[saved]", OUT_DIR / "uav_tn_ntn_oran_system_architecture.png")
    print("[saved]", OUT_DIR / "uav_tn_ntn_oran_system_architecture.pdf")


if __name__ == "__main__":
    main()
