#!/usr/bin/env python3
"""
Train UAV repositioning predictors from ns-3 TN/NTN O-RAN simulation outputs.

The script converts UE position traces and O-RAN/ML handover logs into a time
series of heatmaps. Each heatmap marks where underserved UEs are located at a
given simulation time. The models learn this task:

    previous L underserved-UE heatmaps -> next underserved-UE heatmap

The predicted heatmap can then guide where an aerial UAV gNB should move to
support overloaded or weak-coverage areas.
"""
from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset


# ---------------------------------------------------------------------
# Grid definition
# ---------------------------------------------------------------------
@dataclass
class GridSpec:
    """
    Local rectangular grid used to convert UE positions into heatmap cells.

    east/north are measured in meters after converting latitude/longitude
    around the reference point. nx and ny control the heatmap resolution.
    """
    east_min: float = -3000.0
    east_max: float = 3000.0
    north_min: float = -1500.0
    north_max: float = 1500.0
    nx: int = 24
    ny: int = 12

    @property
    def n_cells(self) -> int:
        return self.nx * self.ny


# ---------------------------------------------------------------------
# Input parsing
# ---------------------------------------------------------------------
def parse_position_trace(path: Path, prefix: str) -> pd.DataFrame:
    """
    Read one UE position trace file.

    Expected trace format per line:
        time node latitude longitude altitude

    The prefix records whether the row came from UE set 1 or UE set 2.
    """
    rows: List[Dict[str, object]] = []

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue

            rows.append(
                {
                    "time": float(parts[0]),
                    "node": parts[1],
                    "lat": float(parts[2]),
                    "lon": float(parts[3]),
                    "alt": float(parts[4]),
                    "group": prefix,
                }
            )

    return pd.DataFrame(rows)


def geo_to_local(df: pd.DataFrame, ref_lat: float, ref_lon: float) -> pd.DataFrame:
    """
    Convert latitude/longitude into local east/north meter coordinates.

    This keeps the learning problem in meters instead of GPS degrees, making
    the later grid/heatmap construction easier to interpret.
    """
    out = df.copy()

    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * np.cos(np.deg2rad(ref_lat))

    out["east_m"] = (out["lon"] - ref_lon) * meters_per_deg_lon
    out["north_m"] = (out["lat"] - ref_lat) * meters_per_deg_lat

    return out


def infer_first_ues1_node_id(ml_df: pd.DataFrame) -> int:
    """Infer the first UE node id from ml-ho-dataset.csv."""
    return int(ml_df["ueId"].min())


def infer_first_imsi_s1(log_path: Path) -> int:
    """
    Infer the first S1 UE IMSI from the initial-attach log.

    Logs use IMSIs while position traces use names like UE0 and UE_S2_0, so
    this value is needed to map log events back to trace node names.
    """
    pat = re.compile(r"^---- INIT_ATTACH t=.* UE\(imsi\)=([0-9]+) ----$")

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pat.match(line.strip())
            if m:
                return int(m.group(1))

    raise RuntimeError("Could not infer first S1 IMSI from the log.")


def build_node_id_to_trace_name(
    num_ues1: int,
    num_ues2: int,
    first_ues1_node_id: int,
) -> Dict[int, str]:
    """Map ns-3 node IDs from the ML dataset to position-trace UE names."""
    mapping: Dict[int, str] = {}

    for i in range(num_ues1):
        mapping[first_ues1_node_id + i] = f"UE{i}"

    first_ues2_node_id = first_ues1_node_id + num_ues1

    for i in range(num_ues2):
        mapping[first_ues2_node_id + i] = f"UE_S2_{i}"

    return mapping


def build_imsi_to_trace_name(
    num_ues1: int,
    num_ues2: int,
    first_imsi_s1: int,
) -> Dict[int, str]:
    """Map IMSIs from ns3-oran-lm.log to position-trace UE names."""
    mapping: Dict[int, str] = {}

    for i in range(num_ues1):
        mapping[first_imsi_s1 + i] = f"UE{i}"

    first_imsi_s2 = first_imsi_s1 + num_ues1

    for i in range(num_ues2):
        mapping[first_imsi_s2 + i] = f"UE_S2_{i}"

    return mapping


# ---------------------------------------------------------------------
# Underserved UE detection
# ---------------------------------------------------------------------
def parse_log_underserved_events(
    log_path: Path,
    imsi_to_node: Dict[int, str],
) -> Dict[float, Set[str]]:
    """
    Extract underserved UE events from the O-RAN log.

    A UE is marked underserved if initial attachment failed because all
    candidate cells had low RSRP or all acceptable cells were full.
    """
    events: Dict[float, Set[str]] = {}

    current_time: Optional[float] = None
    current_imsi: Optional[int] = None

    head_pat = re.compile(
        r"^---- INIT_ATTACH t=([0-9.+-eE]+) UE\(imsi\)=([0-9]+) ----$"
    )

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()

            m = head_pat.match(line)
            if m:
                current_time = float(m.group(1))
                current_imsi = int(m.group(2))
                continue

            if current_time is None or current_imsi is None:
                continue

            if (
                "INIT_ATTACH_FAIL_ALL_LOW_RSRP" in line
                or "INIT_ATTACH_FAIL_ALL_FULL" in line
            ):
                # This failure line belongs to the most recent INIT_ATTACH header.
                node_name = imsi_to_node.get(current_imsi)

                if node_name is not None:
                    events.setdefault(current_time, set()).add(node_name)

                current_time = None
                current_imsi = None

            elif "INIT_ATTACH_OK" in line:
                current_time = None
                current_imsi = None

    return events


def parse_ml_underserved_events(
    ml_df: pd.DataFrame,
    node_id_to_node: Dict[int, str],
    rsrp_thresh_dbm: float,
) -> Dict[float, Set[str]]:
    """
    Extract underserved UE events from ml-ho-dataset.csv.

    The CSV has one row per candidate cell, so servingRsrp is repeated for each
    (time, UE) group. We collapse that to one serving RSRP per UE/time and mark
    the UE as underserved when it falls below the configured threshold.
    """
    # One row per candidate, but servingRsrp is repeated in a (time, ueId) group.
    grp = (
        ml_df.groupby(["time", "ueId"], as_index=False)
        .agg(servingRsrp=("servingRsrp", "first"))
    )

    grp = grp[grp["servingRsrp"] < rsrp_thresh_dbm].copy()

    events: Dict[float, Set[str]] = {}

    for _, row in grp.iterrows():
        node_name = node_id_to_node.get(int(row["ueId"]))

        if node_name is None:
            continue

        events.setdefault(float(row["time"]), set()).add(node_name)

    return events


def merge_event_maps(*maps: Dict[float, Set[str]]) -> Dict[float, Set[str]]:
    """Merge several {time -> underserved UE set} dictionaries."""
    out: Dict[float, Set[str]] = {}

    for mp in maps:
        for t, nodes in mp.items():
            out.setdefault(t, set()).update(nodes)

    return out


def align_events_to_position_times(
    events: Dict[float, Set[str]],
    position_times: np.ndarray,
    max_time_gap: float,
) -> Tuple[Dict[float, Set[str]], Dict[str, float]]:
    """
    Map decision/log event times to the closest available position-trace time.

    The ns-3 decision CSV can be sampled at times such as 12.5 s while position
    traces are sampled at 12 s or 15 s. Exact timestamp matching would silently
    drop those events, producing incomplete heatmaps.
    """
    if len(position_times) == 0:
        raise RuntimeError("No position trace times are available for event alignment.")

    sorted_times = np.sort(position_times.astype(float))
    aligned: Dict[float, Set[str]] = {}
    dropped_events = 0
    max_observed_gap = 0.0

    for event_time, nodes in events.items():
        # searchsorted gives the insertion point in the sorted position
        # timeline. The nearest available trace time is either that element or
        # the one immediately before it.
        insert_idx = int(np.searchsorted(sorted_times, event_time))
        candidates: List[float] = []

        if insert_idx < len(sorted_times):
            candidates.append(float(sorted_times[insert_idx]))
        if insert_idx > 0:
            candidates.append(float(sorted_times[insert_idx - 1]))

        if not candidates:
            dropped_events += len(nodes)
            continue

        nearest_time = min(candidates, key=lambda t: abs(t - event_time))
        gap = abs(nearest_time - event_time)
        max_observed_gap = max(max_observed_gap, gap)

        if gap > max_time_gap:
            dropped_events += len(nodes)
            continue

        aligned.setdefault(nearest_time, set()).update(nodes)

    stats = {
        "raw_event_times": float(len(events)),
        "aligned_event_times": float(len(aligned)),
        "dropped_node_events": float(dropped_events),
        "max_observed_alignment_gap_s": float(max_observed_gap),
        "max_allowed_alignment_gap_s": float(max_time_gap),
    }

    return aligned, stats


# ---------------------------------------------------------------------
# Heatmap construction
# ---------------------------------------------------------------------
def build_underserved_heatmaps(
    pos_df: pd.DataFrame,
    underserved_events: Dict[float, Set[str]],
    grid: GridSpec,
) -> Tuple[np.ndarray, np.ndarray, List[int]]:
    """
    Build one 2D underserved-UE heatmap per position-trace timestamp.

    Each cell stores the count of underserved UEs located inside that grid cell.
    Empty heatmaps are kept because "no underserved UEs now" is also a useful
    training target.
    """
    times = np.sort(pos_df["time"].unique())

    x_edges = np.linspace(grid.east_min, grid.east_max, grid.nx + 1)
    y_edges = np.linspace(grid.north_min, grid.north_max, grid.ny + 1)

    heatmaps: List[np.ndarray] = []
    kept_times: List[float] = []
    underserved_counts: List[int] = []

    for t in times:
        # Start with all UE positions at this timestamp, then keep only the UEs
        # that were classified as underserved at the aligned event time.
        snap = pos_df[np.isclose(pos_df["time"], t)].copy()
        nodes = underserved_events.get(float(t), set())

        if nodes:
            snap = snap[snap["node"].isin(nodes)].copy()
        else:
            snap = snap.iloc[0:0].copy()

        east = snap["east_m"].to_numpy() if not snap.empty else np.array([], dtype=np.float32)
        north = snap["north_m"].to_numpy() if not snap.empty else np.array([], dtype=np.float32)

        heatmap, _, _ = np.histogram2d(
            north,
            east,
            bins=[y_edges, x_edges],
        )

        heatmaps.append(heatmap.astype(np.float32))
        kept_times.append(float(t))
        underserved_counts.append(len(nodes))

    if not heatmaps:
        raise RuntimeError("No underserved heatmaps could be built.")

    return np.stack(heatmaps, axis=0), np.array(kept_times, dtype=np.float32), underserved_counts


# ---------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------
class HeatmapSequenceDataset(Dataset):
    """
    PyTorch dataset for sequence-to-next-heatmap prediction.

    For a target index y_idx, x is the previous `lookback` heatmaps and y is the
    heatmap at y_idx.
    """
    def __init__(self, heatmaps: np.ndarray, lookback: int, target_indices: np.ndarray):
        self.heatmaps = heatmaps.astype(np.float32)
        self.lookback = lookback
        self.target_indices = target_indices.astype(int)

    def __len__(self) -> int:
        return len(self.target_indices)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        y_idx = int(self.target_indices[idx])
        x = self.heatmaps[y_idx - self.lookback : y_idx]
        y = self.heatmaps[y_idx]

        return torch.from_numpy(x), torch.from_numpy(y)


# ---------------------------------------------------------------------
# Predictor models
# ---------------------------------------------------------------------
class PersistencePredictor(nn.Module):
    """
    Non-ML baseline:
    Predicts the next heatmap as the latest observed heatmap.
    """
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x[:, -1, :, :]


class HeatmapMLP(nn.Module):
    """
    MLP predictor:
    Uses flattened spatio-temporal heatmap history.
    """
    def __init__(self, lookback: int, n_features: int, hidden_size: int = 128):
        super().__init__()

        # Flatten all lookback heatmaps and learn a direct mapping to the next
        # heatmap. Softplus keeps predicted cell counts non-negative.
        self.net = nn.Sequential(
            nn.Linear(lookback * n_features, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, n_features),
            nn.Softplus(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Input shape: batch x lookback x ny x nx.
        # The MLP expects a single vector per sample.
        b, t, ny, nx = x.shape
        x = x.reshape(b, t * ny * nx)
        pred = self.net(x)

        return pred.reshape(b, ny, nx)


class HeatmapCNN(nn.Module):
    """
    CNN-based spatial predictor:
    Treats the L heatmaps as channels and learns spatial hotspot structure.
    """
    def __init__(self, lookback: int):
        super().__init__()

        # Treat the lookback dimension as image channels so the CNN can learn
        # local spatial patterns, such as nearby underserved hotspots.
        self.net = nn.Sequential(
            nn.Conv2d(lookback, 32, kernel_size=3, padding=1),
            nn.LeakyReLU(0.1),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.LeakyReLU(0.1),
            nn.Conv2d(32, 1, kernel_size=1),
            nn.Softplus(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Conv2d returns batch x 1 x ny x nx; squeeze removes the channel.
        pred = self.net(x)
        return pred.squeeze(1)


class HeatmapGRU(nn.Module):
    """
    GRU-based temporal predictor:
    Learns temporal evolution of underserved-user heatmaps.
    """
    def __init__(self, n_features: int, hidden_size: int = 128, num_layers: int = 2):
        super().__init__()

        # Each heatmap is flattened into one feature vector per time step. The
        # GRU learns how the hotspot pattern evolves over the lookback window.
        self.gru = nn.GRU(
            input_size=n_features,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
        )

        self.head = nn.Sequential(
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, n_features),
            nn.Softplus(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Shape becomes batch x lookback x flattened_grid before entering GRU.
        b, t, ny, nx = x.shape
        x = x.reshape(b, t, ny * nx)

        out, _ = self.gru(x)
        last = out[:, -1, :]

        pred = self.head(last)

        return pred.reshape(b, ny, nx)


# ---------------------------------------------------------------------
# Training and evaluation
# ---------------------------------------------------------------------
def has_trainable_parameters(model: nn.Module) -> bool:
    """Return False for the persistence baseline, True for neural models."""
    return any(p.requires_grad for p in model.parameters())


def run_epoch(
    model: nn.Module,
    loader: DataLoader,
    optimizer: Optional[torch.optim.Optimizer],
    device: torch.device,
    train: bool,
) -> float:
    """
    Run one training or evaluation pass and return average weighted MSE.

    Positive target cells get larger weight so the model pays more attention to
    underserved hotspots instead of learning that mostly-empty heatmaps are easy
    to predict.
    """
    def weighted_mse_loss(pred: torch.Tensor, target: torch.Tensor, alpha: float = 5.0) -> torch.Tensor:
        weights = 1.0 + alpha * (target > 0).float()
        return torch.mean(weights * (pred - target) ** 2)

    model.train(train)

    total_loss = 0.0
    n_batches = 0

    context = torch.enable_grad() if train else torch.no_grad()

    with context:
        for xb, yb in loader:
            xb = xb.to(device)
            yb = yb.to(device)

            if train:
                if optimizer is None:
                    raise RuntimeError("Optimizer is required in training mode.")
                optimizer.zero_grad()

            pred = model(xb)
            loss = weighted_mse_loss(pred, yb, alpha=5.0)

            if train:
                loss.backward()
                optimizer.step()

            total_loss += float(loss.item())
            n_batches += 1

    return total_loss / max(n_batches, 1)


def heatmap_centroid_m(
    hm: np.ndarray,
    grid: GridSpec,
) -> Optional[Tuple[float, float]]:
    """
    Compute the weighted hotspot centroid in meters.

    Returns None for an empty heatmap because there is no hotspot location to
    compare.
    """
    total = float(hm.sum())

    if total <= 0:
        return None

    x_edges = np.linspace(grid.east_min, grid.east_max, grid.nx + 1)
    y_edges = np.linspace(grid.north_min, grid.north_max, grid.ny + 1)

    x_centers = 0.5 * (x_edges[:-1] + x_edges[1:])
    y_centers = 0.5 * (y_edges[:-1] + y_edges[1:])

    yy, xx = np.meshgrid(y_centers, x_centers, indexing="ij")

    east = float((hm * xx).sum() / total)
    north = float((hm * yy).sum() / total)

    return east, north


@torch.no_grad()
def evaluate_model(
    model: nn.Module,
    loader: DataLoader,
    device: torch.device,
    grid: GridSpec,
    scale: float,
    hotspot_k: int,
) -> Dict[str, float]:
    """
    Evaluate a trained predictor on the held-out test loader.

    Metrics include cell-count errors, hotspot top-k overlap, and centroid
    distance. Predictions are scaled back from normalized values to original
    UE-count units before metrics are computed.
    """
    model.eval()

    preds: List[torch.Tensor] = []
    targets: List[torch.Tensor] = []

    for xb, yb in loader:
        xb = xb.to(device)

        pred = model(xb).cpu()
        pred = torch.clamp(pred, min=0.0)

        preds.append(pred)
        targets.append(yb.cpu())

    pred_all = torch.cat(preds, dim=0) * scale
    target_all = torch.cat(targets, dim=0) * scale

    diff = pred_all - target_all

    mse = float(torch.mean(diff ** 2).item())
    rmse = float(np.sqrt(mse))
    mae = float(torch.mean(torch.abs(diff)).item())

    pred_counts = pred_all.sum(dim=(1, 2))
    true_counts = target_all.sum(dim=(1, 2))
    count_mae = float(torch.mean(torch.abs(pred_counts - true_counts)).item())

    pred_np = pred_all.numpy()
    true_np = target_all.numpy()

    hit_list: List[float] = []
    precision_list: List[float] = []
    recall_list: List[float] = []
    centroid_errors: List[float] = []

    for p, y in zip(pred_np, true_np):
        p_flat = p.reshape(-1)
        y_flat = y.reshape(-1)

        true_nonzero = np.where(y_flat > 0)[0]

        if len(true_nonzero) == 0:
            # Hotspot metrics are undefined when the true heatmap has no
            # underserved users, so skip those samples for top-k/centroid stats.
            continue

        true_k = min(hotspot_k, len(true_nonzero))

        true_top = set(np.argsort(y_flat)[::-1][:true_k])
        pred_top = set(np.argsort(p_flat)[::-1][:hotspot_k])

        inter = true_top.intersection(pred_top)

        hit_list.append(1.0 if len(inter) > 0 else 0.0)
        precision_list.append(len(inter) / max(hotspot_k, 1))
        recall_list.append(len(inter) / max(len(true_top), 1))

        c_true = heatmap_centroid_m(y, grid)
        c_pred = heatmap_centroid_m(p, grid)

        if c_true is not None and c_pred is not None:
            centroid_errors.append(
                float(np.linalg.norm(np.array(c_true) - np.array(c_pred)))
            )

    return {
        "mse": mse,
        "rmse": rmse,
        "mae": mae,
        "count_mae": count_mae,
        "hit_at_k": float(np.mean(hit_list)) if hit_list else np.nan,
        "precision_at_k": float(np.mean(precision_list)) if precision_list else np.nan,
        "recall_at_k": float(np.mean(recall_list)) if recall_list else np.nan,
        "centroid_error_m": float(np.mean(centroid_errors)) if centroid_errors else np.nan,
    }


def train_single_model(
    model_name: str,
    model: nn.Module,
    train_loader: DataLoader,
    val_loader: DataLoader,
    test_loader: DataLoader,
    device: torch.device,
    grid: GridSpec,
    scale: float,
    hotspot_k: int,
    epochs: int,
    lr: float,
    outdir: Path,
) -> Tuple[nn.Module, Dict[str, float], List[Dict[str, float]]]:
    """
    Train one model, select the best epoch using validation loss, and evaluate
    only once on the final held-out test split.
    """
    model = model.to(device)

    history: List[Dict[str, float]] = []

    if has_trainable_parameters(model):
        optimizer = torch.optim.Adam(model.parameters(), lr=lr)

        best_val_loss = float("inf")
        best_state = None

        for epoch in range(1, epochs + 1):
            train_loss = run_epoch(model, train_loader, optimizer, device, train=True)
            val_loss = run_epoch(model, val_loader, None, device, train=False)

            history.append(
                {
                    "model": model_name,
                    "epoch": epoch,
                    "train_loss": train_loss,
                    "val_loss": val_loss,
                }
            )

            print(
                f"{model_name:12s} | Epoch {epoch:03d} | "
                f"train_loss={train_loss:.6f} | val_loss={val_loss:.6f}"
            )

            if val_loss < best_val_loss:
                # Keep the best validation checkpoint in memory. This prevents
                # the final test result from depending on an overfit last epoch.
                best_val_loss = val_loss
                best_state = {
                    k: v.detach().cpu().clone()
                    for k, v in model.state_dict().items()
                }

        if best_state is not None:
            model.load_state_dict(best_state)

        torch.save(model.state_dict(), outdir / f"{model_name}_state_dict.pt")

    else:
        # The persistence baseline has no parameters, but evaluating it on the
        # validation split keeps the training-history table consistent.
        val_loss = run_epoch(model, val_loader, None, device, train=False)

        history.append(
            {
                "model": model_name,
                "epoch": 0,
                "train_loss": np.nan,
                "val_loss": val_loss,
            }
        )

        print(f"{model_name:12s} | baseline_val_loss={val_loss:.6f}")

    metrics = evaluate_model(
        model=model,
        loader=test_loader,
        device=device,
        grid=grid,
        scale=scale,
        hotspot_k=hotspot_k,
    )

    metrics["model"] = model_name

    return model.cpu(), metrics, history


# ---------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------
def plot_metric_bar(
    df: pd.DataFrame,
    metric: str,
    title: str,
    ylabel: str,
    out_path: Path,
    lower_is_better: bool = True,
) -> None:
    """Save a simple bar chart for one comparison metric."""
    plot_df = df[["model", metric]].copy()
    plot_df = plot_df.sort_values(metric, ascending=lower_is_better)

    plt.figure(figsize=(7, 4))
    plt.bar(plot_df["model"], plot_df[metric])
    plt.title(title)
    plt.xlabel("Predictor")
    plt.ylabel(ylabel)
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)
    plt.close()


def plot_hotspot_metrics(df: pd.DataFrame, out_path: Path) -> None:
    """Save grouped bars for Hit@K, Precision@K, and Recall@K."""
    metrics = ["hit_at_k", "precision_at_k", "recall_at_k"]

    x = np.arange(len(df["model"]))
    width = 0.25

    plt.figure(figsize=(8, 4))

    for i, metric in enumerate(metrics):
        plt.bar(x + i * width, df[metric], width=width, label=metric)

    plt.xticks(x + width, df["model"])
    plt.ylim(0, 1.05)
    plt.title("Hotspot Prediction Metrics")
    plt.xlabel("Predictor")
    plt.ylabel("Score")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)
    plt.close()


def plot_training_curves(history_df: pd.DataFrame, out_path: Path) -> None:
    """Save training/validation loss curves for the trainable neural models."""
    trainable = history_df[history_df["epoch"] > 0].copy()

    if trainable.empty:
        return

    plt.figure(figsize=(8, 4))

    for model_name, g in trainable.groupby("model"):
        plt.plot(g["epoch"], g["val_loss"], label=f"{model_name} val")
        plt.plot(g["epoch"], g["train_loss"], linestyle="--", label=f"{model_name} train")

    plt.title("Training and Validation Loss")
    plt.xlabel("Epoch")
    plt.ylabel("Weighted MSE Loss")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)
    plt.close()


def plot_best_summary(df: pd.DataFrame, out_path: Path) -> None:
    """Save a compact figure naming the best model under key metrics."""
    best_rmse = df.sort_values("rmse", ascending=True).iloc[0]
    best_hit = df.sort_values("hit_at_k", ascending=False).iloc[0]
    best_centroid = df.sort_values("centroid_error_m", ascending=True).iloc[0]

    text = (
        "Best Predictor Summary\n\n"
        f"Lowest RMSE: {best_rmse['model']} ({best_rmse['rmse']:.4f})\n"
        f"Highest Hit@K: {best_hit['model']} ({best_hit['hit_at_k']:.4f})\n"
        f"Lowest centroid error: {best_centroid['model']} "
        f"({best_centroid['centroid_error_m']:.2f} m)\n"
    )

    plt.figure(figsize=(7, 3))
    plt.axis("off")
    plt.text(0.05, 0.75, text, fontsize=12, va="top")
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)
    plt.close()


# ---------------------------------------------------------------------
# Optional ONNX export
# ---------------------------------------------------------------------
def export_best_model_to_onnx(
    model_name: str,
    model: nn.Module,
    lookback: int,
    ny: int,
    nx: int,
    outdir: Path,
) -> None:
    """
    Export the selected neural model to ONNX for possible ns-3 / xApp use.

    The persistence baseline is skipped because it has no learned parameters.
    """
    if model_name == "persistence":
        print("Skipping ONNX export for persistence baseline.")
        return

    import onnx

    model.eval()

    dummy = torch.zeros(1, lookback, ny, nx, dtype=torch.float32)

    # Main exported model
    onnx_path = outdir / f"best_{model_name}_predictor.onnx"

    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        input_names=["heatmap_sequence"],
        output_names=["predicted_heatmap"],
        dynamic_axes={
            "heatmap_sequence": {0: "batch"},
            "predicted_heatmap": {0: "batch"},
        },
        opset_version=13,
        export_params=True,
        do_constant_folding=True,
        training=torch.onnx.TrainingMode.EVAL,
        dynamo=False,
    )

    m = onnx.load(str(onnx_path))

    print("Exported ONNX model:", onnx_path)
    print("ONNX IR version:", m.ir_version)
    print("ONNX opset:", [x.version for x in m.opset_import])

    # Compatibility filename expected by your ns-3 / ONNX Runtime setup
    compat_path = outdir / "uav_underserved_heatmap_gru_ir8.onnx"

    if model_name == "gru":
        if m.ir_version <= 8:
            onnx.save(m, str(compat_path))
            print("Saved IR8-compatible GRU model:", compat_path)
        else:
            print("\nWARNING:")
            print(f"The exported ONNX model has IR version {m.ir_version}, not IR8.")
            print("Do not simply rename it as *_ir8.onnx.")
            print("Your C++ ONNX Runtime may fail to load it if it only supports IR8.")
            print("Use an older ONNX Python package or upgrade ONNX Runtime.")


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main() -> None:
    # Command-line options make the script reusable for different ns-3 result
    # folders, grid sizes, split ratios, and model hyperparameters.
    parser = argparse.ArgumentParser(
        description="Train and compare heatmap predictors for UAV repositioning."
    )

    parser.add_argument("--ues1", type=Path, default=Path("results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ues1-position-trace.tr"))
    parser.add_argument("--ues2", type=Path, default=Path("results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ues2-position-trace.tr"))
    parser.add_argument("--ml-csv", type=Path, default=Path("results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ml-ho-dataset.csv"), help="ml-ho-dataset.csv")
    parser.add_argument("--log", type=Path, default=Path("results/nr/tn-ntn/ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ns3-oran-lm.log"), help="ns3-oran-lm.log")

    parser.add_argument("--outdir", type=Path, default=Path("results/nr/tn-ntn/ml_uav_final"))

    parser.add_argument("--ref-lat", type=float, default=53.3498)
    parser.add_argument("--ref-lon", type=float, default=-6.2603)

    parser.add_argument("--east-min", type=float, default=-3000.0)
    parser.add_argument("--east-max", type=float, default=3000.0)
    parser.add_argument("--north-min", type=float, default=-1500.0)
    parser.add_argument("--north-max", type=float, default=1500.0)

    parser.add_argument("--nx", type=int, default=24)
    parser.add_argument("--ny", type=int, default=12)

    parser.add_argument("--lookback", type=int, default=4)
    parser.add_argument("--hotspot-k", type=int, default=5)

    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--hidden-size", type=int, default=128)
    parser.add_argument("--num-layers", type=int, default=2)
    parser.add_argument("--lr", type=float, default=1e-3)

    parser.add_argument("--train-ratio", type=float, default=0.60)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--rsrp-thresh", type=float, default=-120.0)
    parser.add_argument(
        "--event-time-tolerance",
        type=float,
        default=2.5,
        help="Maximum seconds allowed when aligning event times to position trace times.",
    )

    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--export-onnx", action="store_true")
    parser.add_argument(
        "--export-model",
        choices=["selected", "mlp", "cnn", "gru"],
        default="selected",
        help="Which trained model to export when --export-onnx is used.",
    )

    args = parser.parse_args()

    # Fix random seeds so repeated runs are comparable.
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    args.outdir.mkdir(parents=True, exist_ok=True)

    # -------------------------------------------------------------
    # Load position traces
    # -------------------------------------------------------------
    # UES1 and UES2 are loaded separately, then merged into one position table.
    # The model does not need to know which set a UE came from; it only needs
    # the location of underserved users over time.
    pos1 = parse_position_trace(args.ues1, "UES1")
    pos2 = parse_position_trace(args.ues2, "UES2")

    pos_df = pd.concat([pos1, pos2], ignore_index=True)
    pos_df = geo_to_local(pos_df, args.ref_lat, args.ref_lon)

    # -------------------------------------------------------------
    # Load ML handover dataset and log
    # -------------------------------------------------------------
    # The ML CSV provides per-UE serving RSRP and candidate-cell information.
    # The log provides initial-attach failures. Both are used as evidence that
    # a UE is underserved.
    ml_df = pd.read_csv(args.ml_csv)

    num_ues1 = pos1["node"].nunique()
    num_ues2 = pos2["node"].nunique()

    first_ues1_node_id = infer_first_ues1_node_id(ml_df)
    first_imsi_s1 = infer_first_imsi_s1(args.log)

    # Build ID mappings so all event sources can refer to the same trace names.
    node_id_to_node = build_node_id_to_trace_name(
        num_ues1=num_ues1,
        num_ues2=num_ues2,
        first_ues1_node_id=first_ues1_node_id,
    )

    imsi_to_node = build_imsi_to_trace_name(
        num_ues1=num_ues1,
        num_ues2=num_ues2,
        first_imsi_s1=first_imsi_s1,
    )

    log_events = parse_log_underserved_events(args.log, imsi_to_node)
    ml_events = parse_ml_underserved_events(
        ml_df=ml_df,
        node_id_to_node=node_id_to_node,
        rsrp_thresh_dbm=args.rsrp_thresh,
    )

    # Merge underserved events from both sources, then align their timestamps
    # to the nearest position trace. This avoids losing ML events sampled at
    # times such as 12.5 s when positions are sampled at integer seconds.
    raw_underserved_events = merge_event_maps(log_events, ml_events)
    position_times = np.sort(pos_df["time"].unique())
    underserved_events, alignment_stats = align_events_to_position_times(
        events=raw_underserved_events,
        position_times=position_times,
        max_time_gap=args.event_time_tolerance,
    )

    print("Event time alignment:")
    print(f"  Raw event times: {int(alignment_stats['raw_event_times'])}")
    print(f"  Aligned event times: {int(alignment_stats['aligned_event_times'])}")
    print(f"  Dropped node-events: {int(alignment_stats['dropped_node_events'])}")
    print(
        "  Max observed gap: "
        f"{alignment_stats['max_observed_alignment_gap_s']:.3f} s "
        f"(allowed {alignment_stats['max_allowed_alignment_gap_s']:.3f} s)"
    )

    # -------------------------------------------------------------
    # Build heatmaps
    # -------------------------------------------------------------
    # The grid converts scattered UE locations into a fixed-size tensor:
    # time x north_cells x east_cells.
    grid = GridSpec(
        east_min=args.east_min,
        east_max=args.east_max,
        north_min=args.north_min,
        north_max=args.north_max,
        nx=args.nx,
        ny=args.ny,
    )

    heatmaps, times, underserved_counts = build_underserved_heatmaps(
        pos_df=pos_df,
        underserved_events=underserved_events,
        grid=grid,
    )

    print(f"Built heatmaps: {heatmaps.shape}")
    print(f"Total time steps: {len(heatmaps)}")
    print(f"Total underserved events counted in heatmaps: {int(np.sum(underserved_counts))}")

    # -------------------------------------------------------------
    # Chronological train/validation/test split
    # -------------------------------------------------------------
    # Keep time ordering. Random splitting would leak future movement/coverage
    # information into training and make the test result too optimistic.
    if args.train_ratio <= 0 or args.val_ratio < 0 or args.train_ratio + args.val_ratio >= 1.0:
        raise RuntimeError(
            "--train-ratio must be > 0, --val-ratio must be >= 0, "
            "and train + validation must leave a non-empty test period."
        )

    train_end = int(len(heatmaps) * args.train_ratio)
    val_end = int(len(heatmaps) * (args.train_ratio + args.val_ratio))

    # Make sure each split remains non-empty after accounting for lookback.
    train_end = max(args.lookback + 1, train_end)
    train_end = min(train_end, len(heatmaps) - 2)

    val_end = max(train_end + 1, val_end)
    val_end = min(val_end, len(heatmaps) - 1)

    train_raw = heatmaps[:train_end]

    # Normalization must use only the training period to avoid leaking future
    # test-set scale information into the model.
    max_count = float(np.max(train_raw)) if np.max(train_raw) > 0 else 1.0
    heatmaps_norm = heatmaps / max_count

    # Each target heatmap is predicted from the previous L heatmaps.
    # Training targets are inside the training period.
    # Validation targets tune model selection.
    # Testing targets are held out for final reporting only.
    train_target_indices = np.arange(args.lookback, train_end)
    val_target_indices = np.arange(train_end, val_end)
    test_target_indices = np.arange(val_end, len(heatmaps))

    train_ds = HeatmapSequenceDataset(heatmaps_norm, args.lookback, train_target_indices)
    val_ds = HeatmapSequenceDataset(heatmaps_norm, args.lookback, val_target_indices)
    test_ds = HeatmapSequenceDataset(heatmaps_norm, args.lookback, test_target_indices)

    if len(train_ds) == 0 or len(val_ds) == 0 or len(test_ds) == 0:
        raise RuntimeError(
            "Not enough heatmap time steps to create train/validation/test sequences. "
            "Try reducing --lookback."
        )

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
    )

    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
    )

    test_loader = DataLoader(
        test_ds,
        batch_size=args.batch_size,
        shuffle=False,
    )

    print(f"Training heatmaps: {train_end}")
    print(f"Validation heatmaps: {val_end - train_end}")
    print(f"Testing heatmaps: {len(heatmaps) - val_end}")
    print(f"Training samples: {len(train_ds)}")
    print(f"Validation samples: {len(val_ds)}")
    print(f"Testing samples: {len(test_ds)}")
    print(f"Normalization max count from training set: {max_count}")

    # -------------------------------------------------------------
    # Define predictor categories
    # -------------------------------------------------------------
    # Compare a no-learning baseline against three neural predictors. The
    # baseline is important: a neural model is only useful if it beats "same as
    # the latest heatmap."
    models: Dict[str, nn.Module] = {
        "persistence": PersistencePredictor(),
        "mlp": HeatmapMLP(
            lookback=args.lookback,
            n_features=grid.n_cells,
            hidden_size=args.hidden_size,
        ),
        "cnn": HeatmapCNN(
            lookback=args.lookback,
        ),
        "gru": HeatmapGRU(
            n_features=grid.n_cells,
            hidden_size=args.hidden_size,
            num_layers=args.num_layers,
        ),
    }

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    comparison_rows: List[Dict[str, float]] = []
    all_history: List[Dict[str, float]] = []
    trained_models: Dict[str, nn.Module] = {}

    # -------------------------------------------------------------
    # Train and evaluate all models
    # -------------------------------------------------------------
    # Each neural model is selected by validation loss, then evaluated once on
    # the held-out test period. The persistence baseline is evaluated directly.
    for model_name, model in models.items():
        print("\n" + "=" * 70)
        print(f"Model: {model_name}")
        print("=" * 70)

        trained_model, metrics, history = train_single_model(
            model_name=model_name,
            model=model,
            train_loader=train_loader,
            val_loader=val_loader,
            test_loader=test_loader,
            device=device,
            grid=grid,
            scale=max_count,
            hotspot_k=args.hotspot_k,
            epochs=args.epochs,
            lr=args.lr,
            outdir=args.outdir,
        )

        trained_models[model_name] = trained_model
        comparison_rows.append(metrics)
        all_history.extend(history)

        print(f"\nFinal testing metrics for {model_name}:")
        for k, v in metrics.items():
            if k != "model":
                print(f"  {k}: {v}")

    # -------------------------------------------------------------
    # Save comparison results
    # -------------------------------------------------------------
    # These CSVs are the main numeric outputs for comparing predictors.
    comparison_df = pd.DataFrame(comparison_rows)

    comparison_df = comparison_df[
        [
            "model",
            "mse",
            "rmse",
            "mae",
            "count_mae",
            "hit_at_k",
            "precision_at_k",
            "recall_at_k",
            "centroid_error_m",
        ]
    ]

    comparison_df.to_csv(args.outdir / "predictor_comparison.csv", index=False)

    history_df = pd.DataFrame(all_history)
    history_df.to_csv(args.outdir / "training_history_all_models.csv", index=False)

    # -------------------------------------------------------------
    # Determine best model
    # -------------------------------------------------------------
    # Different metrics answer different questions:
    # RMSE/MAE: cell-count accuracy.
    # Hit@K: whether the predicted hotspot overlaps the true hotspot.
    # Centroid error: distance between predicted and true hotspot centers.
    best_by_rmse = comparison_df.sort_values("rmse", ascending=True).iloc[0]
    best_by_hit = comparison_df.sort_values("hit_at_k", ascending=False).iloc[0]
    best_by_centroid = comparison_df.sort_values("centroid_error_m", ascending=True).iloc[0]

    centroid_df = comparison_df.dropna(subset=["centroid_error_m"]).copy()

    if not centroid_df.empty:
        best_model_name = str(
            centroid_df.sort_values("centroid_error_m", ascending=True).iloc[0]["model"]
        )
    else:
        best_model_name = str(best_by_rmse["model"])

    print("\n" + "=" * 70)
    print("Predictor comparison")
    print("=" * 70)
    print(comparison_df)

    print("\nBest model based on lowest RMSE:")
    print(f"  {best_by_rmse['model']} with RMSE={best_by_rmse['rmse']:.6f}")

    print("\nBest model based on highest Hit@K:")
    print(f"  {best_by_hit['model']} with Hit@K={best_by_hit['hit_at_k']:.6f}")

    print("\nBest model based on lowest hotspot-centroid error:")
    print(
        f"  {best_by_centroid['model']} "
        f"with centroid error={best_by_centroid['centroid_error_m']:.2f} m"
    )

    # -------------------------------------------------------------
    # Save graphs
    # -------------------------------------------------------------
    plot_metric_bar(
        comparison_df,
        metric="rmse",
        title="",
        ylabel="RMSE",
        out_path=args.outdir / "compare_rmse.png",
        lower_is_better=True,
    )

    plot_metric_bar(
        comparison_df,
        metric="mae",
        title="Heatmap Prediction MAE",
        ylabel="MAE",
        out_path=args.outdir / "compare_mae.png",
        lower_is_better=True,
    )

    plot_metric_bar(
        comparison_df,
        metric="count_mae",
        title="Underserved-UE Count Prediction Error",
        ylabel="Count MAE",
        out_path=args.outdir / "compare_count_mae.png",
        lower_is_better=True,
    )

    plot_metric_bar(
        comparison_df,
        metric="centroid_error_m",
        title="",
        ylabel="Centroid error (m)",
        out_path=args.outdir / "compare_centroid_error.png",
        lower_is_better=True,
    )

    plot_hotspot_metrics(
        comparison_df,
        out_path=args.outdir / "compare_hotspot_metrics.png",
    )

    plot_training_curves(
        history_df,
        out_path=args.outdir / "training_curves.png",
    )

    plot_best_summary(
        comparison_df,
        out_path=args.outdir / "best_predictor_summary.png",
    )

    # -------------------------------------------------------------
    # Save heatmap summary
    # -------------------------------------------------------------
    # This file is useful for checking whether the training data contains many
    # empty timesteps or enough underserved-user activity to learn from.
    pd.DataFrame(
        {
            "time": times,
            "underserved_count": underserved_counts,
            "heatmap_sum": heatmaps.reshape(len(heatmaps), -1).sum(axis=1),
        }
    ).to_csv(args.outdir / "underserved_heatmap_summary.csv", index=False)

    # -------------------------------------------------------------
    # Save config
    # -------------------------------------------------------------
    # Store enough metadata to reproduce the run and understand the data split.
    config = {
        "train_test_split": "chronological",
        "train_ratio": args.train_ratio,
        "validation_ratio": args.val_ratio,
        "test_ratio": 1.0 - args.train_ratio - args.val_ratio,
        "train_heatmaps": int(train_end),
        "validation_heatmaps": int(val_end - train_end),
        "test_heatmaps": int(len(heatmaps) - val_end),
        "lookback": args.lookback,
        "hotspot_k": args.hotspot_k,
        "rsrp_thresh_dbm": args.rsrp_thresh,
        "event_time_alignment": alignment_stats,
        "normalization": "train-set max count only",
        "normalization_max_count": max_count,
        "grid": {
            "east_min": args.east_min,
            "east_max": args.east_max,
            "north_min": args.north_min,
            "north_max": args.north_max,
            "nx": args.nx,
            "ny": args.ny,
        },
        "best_model_by_rmse": str(best_by_rmse["model"]),
        "best_rmse": float(best_by_rmse["rmse"]),
        "best_model_by_hit_at_k": str(best_by_hit["model"]),
        "best_hit_at_k": float(best_by_hit["hit_at_k"]),
        "best_model_by_centroid_error": str(best_by_centroid["model"]),
        "best_centroid_error_m": float(best_by_centroid["centroid_error_m"]),
        "selected_runtime_model": best_model_name,
        "notes": [
            "Dataset is generated from TN-NTN ns-3 simulation outputs.",
            "Underserved-event timestamps are aligned to the nearest position-trace timestamp.",
            "Input is the previous L underserved-user heatmaps.",
            "Target is the next underserved-user heatmap.",
            "The validation period is used for model selection.",
            "The final test period is used only for held-out reporting.",
            "The split is chronological to avoid future information leakage.",
            "Predictors compared: persistence, MLP, CNN, GRU.",
        ],
    }

    with (args.outdir / "predictor_config.json").open("w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)

    # -------------------------------------------------------------
    # Optional ONNX export
    # -------------------------------------------------------------
    if args.export_onnx:
        export_model_name = best_model_name if args.export_model == "selected" else args.export_model
        export_best_model_to_onnx(
            model_name=export_model_name,
            model=trained_models[export_model_name],
            lookback=args.lookback,
            ny=args.ny,
            nx=args.nx,
            outdir=args.outdir,
        )

    print("\nSaved outputs:")
    print(args.outdir / "predictor_comparison.csv")
    print(args.outdir / "training_history_all_models.csv")
    print(args.outdir / "compare_rmse.png")
    print(args.outdir / "compare_mae.png")
    print(args.outdir / "compare_count_mae.png")
    print(args.outdir / "compare_centroid_error.png")
    print(args.outdir / "compare_hotspot_metrics.png")
    print(args.outdir / "training_curves.png")
    print(args.outdir / "best_predictor_summary.png")
    print(args.outdir / "underserved_heatmap_summary.csv")
    print(args.outdir / "predictor_config.json")


if __name__ == "__main__":
    main()
