#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

# Publication defaults: readable text + arrows when figures are scaled to column width in papers.
_PAPER_FIG_DPI = 300


def _apply_paper_mpl_style():
    """Larger default fonts and tick labels for print/PDF figures."""
    import matplotlib as mpl

    mpl.rcParams.update(
        {
            "font.size": 13,
            "axes.titlesize": 15,
            "axes.labelsize": 14,
            "xtick.labelsize": 12,
            "ytick.labelsize": 12,
            "legend.fontsize": 12,
            "figure.titlesize": 16,
            "axes.linewidth": 1.1,
            "lines.linewidth": 1.2,
            "grid.alpha": 0.28,
            "grid.linestyle": "--",
        }
    )


# 3D quivers use ``linewidths`` (points); BF / final opt should be clearly visible.
QUIVER_LW_BF_OPT_3D = 3.2
QUIVER_LW_TRAJ_3D = 1.35
# 2D quivers use ``width`` in axes fraction.
QUIVER_WIDTH_2D_MAIN = 0.0062
QUIVER_WIDTH_2D_TRAJ = 0.0022
# Matplotlib defaults (headwidth/headlength=3) make arrowheads huge at paper DPI; trim both layers.
QUIVER_HEAD_2D_TRAJ = {"headwidth": 1.85, "headlength": 2.35, "headaxislength": 1.85}
QUIVER_HEAD_2D_MAIN = {"headwidth": 2.05, "headlength": 2.45, "headaxislength": 2.0}


def load_csv(path, skiprows=0):
    if not path.exists():
        raise SystemExit(f"Missing file: {path}")
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()
    if skiprows:
        lines = lines[skiprows:]
    cleaned = []
    for line in lines:
        if not line:
            continue
        clean = line.replace("\x00", "").strip()
        if not clean:
            continue
        cleaned.append(clean)
    if not cleaned:
        return np.zeros((0, 0))
    import io
    data = "\n".join(cleaned)
    return np.loadtxt(io.StringIO(data), delimiter=",")


def load_csv_tail(path, start_line):
    if not path.exists():
        return np.zeros((0, 0))
    with path.open("r", encoding="utf-8") as f:
        lines = f.readlines()
    if start_line >= len(lines):
        return np.zeros((0, 0))
    data_lines = lines[start_line:]
    if data_lines:
        first = data_lines[0].strip()
        if not first or any(ch.isalpha() for ch in first):
            data_lines = data_lines[1:]
    data = "".join(data_lines)
    if not data.strip():
        return np.zeros((0, 0))
    import io
    return np.loadtxt(io.StringIO(data), delimiter=",")


def read_run_info(path):
    info = {}
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("files:"):
                break
            if ":" not in line:
                continue
            key, val = line.split(":", 1)
            info[key.strip()] = val.strip()
    return info


def parse_simple_yaml(path):
    data = {}
    current_key = None
    with path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].rstrip("\n")
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip())
            if indent == 0:
                if ":" not in line:
                    continue
                key, rest = line.split(":", 1)
                key = key.strip()
                rest = rest.strip()
                if rest == "":
                    data[key] = {}
                    current_key = key
                else:
                    data[key] = parse_scalar(rest)
                    current_key = None
            else:
                if current_key is None or not isinstance(data.get(current_key), dict):
                    continue
                if ":" not in line:
                    continue
                key, rest = line.split(":", 1)
                sub_key = key.strip()
                data[current_key][sub_key] = parse_scalar(rest.strip())
    return data


def parse_scalar(value):
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        parts = [p.strip() for p in inner.split(",")]
        return [parse_scalar(p) for p in parts]
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        return value[1:-1]
    try:
        if "." in value or "e" in value.lower():
            return float(value)
        return int(value)
    except ValueError:
        return value


def resolve_manifest_path(manifest_path, rel_path):
    """Resolve paths stored in map manifests (e.g. subsample_maps, base_map).

    Generator YAML uses paths relative to the project root (My_FoV_Optimization),
    e.g. ``Map/cluster9/0_1_cluster9_map.csv``. The manifest file lives at
    ``<root>/Map/<name>/<stem>_manifest.yaml``, so the project root is
    ``manifest_path.parent.parent.parent`` — not ``parent.parent`` (that would
    incorrectly produce ``Map/Map/...`` and break per-level map resolution).
    """
    path = Path(str(rel_path))
    if path.is_absolute():
        return path
    manifest_path = Path(manifest_path).resolve()
    root = manifest_path.parent.parent.parent
    candidate = root / path
    if candidate.exists():
        return candidate
    # Legacy manifests: path relative to Map/ only (no leading Map/ segment).
    legacy = manifest_path.parent.parent / path
    if legacy.exists():
        return legacy
    return candidate


def parse_prefix_level(prefix):
    parts = [p for p in prefix.strip("_").split("_") if p]
    if not parts:
        return None
    try:
        return int(parts[-1])
    except ValueError:
        return None


def resolve_map_for_prefix(prefix, base_map_path):
    if base_map_path is None:
        return None
    base_map_path = Path(base_map_path)
    if base_map_path.name.startswith(prefix):
        return base_map_path

    base_name = base_map_path.name
    match = None
    try:
        import re
        match = re.match(r"^\\d+_\\d+_(.+)$", base_name)
    except Exception:
        match = None
    if match:
        base_name = match.group(1)

    candidate = base_map_path.with_name(prefix + base_name)
    if candidate.exists():
        return candidate

    manifest_stem = Path(base_name).stem
    manifest_path = base_map_path.with_name(manifest_stem + "_manifest.yaml")
    if manifest_path.exists():
        manifest = parse_simple_yaml(manifest_path)
        subsample_maps = manifest.get("subsample_maps", {})
        level = parse_prefix_level(prefix)
        if level is not None:
            key = str(level)
            if key in subsample_maps:
                path = resolve_manifest_path(manifest_path, subsample_maps[key])
                if path.exists():
                    return path
            if level in subsample_maps:
                path = resolve_manifest_path(manifest_path, subsample_maps[level])
                if path.exists():
                    return path

    return base_map_path


def resolve_base_map(base_map_path):
    if base_map_path is None:
        return None
    base_map_path = Path(base_map_path)
    base_name = base_map_path.name
    match = None
    try:
        import re
        match = re.match(r"^\\d+_\\d+_(.+)$", base_name)
    except Exception:
        match = None
    if match:
        base_name = match.group(1)

    manifest_stem = Path(base_name).stem
    manifest_path = base_map_path.with_name(manifest_stem + "_manifest.yaml")
    if manifest_path.exists():
        manifest = parse_simple_yaml(manifest_path)
        base_map = manifest.get("base_map")
        if base_map:
            path = resolve_manifest_path(manifest_path, base_map)
            if path.exists():
                return path
    return base_map_path


def resolve_map_for_subsample_level(base_map_path, level):
    """Map CSV for manifest subsample level k (same k as 0_k_<map>.csv in the generator)."""
    base_map_path = Path(base_map_path)
    manifest_path = base_map_path.with_name(base_map_path.stem + "_manifest.yaml")
    if not manifest_path.exists():
        return base_map_path
    manifest = parse_simple_yaml(manifest_path)
    subs = manifest.get("subsample_maps", {})
    entry = subs.get(str(level)) if isinstance(subs, dict) else None
    if entry is None and isinstance(subs, dict):
        entry = subs.get(level)
    if not entry:
        return base_map_path
    path = resolve_manifest_path(manifest_path, entry)
    return path if path.exists() else base_map_path


def discover_monte_carlo_data_cases(data_dir):
    """
    Flat layout: data/0_3_single_run_rotated_quivers.csv (prefix on filename).
    Per-level layout: data/level_3/ with *single_run_rotated_quivers.csv (may be prefixed,
    e.g. 0_3_single_run_rotated_quivers.csv) so each level's outputs stay distinct.

    If any data/level_* folder contains quiver outputs, only those cases are returned (avoids
    double-counting the same run when both level_* and stray top-level CSVs exist).
    """
    data_dir = Path(data_dir)
    level_cases = []
    for level_dir in sorted(data_dir.glob("level_*")):
        if not level_dir.is_dir():
            continue
        quiver_files = sorted(level_dir.glob("*single_run_rotated_quivers.csv"))
        if not quiver_files:
            continue
        p = quiver_files[0]
        if len(quiver_files) > 1:
            print(
                f"Warning: multiple *single_run_rotated_quivers.csv in {level_dir}; using {p.name}",
                file=sys.stderr,
            )
        prefix = p.name.replace("single_run_rotated_quivers.csv", "")
        m = re.match(r"level_(\d+)$", level_dir.name)
        level_hint = int(m.group(1)) if m else None
        level_cases.append((level_dir, prefix, level_hint))

    if level_cases:
        return level_cases

    cases = []
    for p in sorted(data_dir.glob("*single_run_rotated_quivers.csv")):
        prefix = p.name.replace("single_run_rotated_quivers.csv", "")
        cases.append((data_dir, prefix, None))
    return cases


def plot_case_output_dir(run_dir, level_hint, prefix):
    """Distinct folder under run_dir/plots/ for each case (level or flat prefix)."""
    run_dir = Path(run_dir)
    if level_hint is not None:
        return run_dir / "plots" / f"level_{level_hint}"
    tag = prefix.strip("_").replace("/", "_") if prefix else "base"
    return run_dir / "plots" / f"prefix_{tag}"


def compute_bounds(*arrays):
    mins = np.array([np.inf, np.inf, np.inf], dtype=float)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=float)
    found = False
    for arr in arrays:
        if arr is None or arr.size == 0:
            continue
        pts = arr[:, 0:3]
        mins = np.minimum(mins, pts.min(axis=0))
        maxs = np.maximum(maxs, pts.max(axis=0))
        found = True
    if not found:
        return None
    return mins, maxs


def apply_axes_bounds(ax, bounds, z_scale=1.0):
    if bounds is None:
        return
    mins, maxs = bounds
    x_min, x_max = float(mins[0]), float(maxs[0])
    y_min, y_max = float(mins[1]), float(maxs[1])
    z_min, z_max = float(mins[2]), float(maxs[2])

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    # Center the Z axis on the midpoint of the data z-range (not on z_max when poses sit
    # on one plane). Symmetric z limits around z_center improve 3D framing (e.g. poses at
    # z=20 with map z in [0,20] → limits ~ [-pad, 20+pad] so mid-z ≈ 10 reads as the visual center).
    zr = z_max - z_min
    z_c = 0.5 * (z_min + z_max)
    xy_span = max(x_max - x_min, y_max - y_min, 1e-9)
    if zr < 1e-9:
        z_lo = z_min - 1.0
        z_hi = z_max + 1.0
    else:
        z_rad = 0.5 * zr
        z_pad = max(z_rad * 0.15, xy_span * 0.015)
        z_lo = z_c - z_rad - z_pad
        z_hi = z_c + z_rad + z_pad
    ax.set_zlim(z_lo, z_hi)

    ranges = np.array([x_max - x_min, y_max - y_min, z_hi - z_lo], dtype=float)
    ranges[ranges == 0] = 1.0
    z_scale = max(float(z_scale), 1e-3)
    ax.set_box_aspect((ranges[0], ranges[1], ranges[2] * z_scale))

    # Keep Z-axis labels minimal to avoid overlap on short Z ranges.
    def _fmt_z(v):
        if abs(v - round(v)) < 1e-6:
            return str(int(round(v)))
        return f"{v:.2f}".rstrip("0").rstrip(".")

    if abs(z_hi - z_lo) < 1e-9:
        ax.set_zticks([z_lo])
        ax.set_zticklabels([_fmt_z(z_lo)])
    else:
        ax.set_zticks([z_lo, z_hi])
        ax.set_zticklabels([_fmt_z(z_lo), _fmt_z(z_hi)])


def compute_xy_density(points, max_bins=120, min_bins=20, gamma=0.7):
    if points is None or points.size == 0:
        return None
    pts = points[:, 0:2]
    n = pts.shape[0]
    if n == 0:
        return None
    bins = int(np.clip(np.sqrt(n), min_bins, max_bins))
    hist, xedges, yedges = np.histogram2d(pts[:, 0], pts[:, 1], bins=bins)
    xi = np.searchsorted(xedges, pts[:, 0], side="right") - 1
    yi = np.searchsorted(yedges, pts[:, 1], side="right") - 1
    xi = np.clip(xi, 0, hist.shape[0] - 1)
    yi = np.clip(yi, 0, hist.shape[1] - 1)
    density = hist[xi, yi].astype(float)
    max_density = float(density.max()) if density.size else 0.0
    if max_density > 0:
        density = density / max_density
        density = np.power(density, gamma)
    return density


def quiver_length_from_bounds(bounds, fallback=1.0, fraction=0.05):
    if bounds is None:
        return fallback
    mins, maxs = bounds
    ranges = maxs - mins
    max_range = float(np.max(ranges)) if ranges.size else 0.0
    if max_range <= 0:
        return fallback
    return max(fallback, fraction * max_range)


def _normalize_xy(vectors):
    if vectors.size == 0:
        return vectors
    norms = np.linalg.norm(vectors, axis=1)
    safe = norms > 1e-9
    out = np.zeros_like(vectors)
    out[safe] = vectors[safe] / norms[safe][:, None]
    return out


def select_single_pose_z_layer(opt_quivers, bf_quivers, log_rows):
    """
    Keep only one pose Z layer for quiver visualization clarity.
    Returns filtered (opt_quivers, bf_quivers, log_rows) and selected z.
    """
    opt_quivers = np.atleast_2d(opt_quivers)
    bf_quivers = np.atleast_2d(bf_quivers)
    if opt_quivers.size == 0:
        return opt_quivers, bf_quivers, log_rows, None

    z_vals = opt_quivers[:, 2]
    unique_z = np.unique(np.round(z_vals, 6))
    if unique_z.size <= 1:
        return opt_quivers, bf_quivers, log_rows, float(unique_z[0]) if unique_z.size else None

    # Pick a central Z slice (avoid top/bottom slices when possible).
    z_mid = 0.5 * float(unique_z[0] + unique_z[-1])
    if unique_z.size > 2:
        candidates = unique_z[1:-1]
    else:
        candidates = unique_z
    selected_z = float(candidates[np.argmin(np.abs(candidates - z_mid))])
    opt_mask = np.isclose(z_vals, selected_z, atol=1e-6)
    opt_filtered = opt_quivers[opt_mask]

    # Keep BF rows aligned by pose index.
    bf_filtered = bf_quivers[opt_mask] if bf_quivers.shape[0] == opt_quivers.shape[0] else bf_quivers

    if log_rows is None or log_rows.size == 0:
        return opt_filtered, bf_filtered, log_rows, selected_z

    log_rows = np.atleast_2d(log_rows)
    # Map old pose indices to new compact indices for compatibility with downstream code.
    selected_pose_ids = np.where(opt_mask)[0]
    id_map = {int(old_id): int(new_id) for new_id, old_id in enumerate(selected_pose_ids.tolist())}

    filtered_rows = []
    for row in log_rows:
        pid = int(row[0])
        if pid in id_map:
            new_row = row.copy()
            new_row[0] = id_map[pid]
            filtered_rows.append(new_row)
    if not filtered_rows:
        return opt_filtered, bf_filtered, np.zeros((0, 0)), selected_z

    return opt_filtered, bf_filtered, np.asarray(filtered_rows), selected_z


def plot_top_view_quivers(
    points,
    opt_quivers,
    bf_quivers,
    log_rows,
    out_path,
    sample_points=0,
    include_per_iter_opt_quivers=True,
    include_init_opt_quivers=True,
):
    import matplotlib.pyplot as plt

    if log_rows.size == 0:
        print("No per-iteration log rows found; skipping top-view quiver plot.")
        return

    if sample_points and sample_points > 0 and points.shape[0] > sample_points:
        idx = np.random.choice(points.shape[0], sample_points, replace=False)
        points = points[idx]

    pose_points = opt_quivers[:, 0:3]
    bounds = compute_bounds(points, pose_points)
    scale = quiver_length_from_bounds(bounds)

    fig, ax = plt.subplots(figsize=(8, 7))
    density = compute_xy_density(points)
    if density is None:
        ax.scatter(points[:, 0], points[:, 1], c="gray", alpha=0.5, s=3)
    else:
        ax.scatter(
            points[:, 0], points[:, 1],
            c=density, cmap="Greys", vmin=0, vmax=1,
            alpha=0.6, s=3,
        )
    ax.scatter(pose_points[:, 0], pose_points[:, 1], c="black", alpha=0.6, s=6)

    # All iteration quivers (very light).
    if include_per_iter_opt_quivers:
        refs_all = log_rows[:, 1:3]
        dirs_all = _normalize_xy(log_rows[:, 4:6])
        ax.quiver(
            refs_all[:, 0], refs_all[:, 1],
            dirs_all[:, 0] * (scale * 0.6), dirs_all[:, 1] * (scale * 0.6),
            color="green",
            alpha=0.15,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=QUIVER_WIDTH_2D_TRAJ,
            **QUIVER_HEAD_2D_TRAJ,
        )

    # Initial quiver per pose (first logged iteration).
    if include_init_opt_quivers:
        pose_ids = log_rows[:, 0].astype(int)
        first_idx = {}
        for idx, pid in enumerate(pose_ids):
            if pid not in first_idx:
                first_idx[pid] = idx
        init_rows = log_rows[list(first_idx.values())]
        refs_init = init_rows[:, 1:3]
        dirs_init = _normalize_xy(init_rows[:, 4:6])
        ax.quiver(
            refs_init[:, 0], refs_init[:, 1],
            dirs_init[:, 0] * (scale * 0.6), dirs_init[:, 1] * (scale * 0.6),
            color="green",
            alpha=0.7,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=QUIVER_WIDTH_2D_TRAJ,
            **QUIVER_HEAD_2D_TRAJ,
            label="Init",
        )

    # Brute-force quivers.
    if bf_quivers is not None and bf_quivers.size:
        refs_bf = bf_quivers[:, 0:2]
        dirs_bf = _normalize_xy(bf_quivers[:, 6:8])
        ax.quiver(
            refs_bf[:, 0], refs_bf[:, 1],
            dirs_bf[:, 0] * (scale * 0.9), dirs_bf[:, 1] * (scale * 0.9),
            color="blue",
            alpha=0.95,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=QUIVER_WIDTH_2D_MAIN,
            **QUIVER_HEAD_2D_MAIN,
            label="BF",
        )

    # Final optimized quivers.
    refs_final = opt_quivers[:, 0:2]
    dirs_final = _normalize_xy(opt_quivers[:, 3:5])
    ax.quiver(
        refs_final[:, 0], refs_final[:, 1],
        dirs_final[:, 0] * (scale * 0.6), dirs_final[:, 1] * (scale * 0.6),
        color="red",
        alpha=0.95,
        angles="xy",
        scale_units="xy",
        scale=1,
        width=QUIVER_WIDTH_2D_MAIN,
        **QUIVER_HEAD_2D_MAIN,
        label="Final Opt",
    )

    if bounds is not None:
        mins, maxs = bounds
        ax.set_xlim(mins[0], maxs[0])
        ax.set_ylim(mins[1], maxs[1])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Top View: Opt (red), BF (blue), Init/Iter (green)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def compute_visibility(points, ref_point, direction, visibility_angle_deg, ks):
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return 0.0
    pts = np.atleast_2d(pts)
    if pts.ndim > 2:
        pts = pts.reshape(-1, pts.shape[-1])
    if pts.shape[1] < 3:
        return 0.0
    pts = pts[:, :3]
    ref = np.asarray(ref_point, dtype=float).reshape(-1)
    if ref.size < 3:
        return 0.0
    ref = ref[:3]
    dir_vec = np.asarray(direction, dtype=float).reshape(-1)
    if dir_vec.size < 3:
        return 0.0
    dir_vec = dir_vec[:3]
    dir_norm = np.linalg.norm(dir_vec)
    if dir_norm < 1e-9:
        return 0.0

    vecs = pts - ref
    norms = np.linalg.norm(vecs, axis=1)
    mask = norms > 1e-9
    if not np.any(mask):
        return 0.0
    vecs = vecs[mask] / norms[mask][:, None]
    direction = dir_vec / dir_norm
    cos_theta = vecs @ direction
    cos_alpha = np.cos(np.deg2rad(visibility_angle_deg))
    visibility = 1.0 / (1.0 + np.exp(-ks * (cos_theta - cos_alpha)))
    return float(np.sum(visibility))


def compute_count_in_fov(points, ref_point, direction, visibility_angle_deg):
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return 0
    pts = np.atleast_2d(pts)
    if pts.ndim > 2:
        pts = pts.reshape(-1, pts.shape[-1])
    if pts.shape[1] < 3:
        return 0
    pts = pts[:, :3]
    ref = np.asarray(ref_point, dtype=float).reshape(-1)
    if ref.size < 3:
        return 0
    ref = ref[:3]
    dir_vec = np.asarray(direction, dtype=float).reshape(-1)
    if dir_vec.size < 3:
        return 0
    dir_vec = dir_vec[:3]
    dir_norm = np.linalg.norm(dir_vec)
    if dir_norm < 1e-9:
        return 0

    vecs = pts - ref
    norms = np.linalg.norm(vecs, axis=1)
    mask = norms > 1e-9
    if not np.any(mask):
        return 0
    vecs = vecs[mask] / norms[mask][:, None]
    direction = dir_vec / dir_norm
    cos_theta = vecs @ direction
    cos_alpha = np.cos(np.deg2rad(visibility_angle_deg))
    return int(np.sum(cos_theta >= cos_alpha))


def compute_bf_visibility(points, bf_quivers, visibility_angle_deg, ks):
    bf_visibility = []
    for row in bf_quivers:
        ref = row[0:3]
        bf_dir = row[6:9]
        bf_visibility.append(compute_visibility(points, ref, bf_dir, visibility_angle_deg, ks))
    return np.array(bf_visibility, dtype=float)


def compute_iteration_averages(log_rows, bf_quivers, points, visibility_angle_deg, ks):
    if log_rows.size == 0:
        return None
    pose_ids = log_rows[:, 0].astype(int)
    iter_index = np.zeros_like(pose_ids)
    counts = defaultdict(int)
    for i, pid in enumerate(pose_ids):
        iter_index[i] = counts[pid]
        counts[pid] += 1

    bf_vis = compute_bf_visibility(points, bf_quivers, visibility_angle_deg, ks)
    max_iter = int(iter_index.max()) if iter_index.size else 0
    sums = np.zeros((max_iter + 1, 4), dtype=float)
    nums = np.zeros(max_iter + 1, dtype=int)

    for row, pid, it in zip(log_rows, pose_ids, iter_index):
        if pid < 0 or pid >= len(bf_quivers):
            continue
        ref = row[1:4]
        opt_dir = row[4:7]
        feature_count = compute_count_in_fov(points, ref, opt_dir, visibility_angle_deg)
        angle_diff = row[8]
        j_norm = row[9]
        opt_vis = compute_visibility(points, ref, opt_dir, visibility_angle_deg, ks)
        vis_diff = bf_vis[pid] - opt_vis
        sums[it, 0] += vis_diff
        sums[it, 1] += j_norm
        sums[it, 2] += feature_count
        sums[it, 3] += angle_diff
        nums[it] += 1

    with np.errstate(divide="ignore", invalid="ignore"):
        avgs = sums / np.maximum(nums[:, None], 1)
    return avgs, nums


def compute_per_pose_metrics(points, opt_quivers, bf_quivers, visibility_angle_deg, ks):
    opt_quivers = np.atleast_2d(opt_quivers)
    bf_quivers = np.atleast_2d(bf_quivers)
    n = min(opt_quivers.shape[0], bf_quivers.shape[0])
    results = np.zeros((n, 7), dtype=float)
    for i in range(n):
        ref = opt_quivers[i, 0:3]
        opt_dir = opt_quivers[i, 3:6]
        bf_dir = bf_quivers[i, 6:9]
        opt_vis = compute_visibility(points, ref, opt_dir, visibility_angle_deg, ks)
        bf_vis = compute_visibility(points, ref, bf_dir, visibility_angle_deg, ks)
        vis_diff = bf_vis - opt_vis
        opt_count = compute_count_in_fov(points, ref, opt_dir, visibility_angle_deg)
        bf_count = compute_count_in_fov(points, ref, bf_dir, visibility_angle_deg)
        angle_diff = np.degrees(
            np.arccos(
                np.clip(
                    np.dot(
                        bf_dir / np.linalg.norm(bf_dir),
                        opt_dir / np.linalg.norm(opt_dir),
                    ),
                    -1.0,
                    1.0,
                )
            )
        )
        results[i, 0] = opt_vis
        results[i, 1] = bf_vis
        results[i, 2] = vis_diff
        results[i, 3] = opt_count
        results[i, 4] = bf_count
        results[i, 5] = angle_diff
        results[i, 6] = i
    return results


def load_time_per_pose_us(path, max_rows=None):
    """
    Per-pose runtimes in microseconds from *_avg_time_file.csv.

    Skips the header row. If the C++ runner appended a final summary line equal to the
    mean of the per-pose rows, that duplicate line is dropped so mean/std reflect only
    per-pose samples.
    """
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except OSError:
        return None
    values = []
    row_count = 0
    for line in lines[1:]:
        if not line:
            continue
        clean = line.replace("\x00", "").strip()
        if not clean:
            continue
        try:
            values.append(float(clean.split(",")[0]))
            row_count += 1
            if max_rows is not None and row_count >= max_rows:
                break
        except ValueError:
            continue
    if not values:
        return None
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    if values.size >= 2:
        mean_prev = float(np.mean(values[:-1]))
        if abs(values[-1] - mean_prev) <= 1e-6 * max(1.0, abs(mean_prev)):
            values = values[:-1]
    return values


def load_time_stats(path, max_rows=None):
    values = load_time_per_pose_us(path, max_rows=max_rows)
    if values is None or values.size == 0:
        return None
    mean = float(np.mean(values))
    std = float(np.std(values))
    return mean, std


def analyze_runtime_times_us(values_us, label=""):
    """
    Summarize per-pose times (microseconds) and flag likely anomalies.

    Anomalies (warnings): non-positive times; suspiciously small vs median (often
    cache/IO bugs). Longer-than-usual runs are *not* flagged as errors — they are
    reported separately as IQR long-tail counts (legitimate slow poses).
    """
    v = np.asarray(values_us, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return None
    med = float(np.median(v))
    q1, q3 = float(np.percentile(v, 25)), float(np.percentile(v, 75))
    iqr = q3 - q1
    high_fence = q3 + 1.5 * iqr if iqr > 0 else float("inf")
    # Stricter than generic outliers: only flag values far below typical (bad zeros / stale rows).
    low_thresh = max(100.0, 0.005 * med) if med > 0 else 100.0

    anomalies = []
    for i, val in enumerate(v):
        if val <= 0:
            anomalies.append((i, float(val), "non_positive"))
        elif val < low_thresh:
            anomalies.append((i, float(val), "suspiciously_low_vs_median"))

    long_tail_iqr = int(np.sum(v > high_fence)) if np.isfinite(high_fence) else 0

    return {
        "label": label,
        "n": int(v.size),
        "mean_us": float(np.mean(v)),
        "std_us": float(np.std(v)),
        "min_us": float(np.min(v)),
        "median_us": med,
        "max_us": float(np.max(v)),
        "q1_us": q1,
        "q3_us": q3,
        "iqr_high_fence_us": float(high_fence) if np.isfinite(high_fence) else float("nan"),
        "long_tail_iqr_count": long_tail_iqr,
        "anomalies": anomalies,
    }


def format_runtime_sanity_report(opt_path, bf_path, opt_stats, bf_stats):
    """Human-readable report for one case (level/prefix)."""
    lines = []
    lines.append("Runtime sanity (per-pose rows in *_avg_time_file.csv; σ = std across poses)")
    lines.append("")

    def _block(path, stats):
        if stats is None:
            lines.append(f"{path.name}: missing or empty")
            lines.append("")
            return
        lines.append(f"{path.name}")
        lines.append(
            f"  n={stats['n']}  mean={stats['mean_us']:.2f} μs  σ={stats['std_us']:.2f} μs"
        )
        lines.append(
            f"  min={stats['min_us']:.2f}  median={stats['median_us']:.2f}  max={stats['max_us']:.2f} μs"
        )
        lt = stats.get("long_tail_iqr_count")
        fence = stats.get("iqr_high_fence_us")
        if lt is not None and lt > 0 and np.isfinite(fence):
            lines.append(
                f"  long runs (time > Q3+1.5·IQR, fence={fence:.1f} μs): {lt} pose(s) — usually legitimate, not errors"
            )
        if stats["anomalies"]:
            lines.append(f"  anomalies flagged ({len(stats['anomalies'])}):")
            for pose_i, val, reason in stats["anomalies"][:50]:
                lines.append(f"    pose_index={pose_i}  time={val:.2f} μs  ({reason})")
            if len(stats["anomalies"]) > 50:
                lines.append(f"    ... and {len(stats['anomalies']) - 50} more")
        else:
            lines.append("  no anomalies flagged (non-positive / suspiciously low)")
        lines.append("")

    _block(opt_path, opt_stats)
    _block(bf_path, bf_stats)
    return "\n".join(lines)


def write_runtime_sanity_file(out_path, text):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        f.write(text)


def write_avg_metrics(path, avgs, counts):
    with path.open("w", encoding="utf-8") as f:
        f.write("iteration,avg_visibility_diff,avg_jacobian,avg_feature_count,avg_angle_diff,count\n")
        for i in range(avgs.shape[0]):
            f.write(
                f"{i},{avgs[i,0]:.6f},{avgs[i,1]:.6f},{avgs[i,2]:.6f},{avgs[i,3]:.6f},{counts[i]}\n"
            )


def write_per_pose_metrics(path, data):
    with path.open("w", encoding="utf-8") as f:
        f.write(
            "pose_id,opt_visibility,bf_visibility,visibility_diff,opt_feature_count,bf_feature_count,angle_diff_deg\n"
        )
        for row in data:
            f.write(
                f"{int(row[6])},{row[0]:.6f},{row[1]:.6f},{row[2]:.6f},"
                f"{int(row[3])},{int(row[4])},{row[5]:.6f}\n"
            )


def plot_avg_metrics(avgs, out_path):
    import matplotlib.pyplot as plt

    iters = np.arange(avgs.shape[0])
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes = axes.ravel()
    axes[0].plot(iters, avgs[:, 0], marker="o", ms=5)
    axes[0].set_title("Avg Visibility Diff (BF - Opt)")
    axes[0].set_xlabel("Iteration")
    axes[0].set_ylabel("Visibility Diff")

    axes[1].plot(iters, avgs[:, 1], marker="o", ms=5)
    axes[1].set_title("Avg Jacobian Norm")
    axes[1].set_xlabel("Iteration")
    axes[1].set_ylabel("Jacobian Norm")

    axes[2].plot(iters, avgs[:, 2], marker="o", ms=5)
    axes[2].set_title("Avg Feature Count")
    axes[2].set_xlabel("Iteration")
    axes[2].set_ylabel("Feature Count")

    axes[3].plot(iters, avgs[:, 3], marker="o", ms=5)
    axes[3].set_title("Avg Angle Diff (deg)")
    axes[3].set_xlabel("Iteration")
    axes[3].set_ylabel("Angle Diff")

    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI)
    plt.close(fig)


def plot_avg_metrics_total(level_buffers, out_path):
    """Overlay iteration-averaged metrics for every subsample level on one figure."""
    import matplotlib.pyplot as plt

    if not level_buffers:
        return
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes = axes.ravel()
    titles = [
        "Avg visibility diff (BF − Opt)",
        "Avg Jacobian norm",
        "Avg feature count",
        "Avg angle diff (deg)",
    ]
    ylabels = ["Visibility diff", "‖J‖", "Feature count", "Angle diff (deg)"]
    for bi, b in enumerate(level_buffers):
        if b.get("metrics") is None:
            continue
        avgs, _ = b["metrics"]
        it = np.arange(avgs.shape[0])
        label = b.get("label") or f"case_{bi}"
        color = f"C{bi % 10}"
        for k in range(4):
            axes[k].plot(it, avgs[:, k], marker="o", ms=5, label=label, color=color, alpha=0.85, linewidth=1.25)

    for k in range(4):
        axes[k].set_title(titles[k])
        axes[k].set_xlabel("Iteration")
        axes[k].set_ylabel(ylabels[k])
        axes[k].grid(alpha=0.28, linestyle="--", linewidth=0.75)
        axes[k].legend(ncol=2, loc="best", frameon=True)
    fig.suptitle("Optimization metrics (all subsample levels)", y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def plot_per_pose_metrics_total(level_buffers, out_path):
    """Overlay per-pose curves for every subsample level (same pose grid)."""
    import matplotlib.pyplot as plt

    if not level_buffers:
        return
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes = axes.ravel()
    titles = [
        "Optimized visibility",
        "Visibility diff (BF − Opt)",
        "Optimized feature count",
        "Angle diff vs BF (deg)",
    ]
    ylabels = ["Visibility", "Diff", "Count", "Angle (deg)"]
    for bi, b in enumerate(level_buffers):
        d = np.atleast_2d(b["per_pose"])
        if d.size == 0:
            continue
        pid = d[:, 6]
        label = b.get("label") or f"case_{bi}"
        color = f"C{bi % 10}"
        axes[0].plot(pid, d[:, 0], ".", ms=3.5, alpha=0.55, color=color, label=label)
        axes[1].plot(pid, d[:, 2], ".", ms=3.5, alpha=0.55, color=color, label=label)
        axes[2].plot(pid, d[:, 3], ".", ms=3.5, alpha=0.55, color=color, label=label)
        axes[3].plot(pid, d[:, 5], ".", ms=3.5, alpha=0.55, color=color, label=label)

    for k in range(4):
        axes[k].set_title(titles[k])
        axes[k].set_xlabel("Pose ID")
        axes[k].set_ylabel(ylabels[k])
        axes[k].grid(alpha=0.28, linestyle="--", linewidth=0.75)
        axes[k].legend(ncol=2, loc="best", frameon=True)
    fig.suptitle("Per-pose metrics (all subsample levels)", y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def mean_per_pose_metrics_across_levels(level_buffers):
    """Mean of per-pose metric rows across levels (same pose ordering)."""
    if not level_buffers:
        return None, None
    stacks = [np.atleast_2d(b["per_pose"]) for b in level_buffers]
    n0 = int(stacks[0].shape[0])
    for s in stacks[1:]:
        if int(s.shape[0]) != n0:
            print(
                "[total] Per-pose row count differs across levels; skipping total aggregates.",
                file=sys.stderr,
            )
            return None, None
    mean = np.mean(np.stack(stacks, axis=0), axis=0)
    mean[:, 6] = np.arange(mean.shape[0])
    opt_ref = np.atleast_2d(level_buffers[0]["opt_quivers"])
    return mean, opt_ref


def j_norm_mean_across_levels(level_buffers):
    """Mean last-iteration ‖J‖ per pose across levels."""
    vals = []
    for b in level_buffers:
        j = _j_norm_array_aligned(b["per_pose"], b["log_rows"])
        if j is None:
            return None
        vals.append(j)
    return np.nanmean(np.stack(vals, axis=0), axis=0)


def write_total_run_outputs(output_dir, level_buffers, dense_map_points, args):
    """
    Run-level PNGs (and one CSV) aggregating all subsample levels.

    Uses **mean per pose** across levels for mismatch / failure-map summary panels.
    Per-pose and avg-metric figures **overlay** every level with distinct colors.
    """
    if len(level_buffers) < 2:
        return
    output_dir = Path(output_dir)
    mean_pose, opt_ref = mean_per_pose_metrics_across_levels(level_buffers)
    if mean_pose is None or mean_pose.size == 0:
        return

    bg = dense_map_points
    if bg is None or bg.size == 0:
        bg = level_buffers[0].get("map_points")

    j_mean = j_norm_mean_across_levels(level_buffers)

    if not args.no_failure_map:
        if bg is not None and np.asarray(bg).size > 0:
            plot_failure_maps(
                bg,
                mean_pose,
                opt_ref,
                output_dir / "total_failure_map.png",
                log_rows=None,
                sample_points=args.plot_sample,
                title_suffix="mean per pose across levels",
                plane=args.failure_map_plane,
                j_norm_override=j_mean,
            )
        else:
            print("[total] No map point cloud; skipping total_failure_map.png.", file=sys.stderr)

    if not args.no_bf_opt_mismatch:
        plot_bf_vs_opt_mismatch(
            mean_pose,
            opt_ref,
            output_dir / "total_bf_vs_opt_mismatch.png",
            title_suffix="mean per pose across levels",
        )
        write_bf_vs_opt_mismatch_csv(
            output_dir / "total_bf_vs_opt_mismatch.csv", mean_pose, opt_ref
        )

    plot_per_pose_metrics_total(level_buffers, output_dir / "total_per_pose_metrics.png")

    if any(b.get("metrics") is not None for b in level_buffers):
        plot_avg_metrics_total(level_buffers, output_dir / "total_avg_metrics.png")

    print(
        "Total (all-level) outputs: "
        f"{output_dir / 'total_failure_map.png'}, "
        f"{output_dir / 'total_bf_vs_opt_mismatch.png'}, "
        f"{output_dir / 'total_bf_vs_opt_mismatch.csv'}, "
        f"{output_dir / 'total_per_pose_metrics.png'}, "
        f"{output_dir / 'total_avg_metrics.png'}",
        file=sys.stderr,
    )


def plot_per_pose_metrics(data, out_path):
    import matplotlib.pyplot as plt

    pose_ids = data[:, 6]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes = axes.ravel()

    axes[0].plot(pose_ids, data[:, 0], marker=".", ms=4, linestyle="none", label="Opt")
    axes[0].plot(pose_ids, data[:, 1], marker=".", ms=4, linestyle="none", label="BF", alpha=0.6)
    axes[0].set_title("Per-Pose Visibility")
    axes[0].set_xlabel("Pose ID")
    axes[0].set_ylabel("Visibility")
    axes[0].legend()

    axes[1].plot(pose_ids, data[:, 2], marker=".", ms=4, linestyle="none", color="tab:red")
    axes[1].set_title("Per-Pose Visibility Diff (BF - Opt)")
    axes[1].set_xlabel("Pose ID")
    axes[1].set_ylabel("Visibility Diff")

    axes[2].plot(pose_ids, data[:, 3], marker=".", ms=4, linestyle="none", label="Opt")
    axes[2].plot(pose_ids, data[:, 4], marker=".", ms=4, linestyle="none", label="BF", alpha=0.6)
    axes[2].set_title("Per-Pose Feature Count")
    axes[2].set_xlabel("Pose ID")
    axes[2].set_ylabel("Count")
    axes[2].legend()

    axes[3].plot(pose_ids, data[:, 5], marker=".", ms=4, linestyle="none", color="tab:purple")
    axes[3].set_title("Per-Pose Angle Diff (deg)")
    axes[3].set_xlabel("Pose ID")
    axes[3].set_ylabel("Angle Diff")

    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI)
    plt.close(fig)


def write_bf_vs_opt_mismatch_csv(path, per_pose, opt_quivers):
    """CSV for downstream analysis: where opt diverges from BF per pose."""
    per_pose = np.atleast_2d(per_pose)
    opt_quivers = np.atleast_2d(opt_quivers)
    n = min(int(per_pose.shape[0]), int(opt_quivers.shape[0]))
    opt_vis = per_pose[:n, 0]
    bf_vis = per_pose[:n, 1]
    vis_diff = per_pose[:n, 2]
    angle_deg = per_pose[:n, 5]
    pose_id = per_pose[:n, 6].astype(int)
    with np.errstate(divide="ignore", invalid="ignore"):
        rel_gap = vis_diff / np.maximum(bf_vis, 1e-12)
    rel_gap = np.nan_to_num(rel_gap, nan=0.0, posinf=0.0, neginf=0.0)

    with path.open("w", encoding="utf-8") as f:
        f.write(
            "pose_id,ref_x,ref_y,ref_z,opt_visibility,bf_visibility,visibility_diff,"
            "relative_gap_bf_minus_opt_over_bf,angle_diff_deg,opt_vs_bf_count_diff\n"
        )
        for i in range(n):
            ref = opt_quivers[i, 0:3]
            oc = int(per_pose[i, 3])
            bc = int(per_pose[i, 4])
            f.write(
                f"{pose_id[i]},{ref[0]:.6f},{ref[1]:.6f},{ref[2]:.6f},"
                f"{opt_vis[i]:.6f},{bf_vis[i]:.6f},{vis_diff[i]:.6f},"
                f"{rel_gap[i]:.6f},{angle_deg[i]:.6f},{bc - oc}\n"
            )


def plot_bf_vs_opt_mismatch(per_pose, opt_quivers, out_path, title_suffix=""):
    """
    Compare BF vs optimization per pose with two clean panels:
    (1) visibility scatter on identity line, (2) XY map colored by BF-relative gap.
    """
    import matplotlib.pyplot as plt

    per_pose = np.atleast_2d(per_pose)
    opt_quivers = np.atleast_2d(opt_quivers)
    if per_pose.shape[0] == 0:
        print("No per-pose data; skipping BF vs opt mismatch plot.")
        return

    n = min(int(per_pose.shape[0]), int(opt_quivers.shape[0]))
    opt_vis = per_pose[:n, 0]
    bf_vis = per_pose[:n, 1]
    vis_diff = per_pose[:n, 2]
    angle_deg = per_pose[:n, 5]
    pose_id = per_pose[:n, 6]

    with np.errstate(divide="ignore", invalid="ignore"):
        rel_gap = vis_diff / np.maximum(bf_vis, 1e-12)
    rel_gap = np.nan_to_num(rel_gap, nan=0.0, posinf=0.0, neginf=0.0)

    ref_xy = opt_quivers[:n, 0:2]
    vmax = float(np.nanpercentile(rel_gap, 98)) if n > 0 else 1.0
    if not np.isfinite(vmax) or vmax <= 0:
        vmax = 1e-6

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8))
    ax_scatter, ax_map = axes.ravel()

    # (1) Opt vs BF visibility — on diagonal means perfect mimicry.
    lo = float(np.nanmin([opt_vis.min(), bf_vis.min()])) if n else 0.0
    hi = float(np.nanmax([opt_vis.max(), bf_vis.max()])) if n else 1.0
    pad = 0.02 * max(hi - lo, 1e-9)
    sc_diag = ax_scatter.scatter(
        bf_vis, opt_vis, c=angle_deg, cmap="viridis", s=26, alpha=0.78
    )
    ax_scatter.plot([lo - pad, hi + pad], [lo - pad, hi + pad], "k--", lw=1, alpha=0.6, label="y = x")
    ax_scatter.set_aspect("equal", adjustable="box")
    ax_scatter.set_xlim(lo - pad, hi + pad)
    ax_scatter.set_ylim(lo - pad, hi + pad)
    ax_scatter.set_xlabel("BF visibility (sigmoid sum)")
    ax_scatter.set_ylabel("Optimized visibility")
    ax_scatter.set_title("Optimized vs BF visibility")
    cbar_diag = fig.colorbar(sc_diag, ax=ax_scatter, fraction=0.046, pad=0.04)
    cbar_diag.set_label("Angle diff ∠(BF, Opt) [deg]")
    ax_scatter.legend(loc="lower right")

    # (2) Spatial map: pose XY colored by relative gap.
    sc = ax_map.scatter(
        ref_xy[:, 0],
        ref_xy[:, 1],
        c=rel_gap,
        cmap="inferno",
        s=26,
        alpha=0.85,
        vmin=0.0,
        vmax=vmax,
    )
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.set_xlabel("X (pose ref)")
    ax_map.set_ylabel("Y (pose ref)")
    ax_map.set_title("BF-relative visibility gap over pose map")
    fig.colorbar(
        sc,
        ax=ax_map,
        fraction=0.046,
        pad=0.04,
        label=r"Relative gap $(BF - Opt) / \max(BF,\epsilon)$",
    )

    suptitle = "BF vs optimization: per-pose mismatch"
    if title_suffix:
        suptitle = f"{suptitle} — {title_suffix}"
    fig.suptitle(suptitle, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)

    # Save each panel as its own PNG for paper layout flexibility.
    out_path = Path(out_path)
    scatter_out = out_path.with_name(out_path.stem + "_scatter.png")
    map_out = out_path.with_name(out_path.stem + "_map.png")

    fig_sc, ax_sc = plt.subplots(figsize=(5.5, 4.8))
    sc_only = ax_sc.scatter(
        bf_vis, opt_vis, c=angle_deg, cmap="viridis", s=28, alpha=0.82
    )
    ax_sc.plot([lo - pad, hi + pad], [lo - pad, hi + pad], "k--", lw=1, alpha=0.6, label="y = x")
    ax_sc.set_aspect("equal", adjustable="box")
    ax_sc.set_xlim(lo - pad, hi + pad)
    ax_sc.set_ylim(lo - pad, hi + pad)
    ax_sc.set_xlabel("BF visibility (sigmoid sum)")
    ax_sc.set_ylabel("Optimized visibility")
    ax_sc.set_title("Optimized vs BF visibility")
    fig_sc.colorbar(sc_only, ax=ax_sc, fraction=0.046, pad=0.04, label="Angle diff ∠(BF, Opt) [deg]")
    ax_sc.legend(loc="lower right")
    fig_sc.tight_layout()
    fig_sc.savefig(scatter_out, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig_sc)

    fig_map, ax_m = plt.subplots(figsize=(5.5, 4.8))
    sc_only_map = ax_m.scatter(
        ref_xy[:, 0],
        ref_xy[:, 1],
        c=rel_gap,
        cmap="inferno",
        s=28,
        alpha=0.88,
        vmin=0.0,
        vmax=vmax,
    )
    ax_m.set_aspect("equal", adjustable="box")
    ax_m.set_xlabel("X (pose ref)")
    ax_m.set_ylabel("Y (pose ref)")
    ax_m.set_title("BF-relative visibility gap over pose map")
    fig_map.colorbar(
        sc_only_map,
        ax=ax_m,
        fraction=0.046,
        pad=0.04,
        label=r"Relative gap $(BF - Opt) / \max(BF,\epsilon)$",
    )
    fig_map.tight_layout()
    fig_map.savefig(map_out, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig_map)


def _last_log_row_per_pose(log_rows):
    """Map pose_id -> last iteration row (for j_norm / visibility at end of opt)."""
    if log_rows is None or log_rows.size == 0:
        return None
    lr = np.atleast_2d(log_rows)
    if lr.shape[1] < 10:
        return None
    last = {}
    for row in lr:
        pid = int(row[0])
        last[pid] = row
    return last


def _j_norm_array_aligned(per_pose, log_rows):
    """
    Per-pose last-iteration Jacobian norm (column 9 in quiversforonepoint), aligned with per_pose rows.
    Returns None if log unavailable; NaN where pose id missing from log.
    """
    last = _last_log_row_per_pose(log_rows)
    if last is None:
        return None
    per_pose = np.atleast_2d(per_pose)
    n = int(per_pose.shape[0])
    out = np.full(n, np.nan, dtype=float)
    for i in range(n):
        pid = int(per_pose[i, 6])
        row = last.get(pid)
        if row is not None and row.shape[0] > 9:
            out[i] = float(row[9])
    return out


def plot_failure_maps(
    points,
    per_pose,
    opt_quivers,
    out_path,
    log_rows=None,
    sample_points=0,
    title_suffix="",
    plane="xy",
    j_norm_override=None,
):
    """
    Failure map: map point cloud as geometry, each pose colored by BF gap / angle error /
    optional last-iter Jacobian norm (convergence stress from logs).

    If ``j_norm_override`` is set (e.g. mean ‖J‖ across levels), it is used instead of
    reading from ``log_rows``.
    """
    import matplotlib.pyplot as plt

    per_pose = np.atleast_2d(per_pose)
    opt_quivers = np.atleast_2d(opt_quivers)
    if per_pose.shape[0] == 0:
        print("No per-pose data; skipping failure map.")
        return

    n = min(int(per_pose.shape[0]), int(opt_quivers.shape[0]))
    vis_diff = per_pose[:n, 2]
    bf_vis = per_pose[:n, 1]
    angle_deg = per_pose[:n, 5]

    with np.errstate(divide="ignore", invalid="ignore"):
        rel_gap = vis_diff / np.maximum(bf_vis, 1e-12)
    rel_gap = np.nan_to_num(rel_gap, nan=0.0, posinf=0.0, neginf=0.0)

    if j_norm_override is not None:
        j_norm = np.asarray(j_norm_override, dtype=float).reshape(-1)
        if j_norm.size < n:
            j_norm = np.pad(j_norm, (0, max(0, n - j_norm.size)), constant_values=np.nan)
        j_norm = j_norm[:n]
        has_j = np.any(np.isfinite(j_norm))
    else:
        j_norm = _j_norm_array_aligned(per_pose[:n], log_rows)
        has_j = j_norm is not None and np.any(np.isfinite(j_norm))

    pts = np.asarray(points, dtype=float)
    if sample_points and sample_points > 0 and pts.shape[0] > sample_points:
        idx = np.random.choice(pts.shape[0], sample_points, replace=False)
        pts = pts[idx]

    plane = (plane or "xy").lower().strip()
    if plane == "xz":
        def proj(p):
            return p[:, 0], p[:, 2]

        xlab, ylab = "X", "Z"
    else:
        def proj(p):
            return p[:, 0], p[:, 1]

        xlab, ylab = "X", "Y"

    px, py = proj(pts)
    ref = opt_quivers[:n, 0:3]
    rx, ry = proj(ref)

    bounds = compute_bounds(pts, ref)
    vmax_gap = float(np.nanpercentile(rel_gap, 98)) if n else 1.0
    if not np.isfinite(vmax_gap) or vmax_gap <= 0:
        vmax_gap = 1e-6
    vmax_ang = float(np.nanpercentile(angle_deg, 98)) if n else 1.0
    if not np.isfinite(vmax_ang) or vmax_ang <= 0:
        vmax_ang = 1e-6

    ncols = 3 if has_j else 2
    fig_w = 6.2 * ncols + 1.0
    fig, axes = plt.subplots(1, ncols, figsize=(fig_w, 5.8))

    density = compute_xy_density(pts) if plane == "xy" else None
    if plane == "xz" and pts.shape[0] > 0:
        density = compute_xy_density(np.column_stack([pts[:, 0], pts[:, 2]]))

    def _bg(ax):
        if density is None:
            ax.scatter(px, py, c="lightgray", s=2, alpha=0.35, linewidths=0, label="map")
        else:
            ax.scatter(
                px, py, c=density, cmap="Greys", vmin=0, vmax=1,
                s=2, alpha=0.5, linewidths=0,
            )

    # (1) BF relative gap — primary "failure to match oracle" metric.
    ax = axes[0]
    _bg(ax)
    sc0 = ax.scatter(
        rx, ry, c=rel_gap, cmap="inferno", s=52, alpha=0.92,
        vmin=0.0, vmax=vmax_gap, edgecolors="white", linewidths=0.55,
    )
    fig.colorbar(sc0, ax=ax, fraction=0.046, pad=0.02, label=r"$(BF-Opt)/\max(BF,\epsilon)$")
    ax.set_title("BF gap (visibility)")
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    ax.set_aspect("equal", adjustable="box")
    if bounds is not None:
        mins, maxs = bounds
        if plane == "xz":
            ax.set_xlim(mins[0], maxs[0])
            ax.set_ylim(mins[2], maxs[2])
        else:
            ax.set_xlim(mins[0], maxs[0])
            ax.set_ylim(mins[1], maxs[1])

    # (2) Direction error vs BF.
    ax = axes[1]
    _bg(ax)
    sc1 = ax.scatter(
        rx, ry, c=angle_deg, cmap="plasma", s=52, alpha=0.92,
        vmin=0.0, vmax=vmax_ang, edgecolors="white", linewidths=0.55,
    )
    fig.colorbar(sc1, ax=ax, fraction=0.046, pad=0.02, label="∠(BF, Opt) [deg]")
    ax.set_title("Direction error vs BF")
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    ax.set_aspect("equal", adjustable="box")
    if bounds is not None:
        mins, maxs = bounds
        if plane == "xz":
            ax.set_xlim(mins[0], maxs[0])
            ax.set_ylim(mins[2], maxs[2])
        else:
            ax.set_xlim(mins[0], maxs[0])
            ax.set_ylim(mins[1], maxs[1])

    # (3) Last-iteration ||J|| (proxy for gradient activity / non-convergence).
    if has_j:
        ax = axes[2]
        _bg(ax)
        j_finite = j_norm[np.isfinite(j_norm)]
        vmax_j = float(np.nanpercentile(j_finite, 98)) if j_finite.size else 1.0
        if not np.isfinite(vmax_j) or vmax_j <= 0:
            vmax_j = 1e-9
        j_plot = np.ma.masked_invalid(j_norm)
        sc2 = ax.scatter(
            rx, ry, c=j_plot, cmap="cividis", s=36, alpha=0.92,
            vmin=0.0, vmax=vmax_j, edgecolors="white", linewidths=0.35,
        )
        fig.colorbar(sc2, ax=ax, fraction=0.046, pad=0.02, label=r"Last iter $\|J\|$")
        ax.set_title("Convergence (last ||J||)")
        ax.set_xlabel(xlab)
        ax.set_ylabel(ylab)
        ax.set_aspect("equal", adjustable="box")
        if bounds is not None:
            mins, maxs = bounds
            if plane == "xz":
                ax.set_xlim(mins[0], maxs[0])
                ax.set_ylim(mins[2], maxs[2])
            else:
                ax.set_xlim(mins[0], maxs[0])
                ax.set_ylim(mins[1], maxs[1])

    suptitle = "Failure map: poses on map geometry"
    if title_suffix:
        suptitle = f"{suptitle} — {title_suffix}"
    fig.suptitle(suptitle, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def plot_quivers(points, opt_quivers, bf_quivers, out_path, sample_points=0, z_scale=1.0):
    import matplotlib.pyplot as plt

    if sample_points and sample_points > 0 and points.shape[0] > sample_points:
        idx = np.random.choice(points.shape[0], sample_points, replace=False)
        points = points[idx]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    density = compute_xy_density(points)
    if density is None:
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="gray", alpha=0.12, s=1.8, depthshade=False)
    else:
        ax.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            c=density, cmap="Greys", vmin=0, vmax=1,
            alpha=0.12, s=1.8, depthshade=False,
        )
    pose_points = opt_quivers[:, 0:3]
    ax.scatter(pose_points[:, 0], pose_points[:, 1], pose_points[:, 2], c="black", alpha=0.6, s=6)

    bounds = compute_bounds(points, pose_points)
    scale = quiver_length_from_bounds(bounds)
    for row in opt_quivers:
        ref = row[0:3]
        opt_dir = row[3:6]
        ax.quiver(
            ref[0],
            ref[1],
            ref[2],
            opt_dir[0],
            opt_dir[1],
            opt_dir[2],
            color="red",
            length=scale * 0.6,
            normalize=True,
            alpha=0.9,
            linewidths=QUIVER_LW_BF_OPT_3D,
        )
    for row in bf_quivers:
        ref = row[0:3]
        bf_dir = row[6:9]
        ax.quiver(
            ref[0],
            ref[1],
            ref[2],
            bf_dir[0],
            bf_dir[1],
            bf_dir[2],
            color="blue",
            length=scale * 0.95,
            normalize=True,
            alpha=0.9,
            linewidths=QUIVER_LW_BF_OPT_3D,
        )

    ax.set_title("Brute force (blue) vs optimized (red) viewing directions")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    apply_axes_bounds(ax, bounds, z_scale=z_scale)
    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def interactive_viewer_all(
    points,
    log_rows,
    bf_quivers,
    opt_quivers,
    sample_points=0,
    z_scale=1.0,
    include_per_iter_opt_quivers=True,
    include_init_opt_quivers=True,
):
    import matplotlib.pyplot as plt

    if log_rows.size == 0:
        print("No per-iteration log rows found; skipping 3D viewer.")
        return

    if sample_points and sample_points > 0 and points.shape[0] > sample_points:
        idx = np.random.choice(points.shape[0], sample_points, replace=False)
        points = points[idx]

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    density = compute_xy_density(points)
    if density is None:
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="gray", alpha=0.12, s=1.8, depthshade=False)
    else:
        ax.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            c=density, cmap="Greys", vmin=0, vmax=1,
            alpha=0.12, s=1.8, depthshade=False,
        )
    pose_points = opt_quivers[:, 0:3]
    ax.scatter(pose_points[:, 0], pose_points[:, 1], pose_points[:, 2], c="black", alpha=0.6, s=6)

    bounds = compute_bounds(points, pose_points)
    apply_axes_bounds(ax, bounds, z_scale=z_scale)
    scale = quiver_length_from_bounds(bounds)

    if include_per_iter_opt_quivers:
        refs = log_rows[:, 1:4]
        dirs = log_rows[:, 4:7]
        ax.quiver(
            refs[:, 0],
            refs[:, 1],
            refs[:, 2],
            dirs[:, 0],
            dirs[:, 1],
            dirs[:, 2],
            color="green",
            length=scale * 0.6,
            normalize=True,
            alpha=0.15,
            linewidths=QUIVER_LW_TRAJ_3D,
        )
    elif include_init_opt_quivers:
        # Plot only the initial (first iteration) quiver per pose.
        pose_ids = log_rows[:, 0].astype(int)
        first_idx = {}
        for idx, pid in enumerate(pose_ids):
            if pid not in first_idx:
                first_idx[pid] = idx
        init_rows = log_rows[list(first_idx.values())]
        refs = init_rows[:, 1:4]
        dirs = init_rows[:, 4:7]
        ax.quiver(
            refs[:, 0],
            refs[:, 1],
            refs[:, 2],
            dirs[:, 0],
            dirs[:, 1],
            dirs[:, 2],
            color="green",
            length=scale * 0.6,
            normalize=True,
            alpha=0.6,
            linewidths=QUIVER_LW_TRAJ_3D,
        )

    bf_refs = bf_quivers[:, 0:3]
    bf_dirs = bf_quivers[:, 6:9]
    opt_refs = opt_quivers[:, 0:3]
    opt_dirs = opt_quivers[:, 3:6]
    ax.quiver(
        opt_refs[:, 0],
        opt_refs[:, 1],
        opt_refs[:, 2],
        opt_dirs[:, 0],
        opt_dirs[:, 1],
        opt_dirs[:, 2],
        color="red",
        length=scale * 0.6,
        normalize=True,
        alpha=0.85,
        linewidths=QUIVER_LW_BF_OPT_3D,
    )
    # Draw BF last so it remains visible where vectors overlap.
    ax.quiver(
        bf_refs[:, 0],
        bf_refs[:, 1],
        bf_refs[:, 2],
        bf_dirs[:, 0],
        bf_dirs[:, 1],
        bf_dirs[:, 2],
        color="blue",
        length=scale * 0.95,
        normalize=True,
        alpha=0.9,
        linewidths=QUIVER_LW_BF_OPT_3D,
    )

    if include_per_iter_opt_quivers:
        title = "Optimization trajectory (green), brute force (blue), final optimized (red)"
    elif include_init_opt_quivers:
        title = "Initial direction (green), brute force (blue), final optimized (red)"
    else:
        title = "Brute force (blue) vs final optimized (red)"
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    fig.tight_layout()
    plt.show()


def _comparison_bar_labels(ax, rects, means, y_cap, fmt_ms=False, intish=False):
    """Place compact value labels in a paper-friendly way."""
    if y_cap is None or not np.isfinite(y_cap) or y_cap <= 0:
        pad = 0.5
    else:
        pad = 0.012 * float(y_cap)
    fs = 10.5
    for rect, m in zip(rects, means):
        if not np.isfinite(m):
            continue
        if intish:
            txt = f"{m:.0f}"
        elif fmt_ms:
            txt = f"{m:.1f}"
        else:
            txt = f"{m:.2f}"
        x = float(rect.get_x() + rect.get_width() / 2.0)
        y = float(m) + pad
        ax.text(
            x,
            y,
            txt,
            ha="center",
            va="bottom",
            fontsize=fs,
            rotation=90,
            clip_on=False,
        )


def _format_count_compact(n):
    n = int(n)
    if n >= 1000000:
        return f"{n / 1000000.0:.1f}M".rstrip("0").rstrip(".")
    if n >= 1000:
        return f"{n / 1000.0:.1f}k".rstrip("0").rstrip(".")
    return str(n)


def _comparison_level_paper_label(r):
    """Single-line label like ``L3(2.5k)`` for paper tables (map feature count, not FoV)."""
    count_txt = _format_count_compact(r["map_count"])
    if r.get("level") is not None:
        return f"L{int(r['level'])}({count_txt})"
    return count_txt


def _comparison_level_labels(rows):
    labels = []
    for r in rows:
        count_txt = _format_count_compact(r["map_count"])
        if r.get("level") is not None:
            labels.append(f"L{r['level']}\n({count_txt})")
        else:
            labels.append(count_txt)
    return labels


def plot_comparison_feature_count(rows, out_path):
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping feature count bars.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = _comparison_level_labels(rows)
    x = np.arange(len(rows))
    width = 0.32

    opt_counts = np.array([r["opt_count_mean"] for r in rows])
    bf_counts = np.array([r["bf_count_mean"] for r in rows])

    fig_w = max(6.8, 0.62 * len(rows) + 2.6)
    fig, ax = plt.subplots(figsize=(fig_w, 3.5), constrained_layout=True)
    bars_opt = ax.bar(
        x - width / 2, opt_counts, width, label="Ours", color="tab:blue", alpha=0.88
    )
    bars_bf = ax.bar(
        x + width / 2, bf_counts, width, label="Brute force", color="tab:orange", alpha=0.78
    )
    ax.set_title("Features in field of view", pad=8)
    ax.set_xlabel("Subsample level")
    ax.set_ylabel("Features in FoV")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0, ha="center", linespacing=0.95)
    ax.legend(loc="upper left", frameon=False)
    ax.grid(axis="y", alpha=0.18, linestyle="--", linewidth=0.65)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    y_max = float(np.nanmax([opt_counts, bf_counts]))
    if np.isfinite(y_max):
        ax.set_ylim(0, y_max * 1.28)

    if len(rows) <= 14:
        _comparison_bar_labels(ax, bars_opt, opt_counts, y_max, intish=True)
        _comparison_bar_labels(ax, bars_bf, bf_counts, y_max, intish=True)

    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def plot_comparison_visibility(rows, out_path):
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping visibility bars.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = _comparison_level_labels(rows)
    x = np.arange(len(rows))
    width = 0.32

    opt_vis = np.array([r["opt_vis_mean"] for r in rows])
    bf_vis = np.array([r["bf_vis_mean"] for r in rows])

    fig_w = max(6.8, 0.62 * len(rows) + 2.6)
    fig, ax = plt.subplots(figsize=(fig_w, 3.5), constrained_layout=True)
    bars_opt = ax.bar(
        x - width / 2, opt_vis, width, label="Ours", color="tab:blue", alpha=0.88
    )
    bars_bf = ax.bar(
        x + width / 2, bf_vis, width, label="Brute force", color="tab:orange", alpha=0.78
    )
    ax.set_title("Sigmoid visibility", pad=8)
    ax.set_xlabel("Subsample level")
    ax.set_ylabel("Visibility (sigmoid sum)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0, ha="center", linespacing=0.95)
    ax.legend(loc="upper left", frameon=False)
    ax.grid(axis="y", alpha=0.18, linestyle="--", linewidth=0.65)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    y_max = float(np.nanmax([opt_vis, bf_vis]))
    if np.isfinite(y_max):
        ax.set_ylim(0, y_max * 1.28)

    if len(rows) <= 14:
        _comparison_bar_labels(ax, bars_opt, opt_vis, y_max, fmt_ms=False)
        _comparison_bar_labels(ax, bars_bf, bf_vis, y_max, fmt_ms=False)

    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def plot_comparison_time_visibility(rows, out_path):
    """
    Single figure: time (top) + visibility (bottom), shared x-axis. Saves vertical space vs two files.
    """
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping time+visibility combined plot.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = _comparison_level_labels(rows)
    x = np.arange(len(rows))
    width = 0.32

    opt_time = np.array([r.get("opt_time_mean_ms", np.nan) for r in rows])
    bf_time = np.array([r.get("bf_time_mean_ms", np.nan) for r in rows])
    opt_vis = np.array([r["opt_vis_mean"] for r in rows])
    bf_vis = np.array([r["bf_vis_mean"] for r in rows])

    fig_w = max(6.8, 0.62 * len(rows) + 2.6)
    fig, (ax_t, ax_v) = plt.subplots(
        2,
        1,
        figsize=(fig_w, 6.2),
        sharex=True,
        constrained_layout=True,
    )

    has_time = not (np.all(np.isnan(opt_time)) or np.all(np.isnan(bf_time)))

    if has_time:
        bars_to = ax_t.bar(
            x - width / 2, opt_time, width, label="Ours", color="tab:blue", alpha=0.88
        )
        bars_tb = ax_t.bar(
            x + width / 2, bf_time, width, label="Brute force", color="tab:orange", alpha=0.78
        )
        ax_t.set_ylabel("Time (ms)")
        ax_t.set_title("Per-pose runtime", pad=6)
        ax_t.grid(axis="y", alpha=0.18, linestyle="--", linewidth=0.65)
        ax_t.spines["top"].set_visible(False)
        ax_t.spines["right"].set_visible(False)
        y_max_t = float(np.nanmax([opt_time, bf_time]))
        if np.isfinite(y_max_t):
            ax_t.set_ylim(0, y_max_t * 1.28)
        if len(rows) <= 14:
            _comparison_bar_labels(ax_t, bars_to, opt_time, y_max_t, fmt_ms=True)
            _comparison_bar_labels(ax_t, bars_tb, bf_time, y_max_t, fmt_ms=True)
    else:
        ax_t.text(0.5, 0.5, "Time data not found", ha="center", va="center", transform=ax_t.transAxes)
        ax_t.set_axis_off()

    bars_vo = ax_v.bar(
        x - width / 2, opt_vis, width, label="Ours", color="tab:blue", alpha=0.88
    )
    bars_vb = ax_v.bar(
        x + width / 2, bf_vis, width, label="Brute force", color="tab:orange", alpha=0.78
    )
    ax_v.set_title("Sigmoid visibility", pad=6)
    ax_v.set_xlabel("Subsample level")
    ax_v.set_ylabel("Visibility (sigmoid sum)")
    ax_v.set_xticks(x)
    ax_v.set_xticklabels(labels, rotation=0, ha="center", linespacing=0.95)
    ax_v.grid(axis="y", alpha=0.18, linestyle="--", linewidth=0.65)
    ax_v.spines["top"].set_visible(False)
    ax_v.spines["right"].set_visible(False)
    y_max_v = float(np.nanmax([opt_vis, bf_vis]))
    if np.isfinite(y_max_v):
        ax_v.set_ylim(0, y_max_v * 1.28)
    if len(rows) <= 14:
        _comparison_bar_labels(ax_v, bars_vo, opt_vis, y_max_v, fmt_ms=False)
        _comparison_bar_labels(ax_v, bars_vb, bf_vis, y_max_v, fmt_ms=False)

    if has_time:
        ax_t.tick_params(axis="x", labelbottom=False)
        h0, l0 = ax_t.get_legend_handles_labels()
        ax_t.legend(h0, l0, loc="upper left", frameon=False)
    else:
        h1, l1 = ax_v.get_legend_handles_labels()
        ax_v.legend(h1, l1, loc="upper left", frameon=False)

    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def plot_comparison_time(rows, out_path):
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping time bars.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = _comparison_level_labels(rows)
    x = np.arange(len(rows))
    width = 0.32

    opt_time = np.array([r.get("opt_time_mean_ms", np.nan) for r in rows])
    bf_time = np.array([r.get("bf_time_mean_ms", np.nan) for r in rows])

    fig_w = max(6.8, 0.62 * len(rows) + 2.6)
    fig, ax = plt.subplots(figsize=(fig_w, 3.5), constrained_layout=True)
    if np.all(np.isnan(opt_time)) or np.all(np.isnan(bf_time)):
        ax.text(0.5, 0.5, "Time data not found", ha="center", va="center")
    else:
        bars_opt = ax.bar(
            x - width / 2, opt_time, width, label="Ours", color="tab:blue", alpha=0.88
        )
        bars_bf = ax.bar(
            x + width / 2, bf_time, width, label="Brute force", color="tab:orange", alpha=0.78
        )
        ax.set_title("Per-pose runtime", pad=8)
        ax.set_xlabel("Subsample level")
        ax.set_ylabel("Time (ms)")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=0, ha="center", linespacing=0.95)
        ax.legend(loc="upper left", frameon=False)
        ax.grid(axis="y", alpha=0.18, linestyle="--", linewidth=0.65)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        y_max = float(np.nanmax([opt_time, bf_time]))
        if np.isfinite(y_max):
            ax.set_ylim(0, y_max * 1.28)

        if len(rows) <= 14:
            _comparison_bar_labels(ax, bars_opt, opt_time, y_max, fmt_ms=True)
            _comparison_bar_labels(ax, bars_bf, bf_time, y_max, fmt_ms=True)

    fig.savefig(out_path, dpi=_PAPER_FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def write_comparison_summary(rows, out_csv_path, out_txt_path):
    if not rows:
        return
    import csv

    def _arr(key):
        return np.array([r.get(key, np.nan) for r in rows], dtype=float)

    metrics = [
        ("time_ms", _arr("opt_time_mean_ms"), _arr("bf_time_mean_ms")),
        ("visibility", _arr("opt_vis_mean"), _arr("bf_vis_mean")),
        ("feature_count", _arr("opt_count_mean"), _arr("bf_count_mean")),
    ]

    out_rows = []
    for name, opt_v, bf_v in metrics:
        opt_mean = float(np.nanmean(opt_v)) if np.any(np.isfinite(opt_v)) else float("nan")
        opt_std = float(np.nanstd(opt_v)) if np.any(np.isfinite(opt_v)) else float("nan")
        bf_mean = float(np.nanmean(bf_v)) if np.any(np.isfinite(bf_v)) else float("nan")
        bf_std = float(np.nanstd(bf_v)) if np.any(np.isfinite(bf_v)) else float("nan")
        delta = bf_mean - opt_mean if np.isfinite(bf_mean) and np.isfinite(opt_mean) else float("nan")
        ratio = (
            bf_mean / opt_mean
            if np.isfinite(bf_mean) and np.isfinite(opt_mean) and abs(opt_mean) > 1e-12
            else float("nan")
        )
        out_rows.append(
            {
                "metric": name,
                "opt_mean": opt_mean,
                "opt_std": opt_std,
                "bf_mean": bf_mean,
                "bf_std": bf_std,
                "bf_minus_opt": delta,
                "bf_div_opt": ratio,
                "n": int(len(rows)),
            }
        )

    out_csv_path = Path(out_csv_path)
    out_txt_path = Path(out_txt_path)
    out_csv_path.parent.mkdir(parents=True, exist_ok=True)

    with out_csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "metric",
                "opt_mean",
                "opt_std",
                "bf_mean",
                "bf_std",
                "bf_minus_opt",
                "bf_div_opt",
                "n",
            ],
        )
        writer.writeheader()
        for r in out_rows:
            writer.writerow(r)

    with out_txt_path.open("w", encoding="utf-8") as f:
        if len(rows) == 1:
            f.write("Comparison summary (single level / prefix)\n")
        else:
            f.write("Comparison summary (aggregated over levels / prefixes)\n")
        f.write(f"n={len(rows)}\n\n")
        for r in out_rows:
            f.write(f"{r['metric']}:\n")
            f.write(f"  opt_mean={r['opt_mean']:.6g}, opt_std={r['opt_std']:.6g}\n")
            f.write(f"  bf_mean={r['bf_mean']:.6g}, bf_std={r['bf_std']:.6g}\n")
            f.write(f"  bf-opt={r['bf_minus_opt']:.6g}\n")
            f.write(f"  bf/opt={r['bf_div_opt']:.6g}\n\n")


def write_comparison_paper_table_tex(rows, out_path):
    """
    Tight LaTeX ``tabular`` for the paper: map level + map size, per-pose mean time (ms),
    per-pose mean feature count in FoV (ours vs BF). Requires ``booktabs`` in the preamble.
    """
    if not rows:
        return
    out_path = Path(out_path)
    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))

    def _cell_time(v):
        if v is None or not np.isfinite(v):
            return "---"
        return f"{float(v):.2f}"

    def _cell_feat(v):
        if v is None or not np.isfinite(v):
            return "---"
        x = float(v)
        if abs(x - round(x)) < 0.05:
            return f"{int(round(x))}"
        return f"{x:.1f}"

    lines = [
        "% Auto-generated by plot_monte_carlo_results.py — wrap with \\footnotesize if needed.",
        r"\setlength{\tabcolsep}{4pt}%",
        r"\begin{tabular}{@{}l rrrr@{}}",
        r"\toprule",
        r" & \multicolumn{2}{c}{Time (ms)} & \multicolumn{2}{c}{Features in FoV} \\",
        r"\cmidrule(lr){2-3}\cmidrule(lr){4-5}",
        r"Level & Ours & BF & Ours & BF \\",
        r"\midrule",
    ]
    for r in rows:
        lab = _comparison_level_paper_label(r)
        lab_tex = lab.replace("%", r"\%")
        lines.append(
            f"{lab_tex} & {_cell_time(r.get('opt_time_mean_ms'))} & "
            f"{_cell_time(r.get('bf_time_mean_ms'))} & "
            f"{_cell_feat(r.get('opt_count_mean'))} & "
            f"{_cell_feat(r.get('bf_count_mean'))} \\\\"
        )
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Plot Monte Carlo experiment results.")
    parser.add_argument("--run-dir", default=None, help="Run folder from run_monte_carlo_experiment.py.")
    parser.add_argument("--prefix", default=None, help="Data file prefix (e.g., 0_1_).")
    parser.add_argument("--map", default=None, help="Map file to load.")
    parser.add_argument("--map-name", default=None, help="Named map folder (Map/<name>/<name>_map.csv).")
    parser.add_argument("--visibility-angle", type=float, default=15.0, help="Visibility angle in degrees.")
    parser.add_argument("--ks", type=float, default=15.0, help="Sigmoid sharpness.")
    parser.add_argument("--show", action="store_true", help="Show interactive 3D viewer.")
    parser.add_argument("--plot-sample", type=int, default=0, help="Max map points to render (0 = no downsample).")
    parser.add_argument("--z-scale", type=float, default=4.0, help="Scale Z aspect for 3D plots.")
    parser.add_argument(
        "--per-iter-opt-quivers",
        choices=["include", "exclude"],
        default="include",
        help="Include per-iteration optimization quivers in 2D/3D plots.",
    )
    parser.add_argument(
        "--init-opt-quiver",
        choices=["include", "exclude"],
        default="include",
        help="Include the initial (first-iteration) optimization quiver per pose.",
    )
    parser.add_argument("--per-pose", action="store_true", help="Also plot per-pose metrics.")
    parser.add_argument(
        "--no-bf-opt-mismatch",
        action="store_true",
        help="Skip BF vs optimization mismatch diagnostic (PNG + CSV per prefix).",
    )
    parser.add_argument(
        "--no-failure-map",
        action="store_true",
        help="Skip failure-map overlay (poses colored by BF gap / angle / ||J|| on map).",
    )
    parser.add_argument(
        "--failure-map-plane",
        choices=["xy", "xz"],
        default="xy",
        help="Projection for failure map (default: xy top view).",
    )
    parser.add_argument(
        "--no-runtime-sanity",
        action="store_true",
        help="Skip per-pose runtime anomaly checks and runtime_sanity.txt reports.",
    )

    args = parser.parse_args()
    _apply_paper_mpl_style()
    root = Path(__file__).resolve().parents[1]

    if args.show:
        try:
            import matplotlib
            backend = matplotlib.get_backend().lower()
            if backend.endswith("agg"):
                for candidate in ("TkAgg", "Qt5Agg", "QtAgg", "MacOSX"):
                    try:
                        matplotlib.use(candidate, force=True)
                        break
                    except Exception:
                        continue
        except Exception:
            pass

    if args.map and args.map_name:
        print("Use only one of --map or --map-name.", file=sys.stderr)
        sys.exit(2)

    if args.run_dir:
        run_dir = Path(args.run_dir).resolve()
        data_dir = run_dir / "data"
        output_dir = run_dir
        run_info = read_run_info(run_dir / "run_info.txt")
        map_path = Path(run_info.get("map_path", ""))
        if map_path and not map_path.is_absolute():
            map_path = (run_dir / map_path).resolve()
        if not map_path.exists():
            print("Map not found from run_info; use --map to override.", file=sys.stderr)
            sys.exit(2)
        prev_lines = int(run_info.get("quiversforonepoint_prev_lines", "0"))
        plot_cases = discover_monte_carlo_data_cases(data_dir)
    else:
        if not args.prefix:
            print("Use --run-dir or provide --prefix with --map/--map-name.", file=sys.stderr)
            sys.exit(2)
        if not args.map and not args.map_name:
            print("Use --run-dir or provide --map/--map-name.", file=sys.stderr)
            sys.exit(2)
        data_dir = Path(".")
        output_dir = data_dir
        plot_cases = [(data_dir, args.prefix, None)]
        if args.map_name:
            name = args.map_name
            candidates = [
                root / "Map" / name / f"{name}_map.csv",
                root / "Map" / f"{name}_map.csv",
            ]
            map_path = None
            for cand in candidates:
                if cand.exists():
                    map_path = cand
                    break
            if map_path is None:
                print(f"Map not found for name '{name}'.", file=sys.stderr)
                sys.exit(2)
        else:
            map_path = Path(args.map)
        prev_lines = 0

    map_cache = {}
    comparison_rows = []
    runtime_sanity_sections = []
    level_plot_buffers = []

    def load_map_points(path):
        key = str(path)
        if key in map_cache:
            return map_cache[key]
        pts = load_csv(path)
        map_cache[key] = pts
        return pts

    dense_map_path = resolve_base_map(map_path)
    dense_map_points = load_map_points(dense_map_path) if dense_map_path else None

    densest_view = None  # (map_count, points_for_view, log_rows, bf_quivers, opt_quivers)

    if args.run_dir and not plot_cases:
        print(
            "No single_run_rotated_quivers.csv found under data/ or data/level_*/.",
            file=sys.stderr,
        )
        sys.exit(2)

    for data_subdir, prefix, level_hint in plot_cases:
        if level_hint is not None:
            prefix_map_path = resolve_map_for_subsample_level(map_path, level_hint)
        else:
            prefix_map_path = resolve_map_for_prefix(prefix, map_path)
        if prefix_map_path is None or not prefix_map_path.exists():
            print(f"Map not found for prefix {prefix}; using base map.", file=sys.stderr)
            prefix_map_path = map_path
        map_points = load_map_points(prefix_map_path)

        base_for_plots = run_dir if args.run_dir else output_dir.resolve()
        plot_out_dir = plot_case_output_dir(base_for_plots, level_hint, prefix)
        plot_out_dir.mkdir(parents=True, exist_ok=True)

        out_tag = f"level_{level_hint}_" if level_hint is not None else prefix

        opt_path = data_subdir / f"{prefix}single_run_rotated_quivers.csv"
        bf_path = data_subdir / f"{prefix}single_run_brute_force_rotated_quivers.csv"
        if not opt_path.exists() or not bf_path.exists():
            print(f"Missing quiver files under {data_subdir} (prefix {repr(prefix)}); skipping.")
            continue
        opt_quivers = load_csv(opt_path, skiprows=1)
        bf_quivers = load_csv(bf_path, skiprows=1)

        per_prefix_log = data_subdir / f"{prefix}quiversforonepoint.csv"
        if per_prefix_log.exists():
            log_rows = load_csv(per_prefix_log)
        else:
            log_path = data_subdir / "quiversforonepoint.csv"
            if not log_path.exists() and data_dir != data_subdir:
                log_path = data_dir / "quiversforonepoint.csv"
            log_rows = load_csv_tail(log_path, prev_lines) if log_path.exists() else np.zeros((0, 0))

        viz_opt_quivers, viz_bf_quivers, viz_log_rows, selected_z = select_single_pose_z_layer(
            opt_quivers, bf_quivers, log_rows
        )

        per_pose = compute_per_pose_metrics(
            map_points, opt_quivers, bf_quivers, args.visibility_angle, args.ks
        )

        if not args.no_bf_opt_mismatch:
            mismatch_png = plot_out_dir / f"{out_tag}bf_vs_opt_mismatch.png"
            mismatch_csv = plot_out_dir / f"{out_tag}bf_vs_opt_mismatch.csv"
            title_suffix = f"level {level_hint}" if level_hint is not None else prefix.strip("_") or "default"
            plot_bf_vs_opt_mismatch(
                per_pose, opt_quivers, mismatch_png, title_suffix=title_suffix
            )
            write_bf_vs_opt_mismatch_csv(mismatch_csv, per_pose, opt_quivers)

        if not args.no_failure_map:
            failure_png = plot_out_dir / f"{out_tag}failure_map.png"
            title_suffix = f"level {level_hint}" if level_hint is not None else prefix.strip("_") or "default"
            plot_failure_maps(
                dense_map_points if dense_map_points is not None else map_points,
                per_pose,
                opt_quivers,
                failure_png,
                log_rows=log_rows,
                sample_points=args.plot_sample,
                title_suffix=title_suffix,
                plane=args.failure_map_plane,
            )

        if args.per_pose:
            per_pose_csv = plot_out_dir / f"{out_tag}per_pose_metrics.csv"
            per_pose_png = plot_out_dir / f"{out_tag}per_pose_metrics.png"
            write_per_pose_metrics(per_pose_csv, per_pose)
            plot_per_pose_metrics(per_pose, per_pose_png)

        metrics = compute_iteration_averages(log_rows, bf_quivers, map_points, args.visibility_angle, args.ks)
        level_label = f"L{level_hint}" if level_hint is not None else (prefix.strip("_") or "run")
        level_plot_buffers.append(
            {
                "per_pose": per_pose,
                "opt_quivers": opt_quivers,
                "bf_quivers": bf_quivers,
                "log_rows": log_rows,
                "map_points": map_points,
                "level": level_hint,
                "label": level_label,
                "metrics": metrics,
                "prefix": prefix,
            }
        )
        if metrics is not None:
            avgs, counts = metrics
            avg_csv = plot_out_dir / f"{out_tag}avg_metrics.csv"
            avg_png = plot_out_dir / f"{out_tag}avg_metrics.png"
            write_avg_metrics(avg_csv, avgs, counts)
            plot_avg_metrics(avgs, avg_png)

        quiver_png = plot_out_dir / f"{out_tag}quivers.png"
        plot_quivers(
            dense_map_points if dense_map_points is not None else map_points,
            viz_opt_quivers,
            viz_bf_quivers,
            quiver_png,
            sample_points=args.plot_sample,
            z_scale=args.z_scale,
        )

        top_view_png = plot_out_dir / f"{out_tag}top_view_quivers.png"
        plot_top_view_quivers(
            dense_map_points if dense_map_points is not None else map_points,
            viz_opt_quivers,
            viz_bf_quivers,
            viz_log_rows,
            top_view_png,
            sample_points=args.plot_sample,
            include_per_iter_opt_quivers=args.per_iter_opt_quivers == "include",
            include_init_opt_quivers=args.init_opt_quiver == "include",
        )

        if args.show:
            map_count = int(map_points.shape[0])
            points_for_view = dense_map_points if dense_map_points is not None else map_points
            candidate = (map_count, points_for_view, viz_log_rows, viz_bf_quivers, viz_opt_quivers, selected_z)
            if densest_view is None or candidate[0] > densest_view[0]:
                densest_view = candidate

        opt_counts = per_pose[:, 3]
        bf_counts = per_pose[:, 4]
        opt_count_mean = float(np.mean(opt_counts)) if opt_counts.size else 0.0
        bf_count_mean = float(np.mean(bf_counts)) if bf_counts.size else 0.0
        opt_count_std = float(np.std(opt_counts)) if opt_counts.size else 0.0
        bf_count_std = float(np.std(bf_counts)) if bf_counts.size else 0.0

        opt_vis = per_pose[:, 0]
        bf_vis = per_pose[:, 1]
        opt_vis_mean = float(np.mean(opt_vis)) if opt_vis.size else 0.0
        bf_vis_mean = float(np.mean(bf_vis)) if bf_vis.size else 0.0
        opt_vis_std = float(np.std(opt_vis)) if opt_vis.size else 0.0
        bf_vis_std = float(np.std(bf_vis)) if bf_vis.size else 0.0

        opt_time_path = data_subdir / f"{prefix}optimizer_avg_time_file.csv"
        bf_time_path = data_subdir / f"{prefix}brute_force_avg_time_file.csv"
        opt_time_stats = load_time_stats(opt_time_path)
        bf_time_stats = load_time_stats(bf_time_path)
        opt_time_mean_ms = opt_time_stats[0] / 1000.0 if opt_time_stats else np.nan
        opt_time_std_ms = opt_time_stats[1] / 1000.0 if opt_time_stats else np.nan
        bf_time_mean_ms = bf_time_stats[0] / 1000.0 if bf_time_stats else np.nan
        bf_time_std_ms = bf_time_stats[1] / 1000.0 if bf_time_stats else np.nan

        if not args.no_runtime_sanity:
            opt_us = load_time_per_pose_us(opt_time_path)
            bf_us = load_time_per_pose_us(bf_time_path)
            opt_st = analyze_runtime_times_us(opt_us, "optimization") if opt_us is not None else None
            bf_st = analyze_runtime_times_us(bf_us, "brute_force") if bf_us is not None else None
            sanity_text = format_runtime_sanity_report(opt_time_path, bf_time_path, opt_st, bf_st)
            write_runtime_sanity_file(plot_out_dir / "runtime_sanity.txt", sanity_text)
            runtime_sanity_sections.append((plot_out_dir, sanity_text))
            for st, p in ((opt_st, opt_time_path), (bf_st, bf_time_path)):
                if st and st["anomalies"]:
                    print(
                        f"[runtime sanity] {p.name}: {len(st['anomalies'])} flagged row(s) "
                        f"(see {plot_out_dir / 'runtime_sanity.txt'})",
                        file=sys.stderr,
                    )

        comp_level = (
            level_hint
            if level_hint is not None
            else parse_prefix_level(prefix)
        )
        comparison_rows.append(
            {
                "prefix": out_tag,
                "level": comp_level,
                "map_count": int(map_points.shape[0]),
                "opt_count_mean": opt_count_mean,
                "opt_count_std": opt_count_std,
                "bf_count_mean": bf_count_mean,
                "bf_count_std": bf_count_std,
                "opt_vis_mean": opt_vis_mean,
                "opt_vis_std": opt_vis_std,
                "bf_vis_mean": bf_vis_mean,
                "bf_vis_std": bf_vis_std,
                "opt_time_mean_ms": opt_time_mean_ms,
                "opt_time_std_ms": opt_time_std_ms,
                "bf_time_mean_ms": bf_time_mean_ms,
                "bf_time_std_ms": bf_time_std_ms,
            }
        )

        case_row = comparison_rows[-1]
        plot_comparison_feature_count([case_row], plot_out_dir / "comparison_feature_count.png")
        plot_comparison_visibility([case_row], plot_out_dir / "comparison_visibility.png")
        plot_comparison_time([case_row], plot_out_dir / "comparison_time_cost.png")
        plot_comparison_time_visibility([case_row], plot_out_dir / "comparison_time_visibility.png")
        write_comparison_summary(
            [case_row],
            plot_out_dir / "comparison_summary.csv",
            plot_out_dir / "comparison_summary.txt",
        )

    if comparison_rows:
        count_png = output_dir / "comparison_feature_count.png"
        vis_png = output_dir / "comparison_visibility.png"
        time_png = output_dir / "comparison_time_cost.png"
        time_vis_png = output_dir / "comparison_time_visibility.png"
        plot_comparison_feature_count(comparison_rows, count_png)
        plot_comparison_visibility(comparison_rows, vis_png)
        plot_comparison_time(comparison_rows, time_png)
        plot_comparison_time_visibility(comparison_rows, time_vis_png)
        write_comparison_summary(
            comparison_rows,
            output_dir / "comparison_summary.csv",
            output_dir / "comparison_summary.txt",
        )
        write_comparison_paper_table_tex(comparison_rows, output_dir / "comparison_paper_table.tex")
        if args.run_dir:
            print(
                f"Run-level comparison: {output_dir / 'comparison_time_visibility.png'} (time+vis), "
                f"{output_dir / 'comparison_visibility.png'}, "
                f"{output_dir / 'comparison_summary.csv'}, "
                f"{output_dir / 'comparison_paper_table.tex'} "
                f"({len(comparison_rows)} level(s)/prefix(es)). "
                f"Per-case plots under {output_dir / 'plots'}/.",
                file=sys.stderr,
            )

    if len(level_plot_buffers) >= 2:
        write_total_run_outputs(output_dir, level_plot_buffers, dense_map_points, args)

    if runtime_sanity_sections and not args.no_runtime_sanity:
        parts = []
        for plot_dir, report in runtime_sanity_sections:
            try:
                rel = plot_dir.relative_to(output_dir.resolve())
                header = f"=== {rel.as_posix()} ==="
            except ValueError:
                header = f"=== {plot_dir} ==="
            parts.append(header + "\n" + report)
        write_runtime_sanity_file(output_dir / "runtime_sanity_all.txt", "\n\n".join(parts))
        if args.run_dir:
            print(
                f"Runtime sanity (all cases): {output_dir / 'runtime_sanity_all.txt'}",
                file=sys.stderr,
            )

    if args.show and densest_view is not None:
        _, points_for_view, log_rows, bf_quivers, opt_quivers, selected_z = densest_view
        if selected_z is not None:
            print(f"3D viewer using a single pose Z slice at z={selected_z:.3f}")
        interactive_viewer_all(
            points_for_view,
            log_rows,
            bf_quivers,
            opt_quivers,
            sample_points=args.plot_sample,
            z_scale=args.z_scale,
            include_per_iter_opt_quivers=args.per_iter_opt_quivers == "include",
            include_init_opt_quivers=args.init_opt_quiver == "include",
        )


if __name__ == "__main__":
    main()
