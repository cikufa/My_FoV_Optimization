#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


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
    path = Path(str(rel_path))
    if path.is_absolute():
        return path
    root = manifest_path.parent.parent
    return root / path


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
    ax.set_xlim(mins[0], maxs[0])
    ax.set_ylim(mins[1], maxs[1])
    ax.set_zlim(mins[2], maxs[2])
    ranges = maxs - mins
    ranges[ranges == 0] = 1.0
    z_scale = max(float(z_scale), 1e-3)
    ax.set_box_aspect((ranges[0], ranges[1], ranges[2] * z_scale))


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


def plot_top_view_quivers(points, opt_quivers, bf_quivers, log_rows, out_path, sample_points=0):
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
    refs_all = log_rows[:, 1:3]
    dirs_all = _normalize_xy(log_rows[:, 4:6])
    ax.quiver(
        refs_all[:, 0], refs_all[:, 1],
        dirs_all[:, 0] * scale, dirs_all[:, 1] * scale,
        color="dimgray", alpha=0.18, angles="xy", scale_units="xy", scale=1,
    )

    # Initial quiver per pose (first logged iteration).
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
        dirs_init[:, 0] * scale, dirs_init[:, 1] * scale,
        color="red", alpha=0.7, angles="xy", scale_units="xy", scale=1, label="Init",
    )

    # Brute-force quivers.
    if bf_quivers is not None and bf_quivers.size:
        refs_bf = bf_quivers[:, 0:2]
        dirs_bf = _normalize_xy(bf_quivers[:, 6:8])
        ax.quiver(
            refs_bf[:, 0], refs_bf[:, 1],
            dirs_bf[:, 0] * scale, dirs_bf[:, 1] * scale,
            color="blue", alpha=0.6, angles="xy", scale_units="xy", scale=1, label="BF",
        )

    # Final optimized quivers.
    refs_final = opt_quivers[:, 0:2]
    dirs_final = _normalize_xy(opt_quivers[:, 3:5])
    ax.quiver(
        refs_final[:, 0], refs_final[:, 1],
        dirs_final[:, 0] * scale, dirs_final[:, 1] * scale,
        color="green", alpha=0.8, angles="xy", scale_units="xy", scale=1, label="Final",
    )

    if bounds is not None:
        mins, maxs = bounds
        ax.set_xlim(mins[0], maxs[0])
        ax.set_ylim(mins[1], maxs[1])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Top View: Init (red), Iter (gray), Final (green)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def compute_visibility(points, ref_point, direction, visibility_angle_deg, ks):
    vecs = points - ref_point
    norms = np.linalg.norm(vecs, axis=1)
    mask = norms > 1e-9
    if not np.any(mask):
        return 0.0
    vecs = vecs[mask] / norms[mask][:, None]
    direction = direction / np.linalg.norm(direction)
    cos_theta = vecs @ direction
    cos_alpha = np.cos(np.deg2rad(visibility_angle_deg))
    visibility = 1.0 / (1.0 + np.exp(-ks * (cos_theta - cos_alpha)))
    return float(np.sum(visibility))


def compute_count_in_fov(points, ref_point, direction, visibility_angle_deg):
    vecs = points - ref_point
    norms = np.linalg.norm(vecs, axis=1)
    mask = norms > 1e-9
    if not np.any(mask):
        return 0
    vecs = vecs[mask] / norms[mask][:, None]
    direction = direction / np.linalg.norm(direction)
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


def load_time_stats(path, max_rows=None):
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
    mean = float(np.mean(values))
    std = float(np.std(values))
    return mean, std


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
    axes[0].plot(iters, avgs[:, 0], marker="o")
    axes[0].set_title("Avg Visibility Diff (BF - Opt)")
    axes[0].set_xlabel("Iteration")
    axes[0].set_ylabel("Visibility Diff")

    axes[1].plot(iters, avgs[:, 1], marker="o")
    axes[1].set_title("Avg Jacobian Norm")
    axes[1].set_xlabel("Iteration")
    axes[1].set_ylabel("Jacobian Norm")

    axes[2].plot(iters, avgs[:, 2], marker="o")
    axes[2].set_title("Avg Feature Count")
    axes[2].set_xlabel("Iteration")
    axes[2].set_ylabel("Feature Count")

    axes[3].plot(iters, avgs[:, 3], marker="o")
    axes[3].set_title("Avg Angle Diff (deg)")
    axes[3].set_xlabel("Iteration")
    axes[3].set_ylabel("Angle Diff")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_per_pose_metrics(data, out_path):
    import matplotlib.pyplot as plt

    pose_ids = data[:, 6]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes = axes.ravel()

    axes[0].plot(pose_ids, data[:, 0], marker=".", linestyle="none", label="Opt")
    axes[0].plot(pose_ids, data[:, 1], marker=".", linestyle="none", label="BF", alpha=0.6)
    axes[0].set_title("Per-Pose Visibility")
    axes[0].set_xlabel("Pose ID")
    axes[0].set_ylabel("Visibility")
    axes[0].legend()

    axes[1].plot(pose_ids, data[:, 2], marker=".", linestyle="none", color="tab:red")
    axes[1].set_title("Per-Pose Visibility Diff (BF - Opt)")
    axes[1].set_xlabel("Pose ID")
    axes[1].set_ylabel("Visibility Diff")

    axes[2].plot(pose_ids, data[:, 3], marker=".", linestyle="none", label="Opt")
    axes[2].plot(pose_ids, data[:, 4], marker=".", linestyle="none", label="BF", alpha=0.6)
    axes[2].set_title("Per-Pose Feature Count")
    axes[2].set_xlabel("Pose ID")
    axes[2].set_ylabel("Count")
    axes[2].legend()

    axes[3].plot(pose_ids, data[:, 5], marker=".", linestyle="none", color="tab:purple")
    axes[3].set_title("Per-Pose Angle Diff (deg)")
    axes[3].set_xlabel("Pose ID")
    axes[3].set_ylabel("Angle Diff")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
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
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="gray", alpha=0.45, s=3)
    else:
        ax.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            c=density, cmap="Greys", vmin=0, vmax=1,
            alpha=0.45, s=3, depthshade=False,
        )
    pose_points = opt_quivers[:, 0:3]
    ax.scatter(pose_points[:, 0], pose_points[:, 1], pose_points[:, 2], c="black", alpha=0.6, s=6)

    bounds = compute_bounds(points, pose_points)
    scale = quiver_length_from_bounds(bounds)
    for row in opt_quivers:
        ref = row[0:3]
        opt_dir = row[3:6]
        ax.quiver(ref[0], ref[1], ref[2], opt_dir[0], opt_dir[1], opt_dir[2],
                  color="orange", length=scale, normalize=True, alpha=0.8)
    for row in bf_quivers:
        ref = row[0:3]
        bf_dir = row[6:9]
        ax.quiver(ref[0], ref[1], ref[2], bf_dir[0], bf_dir[1], bf_dir[2],
                  color="blue", length=scale, normalize=True, alpha=0.5)

    ax.set_title("BF (blue) vs Opt (orange) Quivers")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    apply_axes_bounds(ax, bounds, z_scale=z_scale)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def interactive_viewer_all(points, log_rows, bf_quivers, opt_quivers, sample_points=0, z_scale=1.0):
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
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="gray", alpha=0.45, s=3)
    else:
        ax.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            c=density, cmap="Greys", vmin=0, vmax=1,
            alpha=0.45, s=3, depthshade=False,
        )
    pose_points = opt_quivers[:, 0:3]
    ax.scatter(pose_points[:, 0], pose_points[:, 1], pose_points[:, 2], c="black", alpha=0.6, s=6)

    bounds = compute_bounds(points, pose_points)
    apply_axes_bounds(ax, bounds, z_scale=z_scale)
    scale = quiver_length_from_bounds(bounds)

    refs = log_rows[:, 1:4]
    dirs = log_rows[:, 4:7]
    ax.quiver(refs[:, 0], refs[:, 1], refs[:, 2],
              dirs[:, 0], dirs[:, 1], dirs[:, 2],
              color="dimgray", length=scale, normalize=True, alpha=0.15)

    bf_refs = bf_quivers[:, 0:3]
    bf_dirs = bf_quivers[:, 6:9]
    ax.quiver(bf_refs[:, 0], bf_refs[:, 1], bf_refs[:, 2],
              bf_dirs[:, 0], bf_dirs[:, 1], bf_dirs[:, 2],
              color="blue", length=scale, normalize=True, alpha=0.35)

    opt_refs = opt_quivers[:, 0:3]
    opt_dirs = opt_quivers[:, 3:6]
    ax.quiver(opt_refs[:, 0], opt_refs[:, 1], opt_refs[:, 2],
              opt_dirs[:, 0], opt_dirs[:, 1], opt_dirs[:, 2],
              color="green", length=scale, normalize=True, alpha=0.6)

    ax.set_title("All Iterations (gray) + BF (blue) + Final Opt (green)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    fig.tight_layout()
    plt.show()


def plot_comparison_feature_count(rows, out_path):
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping feature count bars.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = [
        f"L{r['level']} ({r['map_count']})" if r.get("level") is not None else str(r["map_count"])
        for r in rows
    ]
    x = np.arange(len(rows))
    width = 0.35

    opt_counts = np.array([r["opt_count_mean"] for r in rows])
    opt_counts_std = np.array([r["opt_count_std"] for r in rows])
    bf_counts = np.array([r["bf_count_mean"] for r in rows])
    bf_counts_std = np.array([r["bf_count_std"] for r in rows])

    fig, ax = plt.subplots(figsize=(7.5, 5))
    bars_opt = ax.bar(x - width / 2, opt_counts, width, yerr=opt_counts_std,
                      capsize=4, label="Ours", color="tab:blue", alpha=0.65)
    bars_bf = ax.bar(x + width / 2, bf_counts, width, yerr=bf_counts_std,
                     capsize=4, label="Brute Force", color="tab:orange", alpha=0.55)
    ax.set_title("Features in FOV comparison")
    ax.set_xlabel("Number of Features in the Map")
    ax.set_ylabel("Features in FOV")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    y_max = np.nanmax([opt_counts + opt_counts_std, bf_counts + bf_counts_std])
    if np.isfinite(y_max):
        ax.set_ylim(0, y_max * 1.15)

    for bars in (bars_opt, bars_bf):
        for rect in bars:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width() / 2, height,
                    f"{height:.2f}", ha="center", va="bottom", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_comparison_visibility(rows, out_path):
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping visibility bars.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = [
        f"L{r['level']} ({r['map_count']})" if r.get("level") is not None else str(r["map_count"])
        for r in rows
    ]
    x = np.arange(len(rows))
    width = 0.35

    opt_vis = np.array([r["opt_vis_mean"] for r in rows])
    opt_vis_std = np.array([r["opt_vis_std"] for r in rows])
    bf_vis = np.array([r["bf_vis_mean"] for r in rows])
    bf_vis_std = np.array([r["bf_vis_std"] for r in rows])

    fig, ax = plt.subplots(figsize=(7.5, 5))
    bars_opt = ax.bar(x - width / 2, opt_vis, width, yerr=opt_vis_std,
                      capsize=4, label="Ours", color="tab:blue", alpha=0.65)
    bars_bf = ax.bar(x + width / 2, bf_vis, width, yerr=bf_vis_std,
                     capsize=4, label="Brute Force", color="tab:orange", alpha=0.55)
    ax.set_title("Visibility comparison")
    ax.set_xlabel("Number of Features in the Map")
    ax.set_ylabel("Visibility (sigmoid sum)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    y_max = np.nanmax([opt_vis + opt_vis_std, bf_vis + bf_vis_std])
    if np.isfinite(y_max):
        ax.set_ylim(0, y_max * 1.15)

    for bars in (bars_opt, bars_bf):
        for rect in bars:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width() / 2, height,
                    f"{height:.2f}", ha="center", va="bottom", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_comparison_time(rows, out_path):
    import matplotlib.pyplot as plt

    if not rows:
        print("No comparison data available; skipping time bars.")
        return

    rows = sorted(rows, key=lambda r: (r.get("level") is None, r.get("level", r["map_count"])))
    labels = [
        f"L{r['level']} ({r['map_count']})" if r.get("level") is not None else str(r["map_count"])
        for r in rows
    ]
    x = np.arange(len(rows))
    width = 0.35

    opt_time = np.array([r.get("opt_time_mean_ms", np.nan) for r in rows])
    opt_time_std = np.array([r.get("opt_time_std_ms", np.nan) for r in rows])
    bf_time = np.array([r.get("bf_time_mean_ms", np.nan) for r in rows])
    bf_time_std = np.array([r.get("bf_time_std_ms", np.nan) for r in rows])

    fig, ax = plt.subplots(figsize=(7.5, 5))
    if np.all(np.isnan(opt_time)) or np.all(np.isnan(bf_time)):
        ax.text(0.5, 0.5, "Time data not found", ha="center", va="center")
    else:
        bars_opt = ax.bar(x - width / 2, opt_time, width, yerr=opt_time_std,
                          capsize=4, label="Ours", color="tab:blue", alpha=0.65)
        bars_bf = ax.bar(x + width / 2, bf_time, width, yerr=bf_time_std,
                         capsize=4, label="Brute Force", color="tab:orange", alpha=0.55)
        ax.set_title("Average Time Cost comparison")
        ax.set_xlabel("Number of Features in the Map")
        ax.set_ylabel("Average Time (ms)")
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.legend()
        y_max = np.nanmax([opt_time + opt_time_std, bf_time + bf_time_std])
        if np.isfinite(y_max):
            ax.set_ylim(0, y_max * 1.15)

        for bars in (bars_opt, bars_bf):
            for rect in bars:
                height = rect.get_height()
                ax.text(rect.get_x() + rect.get_width() / 2, height,
                        f"{height:.2f}", ha="center", va="bottom", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


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
    parser.add_argument("--z-scale", type=float, default=2.0, help="Scale Z aspect for 3D plots.")
    parser.add_argument("--per-pose", action="store_true", help="Also plot per-pose metrics.")

    args = parser.parse_args()
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
        run_dir = Path(args.run_dir)
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
        prefixes = [p.name.replace("single_run_rotated_quivers.csv", "") for p in data_dir.glob("*single_run_rotated_quivers.csv")]
    else:
        if not args.prefix:
            print("Use --run-dir or provide --prefix with --map/--map-name.", file=sys.stderr)
            sys.exit(2)
        if not args.map and not args.map_name:
            print("Use --run-dir or provide --map/--map-name.", file=sys.stderr)
            sys.exit(2)
        data_dir = Path(".")
        output_dir = data_dir
        prefixes = [args.prefix]
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

    def load_map_points(path):
        key = str(path)
        if key in map_cache:
            return map_cache[key]
        pts = load_csv(path)
        map_cache[key] = pts
        return pts

    dense_map_path = resolve_base_map(map_path)
    dense_map_points = load_map_points(dense_map_path) if dense_map_path else None

    for prefix in prefixes:
        prefix_map_path = resolve_map_for_prefix(prefix, map_path)
        if prefix_map_path is None or not prefix_map_path.exists():
            print(f"Map not found for prefix {prefix}; using base map.", file=sys.stderr)
            prefix_map_path = map_path
        map_points = load_map_points(prefix_map_path)

        opt_path = data_dir / f"{prefix}single_run_rotated_quivers.csv"
        bf_path = data_dir / f"{prefix}single_run_brute_force_rotated_quivers.csv"
        if not opt_path.exists() or not bf_path.exists():
            print(f"Missing quiver files for prefix {prefix}; skipping.")
            continue
        opt_quivers = load_csv(opt_path, skiprows=1)
        bf_quivers = load_csv(bf_path, skiprows=1)

        log_path = data_dir / "quiversforonepoint.csv"
        log_rows = load_csv_tail(log_path, prev_lines) if log_path.exists() else np.zeros((0, 0))

        per_pose = compute_per_pose_metrics(
            map_points, opt_quivers, bf_quivers, args.visibility_angle, args.ks
        )

        if args.per_pose:
            per_pose_csv = output_dir / f"{prefix}per_pose_metrics.csv"
            per_pose_png = output_dir / f"{prefix}per_pose_metrics.png"
            write_per_pose_metrics(per_pose_csv, per_pose)
            plot_per_pose_metrics(per_pose, per_pose_png)

        metrics = compute_iteration_averages(log_rows, bf_quivers, map_points, args.visibility_angle, args.ks)
        if metrics is not None:
            avgs, counts = metrics
            avg_csv = output_dir / f"{prefix}avg_metrics.csv"
            avg_png = output_dir / f"{prefix}avg_metrics.png"
            write_avg_metrics(avg_csv, avgs, counts)
            plot_avg_metrics(avgs, avg_png)

        quiver_png = output_dir / f"{prefix}quivers.png"
        plot_quivers(
            dense_map_points if dense_map_points is not None else map_points,
            opt_quivers,
            bf_quivers,
            quiver_png,
            sample_points=args.plot_sample,
            z_scale=args.z_scale,
        )

        top_view_png = output_dir / f"{prefix}top_view_quivers.png"
        plot_top_view_quivers(
            dense_map_points if dense_map_points is not None else map_points,
            opt_quivers,
            bf_quivers,
            log_rows,
            top_view_png,
            sample_points=args.plot_sample,
        )

        if args.show:
            interactive_viewer_all(
                dense_map_points if dense_map_points is not None else map_points,
                log_rows,
                bf_quivers,
                opt_quivers,
                sample_points=args.plot_sample,
                z_scale=args.z_scale,
            )

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

        opt_time_path = data_dir / f"{prefix}optimizer_avg_time_file.csv"
        bf_time_path = data_dir / f"{prefix}brute_force_avg_time_file.csv"
        opt_time_stats = load_time_stats(opt_time_path)
        bf_time_stats = load_time_stats(bf_time_path)
        opt_time_mean_ms = opt_time_stats[0] / 1000.0 if opt_time_stats else np.nan
        opt_time_std_ms = opt_time_stats[1] / 1000.0 if opt_time_stats else np.nan
        bf_time_mean_ms = bf_time_stats[0] / 1000.0 if bf_time_stats else np.nan
        bf_time_std_ms = bf_time_stats[1] / 1000.0 if bf_time_stats else np.nan

        comparison_rows.append(
            {
                "prefix": prefix,
                "level": parse_prefix_level(prefix),
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

    if comparison_rows:
        count_png = output_dir / "comparison_feature_count.png"
        vis_png = output_dir / "comparison_visibility.png"
        time_png = output_dir / "comparison_time_cost.png"
        plot_comparison_feature_count(comparison_rows, count_png)
        plot_comparison_visibility(comparison_rows, vis_png)
        plot_comparison_time(comparison_rows, time_png)


if __name__ == "__main__":
    main()
