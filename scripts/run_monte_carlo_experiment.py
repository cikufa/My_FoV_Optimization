#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np


def run_cmd(cmd, cwd=None):
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd)


def load_csv(path, skiprows=0):
    if not path.exists():
        raise SystemExit(f"Missing file: {path}")
    return np.loadtxt(str(path), delimiter=",", skiprows=skiprows)


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


def compute_bf_visibility(points, bf_quivers, visibility_angle_deg, ks):
    bf_visibility = []
    for row in bf_quivers:
        ref = row[0:3]
        bf_dir = row[6:9]
        bf_visibility.append(compute_visibility(points, ref, bf_dir, visibility_angle_deg, ks))
    return np.array(bf_visibility, dtype=float)


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


def compute_per_pose_metrics(points, opt_quivers, bf_quivers, visibility_angle_deg, ks):
    n = opt_quivers.shape[0]
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
        feature_count = row[7]
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


def write_avg_metrics(path, avgs, counts):
    with path.open("w", encoding="utf-8") as f:
        f.write("iteration,avg_visibility_diff,avg_jacobian,avg_feature_count,avg_angle_diff,count\n")
        for i in range(avgs.shape[0]):
            f.write(
                f"{i},{avgs[i,0]:.6f},{avgs[i,1]:.6f},{avgs[i,2]:.6f},{avgs[i,3]:.6f},{counts[i]}\n"
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


def interactive_viewer(points, log_rows, bf_quivers, opt_quivers, out_dir, sample_points=5000):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    if log_rows.size == 0:
        print("No per-iteration log rows found; skipping interactive viewer.")
        return

    pose_ids = log_rows[:, 0].astype(int)
    iter_dirs = defaultdict(list)
    refs = {}
    for row, pid in zip(log_rows, pose_ids):
        if pid not in refs:
            refs[pid] = row[1:4]
        iter_dirs[pid].append(row[4:7])

    poses = sorted(iter_dirs.keys())
    if not poses:
        print("No pose data to visualize.")
        return

    if points.shape[0] > sample_points:
        idx = np.random.choice(points.shape[0], sample_points, replace=False)
        points = points[idx]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="gray", alpha=0.08, s=1)

    quiver_artists = []
    text_artist = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

    def draw_pose_iter(pose_idx, iter_idx):
        nonlocal quiver_artists
        for artist in quiver_artists:
            artist.remove()
        quiver_artists = []
        pid = poses[pose_idx]
        ref = refs[pid]
        bf_dir = bf_quivers[pid, 6:9] if pid < bf_quivers.shape[0] else None
        opt_final = opt_quivers[pid, 3:6] if pid < opt_quivers.shape[0] else None
        iter_dir = iter_dirs[pid][iter_idx]

        scale = 0.6
        if bf_dir is not None:
            q_bf = ax.quiver(ref[0], ref[1], ref[2], bf_dir[0], bf_dir[1], bf_dir[2],
                             color="blue", length=scale, normalize=True, label="BF")
            quiver_artists.append(q_bf)
        q_it = ax.quiver(ref[0], ref[1], ref[2], iter_dir[0], iter_dir[1], iter_dir[2],
                         color="orange", length=scale, normalize=True, label="Iter")
        quiver_artists.append(q_it)
        if opt_final is not None:
            q_opt = ax.quiver(ref[0], ref[1], ref[2], opt_final[0], opt_final[1], opt_final[2],
                              color="green", length=scale, normalize=True, label="Final")
            quiver_artists.append(q_opt)
        text_artist.set_text(f"Pose {pid} | Iter {iter_idx}/{len(iter_dirs[pid]) - 1}")
        fig.canvas.draw_idle()

    ax_pose = fig.add_axes([0.2, 0.03, 0.6, 0.02])
    ax_iter = fig.add_axes([0.2, 0.00, 0.6, 0.02])
    pose_slider = Slider(ax_pose, "Pose", 0, len(poses) - 1, valinit=0, valstep=1)
    iter_slider = Slider(ax_iter, "Iter", 0, len(iter_dirs[poses[0]]) - 1, valinit=0, valstep=1)

    def update(_):
        pose_idx = int(pose_slider.val)
        max_iter = len(iter_dirs[poses[pose_idx]]) - 1
        if iter_slider.val > max_iter:
            iter_slider.set_val(max_iter)
        iter_slider.valmax = max_iter
        iter_slider.ax.set_xlim(iter_slider.valmin, iter_slider.valmax)
        draw_pose_iter(pose_idx, int(iter_slider.val))

    pose_slider.on_changed(update)
    iter_slider.on_changed(update)

    draw_pose_iter(0, 0)
    plt.show()


def parse_grid(s):
    if s is None:
        return None
    normalized = s.lower().replace("x", ",")
    parts = [p.strip() for p in normalized.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError("grid must be in form 'X,Y,Z' or 'XxYxZ'")
    try:
        x_res, y_res, z_res = [int(p) for p in parts]
    except ValueError as exc:
        raise ValueError("grid values must be integers") from exc
    if x_res <= 0 or y_res <= 0 or z_res <= 0:
        raise ValueError("grid values must be positive integers")
    return x_res, y_res, z_res


def parse_bounds(s):
    if s is None:
        return None
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != 6:
        raise ValueError("bounds must be 6 comma-separated values: x_low,x_high,y_low,y_high,z_low,z_high")
    try:
        vals = [float(p) for p in parts]
    except ValueError as exc:
        raise ValueError("bounds values must be numbers") from exc
    return tuple(vals)

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


def find_manifest_for_map(root, map_path):
    map_path = Path(map_path)
    candidates = []
    candidates.append(map_path.with_name(map_path.stem + "_manifest.yaml"))
    match = re.match(r"^\d+_\d+_(.+)$", map_path.name)
    if match:
        base_name = match.group(1)
        base_stem = Path(base_name).stem
        candidates.append(map_path.with_name(base_stem + "_manifest.yaml"))

    for cand in candidates:
        if cand.exists():
            return cand

    map_dir = root / "Map"
    if map_dir.exists():
        for cand in map_dir.glob("*_manifest.yaml"):
            manifest = parse_simple_yaml(cand)
            base_map = manifest.get("base_map")
            subsample_maps = manifest.get("subsample_maps", {})
            if base_map and Path(str(base_map)).name == map_path.name:
                return cand
            for _, path in subsample_maps.items():
                if Path(str(path)).name == map_path.name:
                    return cand
    return None


def patch_grid_resolution(manifold_test_path, grid):
    if grid is None:
        return False
    x_res, y_res, z_res = grid
    text = manifold_test_path.read_text(encoding="utf-8")
    lines = text.splitlines()
    changed = False
    pattern = re.compile(
        r"(resolution\s*,\s*1\s*,\s*)(\d+)\s*,\s*(\d+)\s*,\s*(\d+)(\s*,\s*1\s*,\s*import_map)"
    )
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if "ExperimentManager" not in line or "import_map" not in line:
            continue
        if stripped.startswith("//"):
            continue
        def _repl(match):
            return f"{match.group(1)}{x_res}, {y_res}, {z_res}{match.group(5)}"
        new_line, n = pattern.subn(_repl, line, count=1)
        if n == 1 and new_line != line:
            lines[i] = new_line
            changed = True
        break
    if changed:
        manifold_test_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return changed


def patch_map_settings(manifold_test_path, map_name=None, map_output=None, bounds=None, mode="import"):
    if map_name is None and map_output is None and bounds is None:
        return False
    text = manifold_test_path.read_text(encoding="utf-8")
    lines = text.splitlines()
    changed = False
    in_import = False
    in_else = False

    for i, line in enumerate(lines):
        stripped = line.strip()
        if "if (import_map)" in line:
            in_import = True
            in_else = False
            continue
        if in_import and ("}else{" in stripped or "} else {" in stripped):
            in_import = False
            in_else = True
            continue
        if in_else and stripped == "}":
            in_else = False
            continue

        active = (mode == "import" and in_import) or (mode == "generate" and in_else)
        if not active:
            continue

        indent = line[: len(line) - len(line.lstrip())]

        if map_name is not None and "map_name =" in line:
            new_line = f'{indent}map_name = "{map_name}";'
            if new_line != line:
                lines[i] = new_line
                changed = True
            continue

        if map_output is not None and "map_output_filename =" in line:
            new_line = f'{indent}map_output_filename = "{map_output}";'
            if new_line != line:
                lines[i] = new_line
                changed = True
            continue

        if bounds is not None and "x_low =" in line and "x_high" in line:
            x_low, x_high, y_low, y_high, z_low, z_high = bounds
            new_line = f"{indent}x_low = {x_low}; x_high = {x_high};"
            if new_line != line:
                lines[i] = new_line
                changed = True
            continue

        if bounds is not None and "y_low =" in line and "y_high" in line:
            x_low, x_high, y_low, y_high, z_low, z_high = bounds
            new_line = f"{indent}y_low = {y_low}; y_high = {y_high};"
            if new_line != line:
                lines[i] = new_line
                changed = True
            continue

        if bounds is not None and "z_low =" in line and "z_high" in line:
            x_low, x_high, y_low, y_high, z_low, z_high = bounds
            new_line = f"{indent}z_low = {z_low}; z_high = {z_high};"
            if new_line != line:
                lines[i] = new_line
                changed = True
            continue

    if changed:
        manifold_test_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return changed


def main():
    parser = argparse.ArgumentParser(
        description="Run Monte Carlo grid experiment and organize outputs."
    )
    parser.add_argument(
        "--root",
        default=str(Path(__file__).resolve().parents[1]),
        help="Path to My_FoV_Optimization repo root.",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Build the C++ binaries before running.",
    )
    parser.add_argument(
        "--cmake",
        action="store_true",
        help="Force re-run cmake when building.",
    )
    parser.add_argument(
        "--levels",
        type=int,
        default=1,
        help="Deprecated (subsampling moved to map generator). Passed to manifold_test but ignored.",
    )
    parser.add_argument(
        "--grid",
        default=None,
        help="Grid resolution as X,Y,Z or XxYxZ (updates manifold_test.cpp).",
    )
    parser.add_argument(
        "--map",
        default=None,
        help="Map file to load (relative to Map/ or absolute). Overrides --import-map.",
    )
    parser.add_argument(
        "--map-manifest",
        default=None,
        help="YAML manifest produced by generate_cluster_map.py.",
    )
    parser.add_argument(
        "--level",
        type=int,
        default=None,
        help="Subsample level to use when --map-manifest is provided.",
    )
    parser.add_argument(
        "--generate-map",
        action="store_true",
        help="Generate a random cluster map (sets --import-map=0).",
    )
    parser.add_argument(
        "--map-out",
        default=None,
        help="Output map filename for generated map (e.g., clusters8_map.csv).",
    )
    parser.add_argument(
        "--bounds",
        default=None,
        help="Grid bounds x_low,x_high,y_low,y_high,z_low,z_high (updates manifold_test.cpp).",
    )
    parser.add_argument(
        "--import-map",
        type=int,
        choices=[0, 1],
        default=1,
        help="Pass 1 to load a map from disk, 0 to generate random map.",
    )
    parser.add_argument(
        "--clusters",
        type=int,
        default=8,
        help="Cluster count argument (ignored when --import-map=1).",
    )
    parser.add_argument(
        "--label",
        default="",
        help="Optional label appended to the output folder name.",
    )
    parser.add_argument(
        "--move",
        action="store_true",
        help="Move outputs into run folder instead of copying.",
    )
    parser.add_argument(
        "--visibility-angle",
        type=float,
        default=45.0,
        help="Visibility angle in degrees for plotting metrics.",
    )
    parser.add_argument(
        "--ks",
        type=float,
        default=15.0,
        help="Sigmoid sharpness for visibility plotting.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip plotting and metric generation.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show interactive 3D viewer after run.",
    )
    parser.add_argument(
        "--plot-sample",
        type=int,
        default=5000,
        help="Max map points to render in interactive viewer.",
    )

    args = parser.parse_args()

    if not args.show:
        import matplotlib
        matplotlib.use("Agg")
    root = Path(args.root).resolve()
    build_dir = root / "Manifold_cpp" / "build"
    bin_path = build_dir / "manifold_test"
    data_dir = root / "Data"
    manifold_test_path = root / "Manifold_cpp" / "manifold_test.cpp"

    data_dir.mkdir(parents=True, exist_ok=True)

    if args.map and args.map_manifest:
        print("Use only one of --map or --map-manifest.", file=sys.stderr)
        sys.exit(2)
    if args.map and args.generate_map:
        print("Use only one of --map or --generate-map.", file=sys.stderr)
        sys.exit(2)
    if args.generate_map or args.import_map == 0:
        print(
            "Map generation has been separated. Use scripts/generate_cluster_map.py to create a map,\n"
            "then rerun with --map <your_map.csv>.",
            file=sys.stderr,
        )
        sys.exit(2)

    map_name = None
    map_output = None
    map_path = None
    bounds = parse_bounds(args.bounds) if args.bounds else None

    if args.map_manifest:
        manifest_path = Path(args.map_manifest)
        if not manifest_path.is_absolute():
            manifest_path = root / args.map_manifest
        if not manifest_path.exists():
            print(f"Manifest not found: {manifest_path}", file=sys.stderr)
            sys.exit(2)
        manifest = parse_simple_yaml(manifest_path)
        base_map = manifest.get("base_map")
        subsample_maps = manifest.get("subsample_maps", {})
        level = args.level
        selected_map = None
        if level is not None:
            level_key = str(level)
            if level_key in subsample_maps:
                selected_map = subsample_maps[level_key]
            elif level in subsample_maps:
                selected_map = subsample_maps[level]
            else:
                print(
                    f"Level {level} not found in manifest. Available: {sorted(subsample_maps.keys())}",
                    file=sys.stderr,
                )
                sys.exit(2)
        else:
            selected_map = base_map

        if not selected_map:
            print("Manifest did not specify a base_map.", file=sys.stderr)
            sys.exit(2)
        map_path = Path(str(selected_map))
        if not map_path.is_absolute():
            map_path = root / map_path
        if not map_path.exists():
            print(f"Map not found: {map_path}", file=sys.stderr)
            sys.exit(2)
        args.import_map = 1
        if map_path.parent == (root / "Map"):
            map_name = f"../../Map/{map_path.name}"
        else:
            map_name = str(map_path)
        map_output = map_path.name

    if args.map:
        args.import_map = 1
        map_path = Path(args.map)
        if not map_path.is_absolute():
            map_path = root / "Map" / args.map
        if not map_path.exists():
            print(f"Map not found: {map_path}", file=sys.stderr)
            sys.exit(2)
        if args.level is not None:
            manifest_path = find_manifest_for_map(root, map_path)
            if manifest_path is None:
                print(
                    f"Level {args.level} requested but no manifest found for {map_path.name}.",
                    file=sys.stderr,
                )
                sys.exit(2)
            manifest = parse_simple_yaml(manifest_path)
            subsample_maps = manifest.get("subsample_maps", {})
            level_key = str(args.level)
            selected_map = subsample_maps.get(level_key) if isinstance(subsample_maps, dict) else None
            if not selected_map:
                print(
                    f"Level {args.level} not found in manifest {manifest_path.name}.",
                    file=sys.stderr,
                )
                sys.exit(2)
            map_path = Path(str(selected_map))
            if not map_path.is_absolute():
                map_path = root / map_path
            if not map_path.exists():
                print(f"Map not found: {map_path}", file=sys.stderr)
                sys.exit(2)
        if map_path.suffix.lower() == ".txt":
            print("Warning: .txt maps are space-separated; current loader expects CSV with commas.")
        # Use relative Map/ path when possible for portability.
        if map_path.parent == (root / "Map"):
            map_name = f"../../Map/{map_path.name}"
        else:
            map_name = str(map_path)
        map_output = map_path.name

    if args.generate_map:
        args.import_map = 0
        map_output = args.map_out or f"clusters{args.clusters}_map.csv"

    grid = None
    if args.grid is not None:
        grid = parse_grid(args.grid)
        changed = patch_grid_resolution(manifold_test_path, grid)
        if changed and not args.build:
            print("Grid change detected; enabling build.")
            args.build = True

    map_changed = False
    if map_name is not None or map_output is not None or bounds is not None:
        mode = "import" if args.import_map == 1 else "generate"
        map_changed = patch_map_settings(
            manifold_test_path,
            map_name=map_name if mode == "import" else None,
            map_output=map_output,
            bounds=bounds,
            mode=mode,
        )
        if map_changed and not args.build:
            print("Map settings changed; enabling build.")
            args.build = True

    if args.build:
        build_dir.mkdir(parents=True, exist_ok=True)
        if args.cmake or not (build_dir / "Makefile").exists():
            run_cmd(["cmake", ".."], cwd=str(build_dir))
        run_cmd(["make", "-j"], cwd=str(build_dir))

    if not bin_path.exists():
        print(f"Binary not found: {bin_path}", file=sys.stderr)
        print("Run with --build or build manually first.", file=sys.stderr)
        sys.exit(1)

    before = {p: p.stat().st_mtime for p in data_dir.glob("*") if p.is_file()}
    before_lines = {}
    for name in ["quiversforonepoint.csv"]:
        path = data_dir / name
        if path.exists():
            with path.open("r", encoding="utf-8") as f:
                before_lines[name] = sum(1 for _ in f)
        else:
            before_lines[name] = 0
    start_time = time.time()

    run_cmd(
        [
            str(bin_path),
            str(args.levels),
            str(args.import_map),
            str(args.clusters),
        ],
        cwd=str(build_dir),
    )

    # Collect outputs created or updated after run start.
    changed = []
    for p in data_dir.glob("*"):
        if not p.is_file():
            continue
        mtime = p.stat().st_mtime
        if mtime >= start_time or p not in before or mtime != before[p]:
            changed.append(p)

    ts = time.strftime("%Y%m%d_%H%M%S")
    label = f"_{args.label}" if args.label else ""
    run_dir = root / "Results" / "monte_carlo" / f"{ts}{label}"
    out_dir = run_dir / "data"
    out_dir.mkdir(parents=True, exist_ok=True)

    for p in changed:
        dest = out_dir / p.name
        if args.move:
            shutil.move(str(p), str(dest))
        else:
            shutil.copy2(str(p), str(dest))

    info_path = run_dir / "run_info.txt"
    with info_path.open("w", encoding="utf-8") as f:
        f.write("command: manifold_test\n")
        f.write(f"levels: {args.levels}\n")
        f.write(f"import_map: {args.import_map}\n")
        f.write(f"clusters: {args.clusters}\n")
        if args.map:
            f.write(f"map: {args.map}\n")
        if args.generate_map:
            f.write(f"map_out: {map_output}\n")
        if bounds is not None:
            f.write(f"bounds: {','.join(str(x) for x in bounds)}\n")
        if grid is not None:
            f.write(f"grid: {grid[0]},{grid[1]},{grid[2]}\n")
        f.write(f"binary: {bin_path}\n")
        f.write(f"data_dir: {data_dir}\n")
        f.write(f"output_dir: {out_dir}\n")
        f.write(f"visibility_angle_deg: {args.visibility_angle}\n")
        f.write(f"ks: {args.ks}\n")
        f.write(f"quiversforonepoint_prev_lines: {before_lines.get('quiversforonepoint.csv', 0)}\n")
        f.write("files:\n")
        for p in sorted(changed, key=lambda x: x.name):
            f.write(f"- {p.name}\n")

    if not args.no_plot:
        if map_path is None:
            print("Plotting skipped: map path not provided. Pass --map or --map-manifest.")
            print(f"Run outputs: {out_dir}")
            return

        prefix_files = sorted(out_dir.glob("*single_run_rotated_quivers.csv"))
        for opt_path in prefix_files:
            prefix = opt_path.name.replace("single_run_rotated_quivers.csv", "")
            bf_path = out_dir / f"{prefix}single_run_brute_force_rotated_quivers.csv"
            map_path_for_plot = map_path
            if not bf_path.exists():
                print(f"Missing brute-force file for prefix {prefix}; skipping plots.")
                continue

            opt_quivers = load_csv(opt_path, skiprows=1)
            bf_quivers = load_csv(bf_path, skiprows=1)

            map_points = load_csv(map_path_for_plot)

            log_path = out_dir / "quiversforonepoint.csv"
            log_rows = np.zeros((0, 0))
            if log_path.exists():
                log_rows = load_csv_tail(
                    log_path, before_lines.get("quiversforonepoint.csv", 0)
                )

            per_pose = compute_per_pose_metrics(
                map_points,
                opt_quivers,
                bf_quivers,
                args.visibility_angle,
                args.ks,
            )
            per_pose_csv = out_dir / f"{prefix}per_pose_metrics.csv"
            write_per_pose_metrics(per_pose_csv, per_pose)
            per_pose_png = out_dir / f"{prefix}per_pose_metrics.png"
            plot_per_pose_metrics(per_pose, per_pose_png)

            if log_rows.size != 0:
                metrics = compute_iteration_averages(
                    log_rows,
                    bf_quivers,
                    map_points,
                    args.visibility_angle,
                    args.ks,
                )
                if metrics is not None:
                    avgs, counts = metrics
                    metrics_csv = out_dir / f"{prefix}avg_metrics.csv"
                    write_avg_metrics(metrics_csv, avgs, counts)
                    metrics_png = out_dir / f"{prefix}avg_metrics.png"
                    plot_avg_metrics(avgs, metrics_png)

            if args.show:
                interactive_viewer(
                    map_points,
                    log_rows,
                    bf_quivers,
                    opt_quivers,
                    out_dir,
                    sample_points=args.plot_sample,
                )

    print(f"Run outputs: {out_dir}")


if __name__ == "__main__":
    main()
