#!/usr/bin/env python3
import argparse
import math
import time
from pathlib import Path

import numpy as np


def parse_bounds(s):
    if isinstance(s, (list, tuple)):
        if len(s) == 1:
            s = s[0]
        else:
            parts = [str(p).strip() for p in s if str(p).strip()]
            if len(parts) != 6:
                raise ValueError("bounds must be 6 values: x_low x_high y_low y_high z_low z_high")
            return tuple(float(p) for p in parts)
    parts = [p.strip() for p in str(s).split(",") if p.strip()]
    if len(parts) != 6:
        raise ValueError("bounds must be 6 comma-separated values: x_low,x_high,y_low,y_high,z_low,z_high")
    return tuple(float(p) for p in parts)


def parse_range(s, name):
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != 2:
        raise ValueError(f"{name} must be 'min,max'")
    low, high = (float(p) for p in parts)
    if low > high:
        raise ValueError(f"{name} min must be <= max")
    return low, high


def parse_features(s, clusters):
    if s is None:
        return [1000] * clusters
    parts = [p.strip() for p in s.split(",") if p.strip()]
    values = [int(p) for p in parts]
    if len(values) == 1 and clusters is not None:
        return values * clusters
    if clusters is None:
        return values
    if len(values) != clusters:
        raise ValueError("features-per-cluster count must match --clusters")
    return values


def parse_resolution(s):
    parts = [p.strip() for p in str(s).split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError("pose-resolution must be 'nx,ny,nz'")
    nx, ny, nz = (int(p) for p in parts)
    if nx <= 0 or ny <= 0 or nz <= 0:
        raise ValueError("pose-resolution values must be positive integers")
    return nx, ny, nz


def generate_cluster_points(rng, center, std, bounds, count):
    x_low, x_high, y_low, y_high, z_low, z_high = bounds
    collected = []
    collected_count = 0
    while collected_count < count:
        remaining = count - collected_count
        batch = max(256, int(remaining * 1.5))
        samples = rng.normal(loc=center, scale=std, size=(batch, 3))
        mask = (
            (samples[:, 0] >= x_low)
            & (samples[:, 0] <= x_high)
            & (samples[:, 1] >= y_low)
            & (samples[:, 1] <= y_high)
            & (samples[:, 2] >= z_low)
            & (samples[:, 2] <= z_high)
        )
        valid = samples[mask]
        if valid.size == 0:
            continue
        take = min(remaining, valid.shape[0])
        collected.append(valid[:take])
        collected_count += take
    return np.vstack(collected)


def main():
    parser = argparse.ArgumentParser(
        description="Generate a clustered Monte Carlo map, save it, and visualize it."
    )
    parser.add_argument(
        "--root",
        default=str(Path(__file__).resolve().parents[1]),
        help="Path to My_FoV_Optimization repo root.",
    )
    parser.add_argument(
        "--clusters",
        type=int,
        default=None,
        help="Number of clusters to generate (default: 8, or inferred from --features-per-cluster).",
    )
    parser.add_argument(
        "--features-per-cluster",
        default=None,
        help="Comma-separated list or single value (e.g., 1000 or 500,800,1200).",
    )
    parser.add_argument(
        "--bounds",
        nargs="+",
        default=["-250,250,-250,250,0,10"],
        help="x_low,x_high,y_low,y_high,z_low,z_high",
    )
    parser.add_argument(
        "--pose-count",
        type=int,
        default=0,
        help="Number of poses to sample and save (0 disables).",
    )
    parser.add_argument(
        "--pose-sample",
        choices=["grid", "random"],
        default="grid",
        help="Pose sampling strategy.",
    )
    parser.add_argument(
        "--pose-bounds",
        nargs="+",
        default=None,
        help="Pose bounds x_low,x_high,y_low,y_high,z_low,z_high (default: map bounds).",
    )
    parser.add_argument(
        "--pose-out",
        default=None,
        help="Pose CSV output (default: Map/<map_stem>_poses.csv).",
    )
    parser.add_argument(
        "--pose-resolution",
        default=None,
        help="Grid resolution as nx,ny,nz (overrides --pose-count when provided).",
    )
    parser.add_argument(
        "--pose-min-dist",
        type=float,
        default=None,
        help="Minimum distance between poses for random sampling (default: auto from bounds and count; 0 disables).",
    )
    parser.add_argument(
        "--std-range",
        default="15,30",
        help="Stddev range for clusters as min,max.",
    )
    parser.add_argument(
        "--dir-range",
        default="-30,30",
        help="Direction range for cluster directions (x,y) as min,max.",
    )
    parser.add_argument(
        "--uncertainty-range",
        default="1,80",
        help="Uncertainty range for clusters as min,max.",
    )
    parser.add_argument(
        "--far-cluster-fraction",
        type=float,
        default=0.25,
        help="Fraction of clusters forced away from map center (0 disables).",
    )
    parser.add_argument(
        "--far-cluster-count",
        type=int,
        default=None,
        help="Exact number of far clusters (overrides --far-cluster-fraction).",
    )
    parser.add_argument(
        "--far-radius-frac",
        type=float,
        default=0.6,
        help="Minimum radius as a fraction of half-diagonal for far clusters.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Name for map folder and file stem (Map/<name>/<name>_map.csv).",
    )
    parser.add_argument(
        "--map-out",
        default=None,
        help="Output map filename (overrides --name).",
    )
    parser.add_argument(
        "--subsample-levels",
        type=int,
        default=0,
        help="Write legacy subsampled maps with levels 1..N (uses 250/2500 scheme).",
    )
    parser.add_argument(
        "--subsample-prefix",
        default="0",
        help="Prefix for subsampled map filenames (default: 0 -> 0_1_<map>.csv).",
    )
    parser.add_argument(
        "--shuffle-before-subsample",
        action="store_true",
        help="Shuffle points before subsampling to reduce ordering bias.",
    )
    parser.add_argument(
        "--write-dir",
        action="store_true",
        help="Also write a *_dir.csv file (dir_x,dir_y,dir_z,uncertainty).",
    )
    parser.add_argument(
        "--plot-out",
        default=None,
        help="Output plot filename (default: Map/<map_folder>/<map_stem>_preview.png).",
    )
    parser.add_argument(
        "--manifest-out",
        default=None,
        help="Output manifest YAML (default: Map/<map_stem>_manifest.yaml).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plot interactively (requires GUI backend).",
    )

    args = parser.parse_args()
    root = Path(args.root).resolve()
    map_dir = root / "Map"
    map_dir.mkdir(parents=True, exist_ok=True)

    bounds = parse_bounds(args.bounds)
    pose_bounds = parse_bounds(args.pose_bounds) if args.pose_bounds else bounds
    std_min, std_max = parse_range(args.std_range, "std-range")
    dir_min, dir_max = parse_range(args.dir_range, "dir-range")
    unc_min, unc_max = parse_range(args.uncertainty_range, "uncertainty-range")

    features_list = None
    if args.features_per_cluster is not None:
        features_list = parse_features(args.features_per_cluster, None)

    if args.clusters is None:
        args.clusters = len(features_list) if features_list is not None else 8

    features_per_cluster = parse_features(args.features_per_cluster, args.clusters)

    if args.name and args.map_out:
        raise SystemExit("Use only one of --name or --map-out.")

    ts = time.strftime("%Y%m%d_%H%M%S")
    if args.map_out:
        map_path = Path(args.map_out)
        if not map_path.is_absolute():
            map_path = map_dir / map_path
        if map_path.exists():
            raise SystemExit(
                f"Map already exists: {map_path}. Use a different --map-out or remove it."
            )
    elif args.name:
        run_dir = map_dir / args.name
        if run_dir.exists() and any(run_dir.iterdir()):
            raise SystemExit(
                f"Map folder already exists and is not empty: {run_dir}. "
                "Choose a new --name."
            )
        run_dir.mkdir(parents=True, exist_ok=True)
        map_path = run_dir / f"{args.name}_map.csv"
    else:
        run_dir = map_dir / f"{ts}_clusters{args.clusters}"
        run_dir.mkdir(parents=True, exist_ok=True)
        map_path = run_dir / f"clusters{args.clusters}_map.csv"
    map_path.parent.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)
    low = np.array([bounds[0], bounds[2], bounds[4]], dtype=float)
    high = np.array([bounds[1], bounds[3], bounds[5]], dtype=float)
    center = (low + high) * 0.5
    half = (high - low) * 0.5
    max_radius = float(np.linalg.norm(half))
    if args.far_radius_frac < 0.0:
        raise SystemExit("--far-radius-frac must be >= 0.")
    min_radius = args.far_radius_frac * max_radius

    if args.far_cluster_count is not None:
        far_count = max(0, int(args.far_cluster_count))
    elif args.far_cluster_fraction > 0.0:
        far_count = int(math.ceil(args.clusters * args.far_cluster_fraction))
    else:
        far_count = 0
    far_count = min(far_count, args.clusters)

    far_indices = set()
    if far_count > 0 and min_radius > 0.0:
        far_indices = set(rng.choice(args.clusters, size=far_count, replace=False).tolist())

    centers = np.zeros((args.clusters, 3), dtype=float)
    for idx in range(args.clusters):
        if idx in far_indices and min_radius > 0.0:
            placed = False
            for _ in range(10000):
                candidate = rng.uniform(low=low, high=high, size=3)
                if np.linalg.norm(candidate - center) >= min_radius:
                    centers[idx] = candidate
                    placed = True
                    break
            if not placed:
                print(
                    "Warning: could not place far cluster after many attempts; "
                    "falling back to unconstrained sampling."
                )
                centers[idx] = rng.uniform(low=low, high=high, size=3)
        else:
            centers[idx] = rng.uniform(low=low, high=high, size=3)

    points_list = []
    labels = []
    dir_rows = []

    for idx in range(args.clusters):
        std = rng.uniform(std_min, std_max, size=3)
        direction = np.array(
            [rng.uniform(dir_min, dir_max), rng.uniform(dir_min, dir_max), 0.0],
            dtype=float,
        )
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            direction = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            direction = direction / norm
        uncertainty = rng.uniform(unc_min, unc_max)

        cluster_points = generate_cluster_points(
            rng, centers[idx], std, bounds, features_per_cluster[idx]
        )
        points_list.append(cluster_points)
        labels.append(np.full(cluster_points.shape[0], idx, dtype=int))

        if args.write_dir:
            dir_block = np.tile(
                np.array([direction[0], direction[1], direction[2], uncertainty]),
                (cluster_points.shape[0], 1),
            )
            dir_rows.append(dir_block)

    points = np.vstack(points_list)
    labels = np.concatenate(labels)

    np.savetxt(map_path, points, delimiter=",", fmt="%.6f")

    dir_path = None
    if args.write_dir:
        dir_path = map_path.with_name(map_path.stem + "_dir.csv")
        dir_data = np.vstack(dir_rows)
        np.savetxt(dir_path, dir_data, delimiter=",", fmt="%.6f")

    subsample_paths = {}
    if args.subsample_levels > 0:
        ordered_points = points
        ordered_dir = dir_data if args.write_dir else None
        if args.shuffle_before_subsample:
            perm = rng.permutation(points.shape[0])
            ordered_points = points[perm]
            if ordered_dir is not None:
                ordered_dir = ordered_dir[perm]
        indices = np.arange(ordered_points.shape[0])
        for level in range(1, args.subsample_levels + 1):
            threshold = level * 250
            mask = (indices % 2500) <= threshold
            subsampled = ordered_points[mask]
            subsample_name = f"{args.subsample_prefix}_{level}_{map_path.name}"
            subsample_path = map_path.with_name(subsample_name)
            np.savetxt(subsample_path, subsampled, delimiter=",", fmt="%.6f")
            if ordered_dir is not None:
                subsample_dir_path = subsample_path.with_name(subsample_path.stem + "_dir.csv")
                np.savetxt(subsample_dir_path, ordered_dir[mask], delimiter=",", fmt="%.6f")
            print(f"Subsample level {level}: {subsample_path} ({subsampled.shape[0]} points)")
            subsample_paths[level] = subsample_path

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    scatter = ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        c=labels,
        cmap="tab10",
        s=2,
        alpha=0.7,
    )
    ax.scatter(
        centers[:, 0],
        centers[:, 1],
        centers[:, 2],
        c="black",
        s=30,
        marker="x",
        label="cluster centers",
    )
    ax.set_title("Generated Cluster Map")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([bounds[0], bounds[1]])
    ax.set_ylim([bounds[2], bounds[3]])
    ax.set_zlim([bounds[4], bounds[5]])
    ax.set_box_aspect(
        [bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]]
    )
    ax.legend(loc="upper right")

    if args.plot_out:
        plot_path = Path(args.plot_out)
        if not plot_path.is_absolute():
            plot_path = root / args.plot_out
    else:
        plot_path = map_path.with_name(f"{map_path.stem}_preview.png")
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)

    if args.show:
        plt.show()

    pose_path = None
    pose_min_dist = 0.0
    pose_grid = None
    if args.pose_count > 0 or args.pose_resolution:
        pose_path = Path(args.pose_out) if args.pose_out else map_path.with_name(map_path.stem + "_poses.csv")
        if not pose_path.is_absolute():
            pose_path = map_dir / pose_path
        low = np.array([pose_bounds[0], pose_bounds[2], pose_bounds[4]], dtype=float)
        high = np.array([pose_bounds[1], pose_bounds[3], pose_bounds[5]], dtype=float)

        effective_count = args.pose_count
        if args.pose_resolution:
            pose_grid = parse_resolution(args.pose_resolution)
            effective_count = pose_grid[0] * pose_grid[1] * pose_grid[2]
            if args.pose_count > 0 and effective_count != args.pose_count:
                print("Note: --pose-resolution overrides --pose-count.")

        dx = max(high[0] - low[0], 1e-6)
        dy = max(high[1] - low[1], 1e-6)
        dz = max(high[2] - low[2], 1e-6)
        volume = dx * dy * dz
        auto_spacing = (volume / max(effective_count, 1)) ** (1.0 / 3.0)

        if args.pose_sample == "grid":
            if pose_grid is None:
                rx, ry, rz = dx, dy, dz
                base = (max(effective_count, 1) / (rx * ry * rz)) ** (1.0 / 3.0)
                cx = max(1, int(round(rx * base)))
                cy = max(1, int(round(ry * base)))
                cz = max(1, int(round(rz * base)))
                best = None
                for nx in range(max(1, cx - 3), cx + 4):
                    for ny in range(max(1, cy - 3), cy + 4):
                        for nz in range(max(1, cz - 3), cz + 4):
                            prod = nx * ny * nz
                            if prod == 0:
                                continue
                            diff = abs(prod - args.pose_count)
                            ratio_err = abs(nx / ny - rx / ry) + abs(nx / nz - rx / rz)
                            score = (diff, ratio_err)
                            if best is None or score < best[0]:
                                best = (score, (nx, ny, nz))
                nx, ny, nz = best[1] if best else (cx, cy, cz)
            else:
                nx, ny, nz = pose_grid

            xs = np.array([(low[0] + high[0]) * 0.5]) if nx == 1 else np.linspace(low[0], high[0], nx)
            ys = np.array([(low[1] + high[1]) * 0.5]) if ny == 1 else np.linspace(low[1], high[1], ny)
            zs = np.array([(low[2] + high[2]) * 0.5]) if nz == 1 else np.linspace(low[2], high[2], nz)
            grid = np.meshgrid(xs, ys, zs, indexing="xy")
            poses = np.column_stack([g.reshape(-1) for g in grid])
            if args.pose_count > 0 and poses.shape[0] != args.pose_count:
                print(
                    f"Note: grid produced {poses.shape[0]} poses (requested {args.pose_count}). "
                    "Adjust --pose-count or pass --pose-resolution."
                )
        else:
            if args.pose_count <= 0:
                raise SystemExit("Random pose sampling requires --pose-count > 0.")
            if args.pose_min_dist is None:
                pose_min_dist = 0.6 * auto_spacing
            else:
                pose_min_dist = float(args.pose_min_dist)

            if pose_min_dist <= 0.0:
                poses = rng.uniform(low=low, high=high, size=(args.pose_count, 3))
            else:
                min_dist_sq = pose_min_dist * pose_min_dist
                poses = []
                max_tries = max(1000, args.pose_count * 2000)
                tries = 0
                while len(poses) < args.pose_count and tries < max_tries:
                    candidate = rng.uniform(low=low, high=high, size=3)
                    if not poses:
                        poses.append(candidate)
                    else:
                        deltas = np.asarray(poses) - candidate
                        if np.min(np.einsum("ij,ij->i", deltas, deltas)) >= min_dist_sq:
                            poses.append(candidate)
                    tries += 1
                if len(poses) < args.pose_count:
                    print(
                        "Warning: only placed "
                        f"{len(poses)}/{args.pose_count} poses with min_dist={pose_min_dist:.3f}. "
                        "Reduce --pose-min-dist or --pose-count if needed."
                    )
                poses = np.asarray(poses, dtype=float).reshape(-1, 3)
        np.savetxt(pose_path, poses, delimiter=",", fmt="%.6f")

    manifest_path = None
    if args.manifest_out:
        manifest_path = Path(args.manifest_out)
        if not manifest_path.is_absolute():
            manifest_path = root / args.manifest_out
    else:
        manifest_path = map_path.with_name(map_path.stem + "_manifest.yaml")

    def rel_path(path):
        try:
            return str(path.relative_to(root))
        except ValueError:
            return str(path)

    with manifest_path.open("w", encoding="utf-8") as f:
        f.write("version: 1\n")
        f.write(f"base_map: {rel_path(map_path)}\n")
        f.write(f"bounds: [{bounds[0]}, {bounds[1]}, {bounds[2]}, {bounds[3]}, {bounds[4]}, {bounds[5]}]\n")
        f.write(f"pose_bounds: [{pose_bounds[0]}, {pose_bounds[1]}, {pose_bounds[2]}, {pose_bounds[3]}, {pose_bounds[4]}, {pose_bounds[5]}]\n")
        f.write(f"pose_count: {poses.shape[0] if pose_path else 0}\n")
        f.write(f"pose_sample: {args.pose_sample}\n")
        if pose_grid is not None:
            f.write(f"pose_grid: [{pose_grid[0]}, {pose_grid[1]}, {pose_grid[2]}]\n")
        f.write(f"pose_min_dist: {pose_min_dist}\n")
        f.write(f"seed: {args.seed}\n")
        f.write(f"clusters: {args.clusters}\n")
        f.write("features_per_cluster: [" + ", ".join(str(v) for v in features_per_cluster) + "]\n")
        f.write(f"std_range: [{std_min}, {std_max}]\n")
        f.write(f"dir_range: [{dir_min}, {dir_max}]\n")
        f.write(f"uncertainty_range: [{unc_min}, {unc_max}]\n")
        f.write(f"far_cluster_fraction: {args.far_cluster_fraction}\n")
        if args.far_cluster_count is not None:
            f.write(f"far_cluster_count: {args.far_cluster_count}\n")
        f.write(f"far_radius_frac: {args.far_radius_frac}\n")
        f.write(f"subsample_levels: {args.subsample_levels}\n")
        f.write(f"subsample_prefix: {args.subsample_prefix}\n")
        f.write(f"subsample_shuffle: {str(args.shuffle_before_subsample).lower()}\n")
        f.write(f"plot: {rel_path(plot_path)}\n")
        if subsample_paths:
            f.write("subsample_maps:\n")
            for level, path in sorted(subsample_paths.items()):
                f.write(f"  {level}: {rel_path(path)}\n")
        if dir_path:
            f.write(f"dir_map: {rel_path(dir_path)}\n")
        if pose_path:
            f.write(f"pose_map: {rel_path(pose_path)}\n")

    print(f"Map saved: {map_path}")
    if dir_path:
        print(f"Dir map saved: {dir_path}")
    print(f"Plot saved: {plot_path}")
    print(f"Manifest saved: {manifest_path}")
    if pose_path:
        print(f"Poses saved: {pose_path}")

    rel_manifest = rel_path(manifest_path)
    print("\nDownstream (from repo root My_FoV_Optimization/):")
    print(
        "  Monte Carlo (one folder per subsample level under Results/.../data/level_<k>/) — "
        "example:"
    )
    print(f"    python scripts/run_monte_carlo_experiment.py --map-manifest {rel_manifest} --all-levels")
    print(
        "  Plots — run-level comparison_visibility.png plus per-level plots/level_<k>/comparison_visibility.png:"
    )
    print("    python scripts/plot_monte_carlo_results.py --run-dir Results/monte_carlo/<timestamp>_<label>/")


if __name__ == "__main__":
    main()
