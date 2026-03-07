#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import numpy as np


def parse_bounds(s):
    parts = [p.strip() for p in s.split(",") if p.strip()]
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
        default="-250,250,-250,250,0,10",
        help="x_low,x_high,y_low,y_high,z_low,z_high",
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
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--map-out",
        default=None,
        help="Output map filename (default: clusters<N>_map.csv in Map/).",
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
        help="Output plot filename (default: Results/map_preview/<timestamp>_<map>.png).",
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
    std_min, std_max = parse_range(args.std_range, "std-range")
    dir_min, dir_max = parse_range(args.dir_range, "dir-range")
    unc_min, unc_max = parse_range(args.uncertainty_range, "uncertainty-range")

    features_list = None
    if args.features_per_cluster is not None:
        features_list = parse_features(args.features_per_cluster, None)

    if args.clusters is None:
        args.clusters = len(features_list) if features_list is not None else 8

    features_per_cluster = parse_features(args.features_per_cluster, args.clusters)

    map_name = args.map_out or f"clusters{args.clusters}_map.csv"
    map_path = Path(map_name)
    if not map_path.is_absolute():
        map_path = map_dir / map_name

    rng = np.random.default_rng(args.seed)
    centers = rng.uniform(
        low=[bounds[0], bounds[2], bounds[4]],
        high=[bounds[1], bounds[3], bounds[5]],
        size=(args.clusters, 3),
    )

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
        ts = time.strftime("%Y%m%d_%H%M%S")
        plot_dir = root / "Results" / "map_preview"
        plot_dir.mkdir(parents=True, exist_ok=True)
        plot_path = plot_dir / f"{ts}_{map_path.stem}.png"
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)

    if args.show:
        plt.show()

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
        f.write(f"seed: {args.seed}\n")
        f.write(f"clusters: {args.clusters}\n")
        f.write("features_per_cluster: [" + ", ".join(str(v) for v in features_per_cluster) + "]\n")
        f.write(f"std_range: [{std_min}, {std_max}]\n")
        f.write(f"dir_range: [{dir_min}, {dir_max}]\n")
        f.write(f"uncertainty_range: [{unc_min}, {unc_max}]\n")
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

    print(f"Map saved: {map_path}")
    if dir_path:
        print(f"Dir map saved: {dir_path}")
    print(f"Plot saved: {plot_path}")
    print(f"Manifest saved: {manifest_path}")


if __name__ == "__main__":
    main()
