#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc


def _detect_skiprows(path):
    with path.open("r", encoding="utf-8") as f:
        first = f.readline().strip()
    if not first:
        return 0
    if first.startswith("#"):
        return 1
    for ch in first:
        if ch.isalpha():
            return 1
    return 0


def load_csv(path, name):
    if not path.exists():
        raise SystemExit(f"Missing {name}: {path}")
    skip = _detect_skiprows(path)
    return np.loadtxt(str(path), delimiter=",", skiprows=skip)


def fov_plot(points, opt_quivers, bf_quivers, onepointlog, accfile, bounds=None):
    fig, axes = plt.subplots(1, 4, figsize=(15, 5))
    opt_step = onepointlog[:, 0]
    unique_ids = np.unique(opt_step)

    feature_cnt = onepointlog[:, 7]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[3].plot(np.where(mask)[0], feature_cnt[mask], marker="s", label=f"ID {int(id_)}")
    axes[3].set_title("feature count")
    axes[3].set_xlabel("step")
    axes[3].set_ylabel("feature count")

    degree_between = onepointlog[:, 8]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[0].plot(np.where(mask)[0], degree_between[mask], marker="o", label=f"ID {int(id_)}")
    axes[0].set_title("Degree Difference Trend")
    axes[0].set_xlabel("step")
    axes[0].set_ylabel("Degree Between")

    j_norm = onepointlog[:, 9]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[1].plot(np.where(mask)[0], j_norm[mask], marker="s", label=f"ID {int(id_)}")
    axes[1].set_title("visibility jacobian norm Trend")
    axes[1].set_xlabel("step")
    axes[1].set_ylabel("aJl norm")

    vis = onepointlog[:, 11]
    vis_bf = accfile[:, 1]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[2].plot(np.where(mask)[0], vis[mask], marker="s", label=f"ID {int(id_)}")
        last_index = np.where(mask)[0][-1]
        axes[2].plot(
            np.where(mask)[0],
            np.full(np.sum(mask), vis_bf[int(id_)], dtype=float),
            color="black",
            label=f"Max Vis ID {id_}",
        )

    axes[2].set_title("visibility")
    axes[2].set_xlabel("step")
    axes[2].set_ylabel("visibility")

    # Quiver plot
    scale_factoropt = 0.5
    scale_factorbfv = 0.4
    arrow = 0.5

    fig_quiver = plt.figure()
    ax = fig_quiver.add_subplot(111, projection="3d")

    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="blue", alpha=0.05, label="Map Points")

    points_a = opt_quivers[:, :3]
    points_b = opt_quivers[:, 3:]
    for a, b in zip(points_a, points_b):
        ax.quiver(
            a[0],
            a[1],
            a[2],
            b[0] * scale_factoropt,
            b[1] * scale_factoropt,
            b[2] * scale_factoropt,
            color="orange",
            arrow_length_ratio=arrow,
        )

    twc = bf_quivers[:, :3]
    bf_vis = bf_quivers[:, 6:]
    for a, b in zip(twc, bf_vis):
        ax.quiver(
            a[0],
            a[1],
            a[2],
            b[0] * scale_factorbfv,
            b[1] * scale_factorbfv,
            b[2] * scale_factorbfv,
            alpha=0.5,
            color="blue",
            arrow_length_ratio=arrow,
        )

    ax.set_title("FOV quivers")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    if bounds is not None:
        ax.set_xlim([bounds[0], bounds[1]])
        ax.set_ylim([bounds[2], bounds[3]])
        ax.set_zlim([bounds[4], bounds[5]])
        ax.set_box_aspect(
            [bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]]
        )

    ax.legend()

    return fig, fig_quiver


def plot_error(acc_data):
    degree_between = acc_data[:, 0]
    brute_force_vis = acc_data[:, 1]
    optimized_vis = acc_data[:, 2]

    fig_scatter = plt.figure(figsize=(8, 6))
    plt.scatter(
        brute_force_vis,
        optimized_vis,
        c=degree_between,
        cmap="coolwarm",
        edgecolors="black",
        alpha=0.7,
    )
    plt.plot(
        [min(brute_force_vis), max(brute_force_vis)],
        [min(brute_force_vis), max(brute_force_vis)],
        "k--",
        label="Perfect Match",
    )
    plt.colorbar(label="Degree Difference")
    plt.xlabel("Brute Force Max Visibility")
    plt.ylabel("Optimized Max Visibility")
    plt.title("Brute Force vs. Optimized Visibility")
    plt.legend()
    plt.grid()

    visibility_diff = np.abs(brute_force_vis - optimized_vis)
    fig_bar = plt.figure(figsize=(10, 5))
    plt.bar(range(len(visibility_diff)), visibility_diff, color="orange", alpha=0.7)
    plt.xlabel("Sample Index")
    plt.ylabel("Visibility Difference")
    plt.title("Difference Between Brute Force and Optimized Visibility")
    plt.grid(axis="y", linestyle="--")

    return fig_scatter, fig_bar


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


def compute_optimizer_accuracy(points, opt_quivers, bf_quivers, visibility_angle_deg, ks):
    n = opt_quivers.shape[0]
    results = np.zeros((n, 4), dtype=float)
    for i in range(n):
        ref = opt_quivers[i, 0:3]
        opt_dir = opt_quivers[i, 3:6]
        bf_vis = bf_quivers[i, 6:9]
        opt_dir = opt_dir / np.linalg.norm(opt_dir)
        bf_vis = bf_vis / np.linalg.norm(bf_vis)
        degree_between = np.degrees(np.arccos(np.clip(np.dot(bf_vis, opt_dir), -1.0, 1.0)))
        bf_visibility = compute_visibility(points, ref, bf_vis, visibility_angle_deg, ks)
        opt_visibility = compute_visibility(points, ref, opt_dir, visibility_angle_deg, ks)
        visibility_diff = bf_visibility - opt_visibility
        results[i, 0] = degree_between
        results[i, 1] = bf_visibility
        results[i, 2] = opt_visibility
        results[i, 3] = visibility_diff
    return results


def write_accuracy(path, acc_data):
    with path.open("w", encoding="utf-8") as f:
        f.write("degree_between,brute_force_max_visibility,optimized_max_visibility,visibility_diff\n")
        for row in acc_data:
            f.write(",".join(f"{v:.6f}" for v in row) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Plot Monte Carlo experiment results.")
    parser.add_argument("--prefix", default="0_1_", help="Data file prefix (e.g., 0_1_).")
    parser.add_argument(
        "--map",
        default="two_walls_points_w.csv",
        help="Map file name in Map/ (e.g., two_walls_points_w.csv).",
    )
    parser.add_argument("--data-dir", default=None, help="Override Data/ directory.")
    parser.add_argument("--map-dir", default=None, help="Override Map/ directory.")
    parser.add_argument("--which-twc", type=int, default=None, help="Filter onepointlog by ID.")
    parser.add_argument("--no-tex", action="store_true", help="Disable LaTeX text rendering.")
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Directory to save plots (default: Results/monte_carlo_plots/<timestamp>).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plots interactively instead of only saving.",
    )
    parser.add_argument(
        "--visibility-angle",
        type=float,
        default=45.0,
        help="Visibility angle (deg) used for accuracy computation.",
    )
    parser.add_argument(
        "--ks",
        type=float,
        default=15.0,
        help="Sigmoid sharpness (ks) used for visibility computation.",
    )
    parser.add_argument(
        "--write-accuracy",
        action="store_true",
        help="Recompute and write optimizer_accuracy_file.csv.",
    )
    parser.add_argument(
        "--bounds",
        default="-2.5,2.5,-2.5,2.5,-1,1",
        help="Axis bounds xmin,xmax,ymin,ymax,zmin,zmax.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    data_dir = Path(args.data_dir).resolve() if args.data_dir else root / "Data"
    map_dir = Path(args.map_dir).resolve() if args.map_dir else root / "Map"

    if not args.no_tex:
        rc("text", usetex=True)
    rc("font", **{"family": "serif", "serif": ["Cardo"], "size": 10})

    points = load_csv(map_dir / args.map, "map points")
    opt_quivers = load_csv(data_dir / f"{args.prefix}single_run_rotated_quivers.csv", "optimized quivers")
    bf_quivers = load_csv(
        data_dir / f"{args.prefix}single_run_brute_force_rotated_quivers.csv",
        "brute force quivers",
    )
    acc_path = data_dir / f"{args.prefix}optimizer_accuracy_file.csv"
    if args.write_accuracy or not acc_path.exists():
        acc_file = compute_optimizer_accuracy(
            points, opt_quivers, bf_quivers, args.visibility_angle, args.ks
        )
        write_accuracy(acc_path, acc_file)
    else:
        acc_file = load_csv(acc_path, "accuracy file")
    onepointlog = load_csv(data_dir / "quiversforonepoint.csv", "quiversforonepoint")

    if args.which_twc is not None:
        onepointlog = onepointlog[onepointlog[:, 0] == args.which_twc]

    bounds = [float(x) for x in args.bounds.split(",")] if args.bounds else None

    fig_trends, fig_quiver = fov_plot(points, opt_quivers, bf_quivers, onepointlog, acc_file, bounds=bounds)
    fig_scatter, fig_bar = plot_error(acc_file)

    ts = time.strftime("%Y%m%d_%H%M%S")
    if args.out_dir:
        out_dir = Path(args.out_dir).resolve()
    else:
        out_dir = root / "Results" / "monte_carlo_plots" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    fig_trends.savefig(out_dir / f"{args.prefix}trends.png", dpi=200, bbox_inches="tight")
    fig_quiver.savefig(out_dir / f"{args.prefix}quivers.png", dpi=200, bbox_inches="tight")
    fig_scatter.savefig(out_dir / f"{args.prefix}bf_vs_opt.png", dpi=200, bbox_inches="tight")
    fig_bar.savefig(out_dir / f"{args.prefix}visibility_diff.png", dpi=200, bbox_inches="tight")

    if args.show:
        plt.show()
    else:
        plt.close("all")

    print(f"Saved plots to: {out_dir}")


if __name__ == "__main__":
    main()
