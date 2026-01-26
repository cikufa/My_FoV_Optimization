#!/usr/bin/env python3
"""
Visualize trajectory optimization results.

Quiver input format: each line has x,y,z,dx,dy,dz (comma or whitespace separated).
Stamped UE pose format: index x y z pitch yaw roll (or x y z pitch yaw roll).
Files can contain multiple blocks separated by blank lines.
"""

import argparse
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from wandb_utils import add_wandb_args, safe_init, log_metrics, finish


def parse_quiver_blocks(path: Path):
    blocks = []
    current = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                if current:
                    blocks.append(np.array(current, dtype=float))
                    current = []
                continue
            parts = [p for p in re.split(r"[,\s]+", line) if p]
            if len(parts) < 6:
                continue
            current.append([float(v) for v in parts[:6]])
    if current:
        blocks.append(np.array(current, dtype=float))
    if not blocks:
        raise ValueError(f"No quiver data found in {path}")
    return blocks


def parse_stamped_ue_blocks(path: Path):
    blocks = []
    current = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                if current:
                    blocks.append(np.array(current, dtype=float))
                    current = []
                continue
            parts = [p for p in re.split(r"[,\s]+", line) if p]
            if len(parts) == 6:
                vals = parts
            elif len(parts) >= 7:
                vals = parts[1:7]
            else:
                continue
            current.append([float(v) for v in vals])
    if current:
        blocks.append(np.array(current, dtype=float))
    if not blocks:
        raise ValueError(f"No stamped UE pose data found in {path}")
    return blocks


def select_block(blocks, selector: str, label: str):
    if selector == "first":
        return blocks[0]
    if selector == "last":
        return blocks[-1]
    try:
        idx = int(selector)
    except ValueError as exc:
        raise ValueError(f"Invalid {label} block selector: {selector}") from exc
    if idx < 0 or idx >= len(blocks):
        raise IndexError(
            f"{label} block index {idx} out of range (0..{len(blocks) - 1})"
        )
    return blocks[idx]


def set_axes_equal(ax, xyz):
    if xyz.size == 0:
        return
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    centers = (mins + maxs) / 2.0
    max_range = (maxs - mins).max()
    if max_range == 0:
        max_range = 1.0
    half = max_range / 2.0
    ax.set_xlim(centers[0] - half, centers[0] + half)
    ax.set_ylim(centers[1] - half, centers[1] + half)
    ax.set_zlim(centers[2] - half, centers[2] + half)
    try:
        ax.set_box_aspect([1, 1, 1])
    except AttributeError:
        pass


def plot_quivers(ax, data, color, label, scale, normalize, alpha=0.8):
    xyz = data[:, :3]
    dirs = data[:, 3:6]
    if normalize:
        norms = np.linalg.norm(dirs, axis=1)
        norms[norms == 0] = 1.0
        dirs = dirs / norms[:, None]
    dirs = dirs * scale

    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=color, linewidth=1.2, label=label)
    ax.quiver(
        xyz[:, 0],
        xyz[:, 1],
        xyz[:, 2],
        dirs[:, 0],
        dirs[:, 1],
        dirs[:, 2],
        color=color,
        length=1.0,
        normalize=False,
        alpha=alpha,
    )
    return xyz


def euler_ue_to_rotmat(pitch_deg, yaw_deg, roll_deg):
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)
    roll = np.deg2rad(roll_deg)

    cr = np.cos(-roll)
    sr = np.sin(-roll)
    cp = np.cos(-pitch)
    sp = np.sin(-pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    r_roll = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    r_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    r_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    return r_yaw @ r_pitch @ r_roll


def ue_forward_vectors(ue_data):
    dirs = []
    for row in ue_data:
        pitch, yaw, roll = row[3], row[4], row[5]
        rot = euler_ue_to_rotmat(pitch, yaw, roll)
        dirs.append(rot @ np.array([1.0, 0.0, 0.0]))
    return np.array(dirs, dtype=float)


def plot_trajectory(ax, ue_data, color, label, scale, show_forward, normalize, alpha):
    xyz = ue_data[:, :3]
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=color, linewidth=1.4, label=label)
    if show_forward:
        dirs = ue_forward_vectors(ue_data)
        if normalize:
            norms = np.linalg.norm(dirs, axis=1)
            norms[norms == 0] = 1.0
            dirs = dirs / norms[:, None]
        dirs = dirs * scale
        ax.quiver(
            xyz[:, 0],
            xyz[:, 1],
            xyz[:, 2],
            dirs[:, 0],
            dirs[:, 1],
            dirs[:, 2],
            color=color,
            length=1.0,
            normalize=False,
            alpha=alpha,
        )
    return xyz


def build_parser():
    parser = argparse.ArgumentParser(
        description="Visualize trajectory quivers and/or stamped UE poses."
    )
    parser.add_argument("--before", help="CSV file of before quivers.")
    parser.add_argument("--after", help="CSV file of after quivers.")
    parser.add_argument("--quiver", help="CSV file of a single quiver set.")
    parser.add_argument(
        "--before-block",
        default="first",
        help="Block selector for before file: first, last, or index.",
    )
    parser.add_argument(
        "--after-block",
        default="last",
        help="Block selector for after file: first, last, or index.",
    )
    parser.add_argument(
        "--quiver-block",
        default="last",
        help="Block selector for quiver file: first, last, or index.",
    )
    parser.add_argument("--stamped-ue", help="Stamped UE pose file.")
    parser.add_argument(
        "--ue-block",
        default="first",
        help="Block selector for stamped UE file: first, last, or index.",
    )
    parser.add_argument(
        "--mode",
        choices=["overlay", "split"],
        default="overlay",
        help="Overlay both or split into two subplots.",
    )
    parser.add_argument("--scale", type=float, default=0.5, help="Quiver scale.")
    parser.add_argument(
        "--ue-forward-scale",
        type=float,
        default=0.3,
        help="Scale for UE forward direction arrows.",
    )
    parser.add_argument(
        "--show-ue-forward",
        action="store_true",
        help="Draw forward direction arrows from stamped UE poses.",
    )
    parser.add_argument(
        "--no-normalize",
        action="store_true",
        help="Do not normalize quiver directions.",
    )
    parser.add_argument("--title", default="", help="Optional plot title.")
    parser.add_argument("--save", default="", help="Save figure to this path.")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not call plt.show() (useful on headless systems).",
    )
    add_wandb_args(parser)
    return parser


def main():
    args = build_parser().parse_args()
    run_name = "trajectory_visualization"
    run, wandb_mod = safe_init(
        args,
        config={
            "before": args.before,
            "after": args.after,
            "quiver": args.quiver,
            "stamped_ue": args.stamped_ue,
            "mode": args.mode,
            "scale": args.scale,
            "ue_forward_scale": args.ue_forward_scale,
            "show_ue_forward": args.show_ue_forward,
            "normalize": not args.no_normalize,
        },
        run_name=run_name)

    has_before_after = bool(args.before) and bool(args.after)
    has_quiver = bool(args.quiver)
    if not has_before_after and not has_quiver:
        raise ValueError("Provide --before/--after or --quiver.")
    if has_before_after and has_quiver:
        raise ValueError("Use either --before/--after or --quiver, not both.")

    ue_data = None
    if args.stamped_ue:
        ue_path = Path(args.stamped_ue).expanduser()
        if not ue_path.exists():
            raise FileNotFoundError(f"Stamped UE file not found: {ue_path}")
        ue_blocks = parse_stamped_ue_blocks(ue_path)
        ue_data = select_block(ue_blocks, args.ue_block, "ue")

    if has_before_after:
        before_path = Path(args.before).expanduser()
        after_path = Path(args.after).expanduser()
        if not before_path.exists():
            raise FileNotFoundError(f"Before file not found: {before_path}")
        if not after_path.exists():
            raise FileNotFoundError(f"After file not found: {after_path}")

        before_blocks = parse_quiver_blocks(before_path)
        after_blocks = parse_quiver_blocks(after_path)
        before = select_block(before_blocks, args.before_block, "before")
        after = select_block(after_blocks, args.after_block, "after")
    else:
        quiver_path = Path(args.quiver).expanduser()
        if not quiver_path.exists():
            raise FileNotFoundError(f"Quiver file not found: {quiver_path}")
        quiver_blocks = parse_quiver_blocks(quiver_path)
        quiver = select_block(quiver_blocks, args.quiver_block, "quiver")

    normalize = not args.no_normalize

    if has_before_after and args.mode == "split":
        fig = plt.figure(figsize=(12, 5))
        ax_before = fig.add_subplot(1, 2, 1, projection="3d")
        ax_after = fig.add_subplot(1, 2, 2, projection="3d")

        before_xyz = plot_quivers(
            ax_before, before, "tab:red", "Before", args.scale, normalize
        )
        after_xyz = plot_quivers(
            ax_after, after, "tab:blue", "After", args.scale, normalize
        )
        if ue_data is not None:
            plot_trajectory(
                ax_before,
                ue_data,
                "0.2",
                "UE trajectory",
                args.ue_forward_scale,
                args.show_ue_forward,
                normalize,
                0.5,
            )
            plot_trajectory(
                ax_after,
                ue_data,
                "0.2",
                "UE trajectory",
                args.ue_forward_scale,
                args.show_ue_forward,
                normalize,
                0.5,
            )

        ax_before.set_title("Before")
        ax_after.set_title("After")
        set_axes_equal(ax_before, before_xyz)
        set_axes_equal(ax_after, after_xyz)
        ax_before.legend()
        ax_after.legend()
    else:
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(1, 1, 1, projection="3d")
        all_xyz = []
        if has_before_after:
            before_xyz = plot_quivers(
                ax, before, "tab:red", "Before", args.scale, normalize, alpha=0.6
            )
            after_xyz = plot_quivers(
                ax, after, "tab:blue", "After", args.scale, normalize, alpha=0.6
            )
            all_xyz = [before_xyz, after_xyz]
        else:
            quiver_xyz = plot_quivers(
                ax, quiver, "tab:orange", "Quiver", args.scale, normalize, alpha=0.7
            )
            all_xyz = [quiver_xyz]
        if ue_data is not None:
            traj_xyz = plot_trajectory(
                ax,
                ue_data,
                "0.2",
                "UE trajectory",
                args.ue_forward_scale,
                args.show_ue_forward,
                normalize,
                0.5,
            )
            all_xyz.append(traj_xyz)
        all_xyz = np.vstack(all_xyz)
        set_axes_equal(ax, all_xyz)
        ax.legend()
        if has_before_after:
            ax.set_title("Before vs After")
        else:
            ax.set_title("Quiver with UE trajectory")

    if args.title:
        fig.suptitle(args.title)

    if args.save:
        fig.savefig(args.save, dpi=200, bbox_inches="tight")

    if wandb_mod is not None:
        metrics = {}
        if has_before_after:
            metrics["before_count"] = int(before.shape[0])
            metrics["after_count"] = int(after.shape[0])
        if has_quiver:
            metrics["quiver_count"] = int(quiver.shape[0])
        if ue_data is not None:
            metrics["ue_pose_count"] = int(ue_data.shape[0])
        log_metrics(wandb_mod, metrics)
        wandb_mod.log({"trajectory_plot": wandb_mod.Image(fig)})

    if not args.no_show:
        plt.show()

    finish(run)


if __name__ == "__main__":
    main()
