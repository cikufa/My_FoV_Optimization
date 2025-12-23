#!/usr/bin/env python3
"""
Visualize before/after trajectory quivers for optimization.

Input format: each line has x,y,z,dx,dy,dz (comma or whitespace separated).
Files can contain multiple blocks separated by blank lines; select with --before-block/--after-block.
"""

import argparse
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


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


def build_parser():
    parser = argparse.ArgumentParser(
        description="Visualize before/after trajectory quivers."
    )
    parser.add_argument("--before", required=True, help="CSV file of before quivers.")
    parser.add_argument("--after", required=True, help="CSV file of after quivers.")
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
        "--mode",
        choices=["overlay", "split"],
        default="overlay",
        help="Overlay both or split into two subplots.",
    )
    parser.add_argument("--scale", type=float, default=0.5, help="Quiver scale.")
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
    return parser


def main():
    args = build_parser().parse_args()

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

    normalize = not args.no_normalize

    if args.mode == "split":
        fig = plt.figure(figsize=(12, 5))
        ax_before = fig.add_subplot(1, 2, 1, projection="3d")
        ax_after = fig.add_subplot(1, 2, 2, projection="3d")

        before_xyz = plot_quivers(
            ax_before, before, "tab:red", "Before", args.scale, normalize
        )
        after_xyz = plot_quivers(
            ax_after, after, "tab:blue", "After", args.scale, normalize
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
        before_xyz = plot_quivers(
            ax, before, "tab:red", "Before", args.scale, normalize, alpha=0.6
        )
        after_xyz = plot_quivers(
            ax, after, "tab:blue", "After", args.scale, normalize, alpha=0.6
        )
        all_xyz = np.vstack([before_xyz, after_xyz])
        set_axes_equal(ax, all_xyz)
        ax.legend()
        ax.set_title("Before vs After")

    if args.title:
        fig.suptitle(args.title)

    if args.save:
        fig.savefig(args.save, dpi=200, bbox_inches="tight")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
