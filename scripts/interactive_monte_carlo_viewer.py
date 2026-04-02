#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive viewer for Monte Carlo / on-manifold optimization.

Features:
- Choose a pose ("cluster location") from the pose CSV.
- Click "Optimize" to run the C++ binary and stream per-iteration directions.
- Animate the red quiver moving toward the final optimized rotation.
- Click "Brute Force" to reveal the brute-force (blue) quiver for that pose.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

import matplotlib

# Tkinter GUI + matplotlib 3D needs TkAgg.
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover
    yaml = None

import tkinter as tk
from tkinter import ttk, messagebox


def _resolve_map_and_manifest(
    repo_root: Path,
    *,
    map_manifest: Optional[Path],
    map_path: Optional[Path],
    map_name: Optional[str],
) -> Tuple[Path, Path, int]:
    """
    Returns (map_csv_path, pose_csv_path, clusters_count).
    """

    map_dir = repo_root / "Map"

    if map_manifest is not None:
        manifest_path = map_manifest
        if not manifest_path.is_absolute():
            manifest_path = (repo_root / manifest_path).resolve()
    else:
        manifest_candidates: List[Path] = []

        if map_name:
            # Common layouts:
            # - Map/<name>/<name>_map_manifest.yaml
            # - Map/<name>/*_map_manifest.yaml
            d = map_dir / map_name
            if d.exists():
                manifest_candidates.extend(sorted(d.glob("*_map_manifest.yaml")))
                manifest_candidates.extend(sorted(d.glob(f"{map_name}_map_manifest.yaml")))

        if not manifest_candidates:
            manifest_candidates = sorted(map_dir.glob("**/*_map_manifest.yaml"))

        if not manifest_candidates:
            raise SystemExit(
                "No map manifest found. Provide --map-manifest, --map-name, or --map."
            )
        if len(manifest_candidates) > 1:
            raise SystemExit(
                "Multiple map manifests found; provide --map-manifest to disambiguate."
            )

        manifest_path = manifest_candidates[0]

    if yaml is None:
        raise SystemExit("PyYAML is required to parse the manifest YAML.")

    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise SystemExit(f"Unexpected manifest format: {manifest_path}")

    base_map = manifest.get("base_map")
    pose_map = manifest.get("pose_map")
    clusters = manifest.get("clusters")

    if base_map is None or pose_map is None or clusters is None:
        raise SystemExit(
            f"Manifest missing base_map/pose_map/clusters: {manifest_path}"
        )

    map_csv = Path(str(base_map))
    pose_csv = Path(str(pose_map))
    if not map_csv.is_absolute():
        map_csv = (repo_root / map_csv).resolve()
    if not pose_csv.is_absolute():
        pose_csv = (repo_root / pose_csv).resolve()

    if map_path is not None:
        # Allow overriding the map csv while still using the pose/clusters from manifest.
        map_csv = map_path.resolve()

    try:
        clusters_int = int(clusters)
    except Exception as exc:
        raise SystemExit(f"Invalid clusters value in manifest: {clusters}") from exc

    if not map_csv.exists():
        raise SystemExit(f"Map CSV not found: {map_csv}")
    if not pose_csv.exists():
        raise SystemExit(f"Pose CSV not found: {pose_csv}")

    return map_csv, pose_csv, clusters_int


def _load_xyz_csv(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(path)
    data = np.loadtxt(str(path), delimiter=",", dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 3:
        raise ValueError(f"Expected at least 3 columns in {path}, got {data.shape}")
    return data[:, 0:3].astype(float)


def _read_csv_numeric_rows(path: Path) -> np.ndarray:
    """
    Read a numeric CSV file (optionally with a header). Returns NxM float array.
    """
    if not path.exists():
        return np.zeros((0, 0), dtype=float)
    rows: List[List[float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.replace("\x00", "").strip()
            if not line:
                continue
            # Skip header / non-numeric lines.
            if any(ch.isalpha() for ch in line.split(",")[0]):
                continue
            parts = [p.strip() for p in line.split(",")]
            try:
                rows.append([float(parts[i]) for i in range(len(parts))])
            except Exception:
                continue
    if not rows:
        return np.zeros((0, 0), dtype=float)
    return np.asarray(rows, dtype=float)


def _append_csv_chunk_to_dirs(
    chunk_text: str, carry: str
) -> Tuple[List[np.ndarray], str]:
    """
    Parse new bytes into direction vectors (dir_x,dir_y,dir_z).

    Expected row format:
      id,ref_x,ref_y,ref_z,dir_x,dir_y,dir_z,feature_count,degree_between,j_norm,step,visibility
    We extract dir_x..dir_z from columns 4..6.
    """
    text = carry + chunk_text
    if not text:
        return [], ""

    # Normalize Windows line endings.
    text = text.replace("\r", "")
    # Keep incomplete last line in carry (if file write isn't newline-terminated yet).
    last_nl = text.rfind("\n")
    if last_nl == -1:
        return [], text
    complete = text[:last_nl]
    new_carry = text[last_nl + 1 :]

    out_dirs: List[np.ndarray] = []
    for line in complete.split("\n"):
        line = line.strip()
        if not line:
            continue
        # Skip header or other accidental content.
        first = line.split(",", 1)[0]
        if any(ch.isalpha() for ch in first):
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 7:
            continue
        try:
            dx = float(parts[4])
            dy = float(parts[5])
            dz = float(parts[6])
        except Exception:
            continue
        v = np.asarray([dx, dy, dz], dtype=float)
        out_dirs.append(v)
    return out_dirs, new_carry


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return v
    return v / n


def _compute_bounds(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    return mins, maxs


def _quiver_length_from_bounds(bounds: Tuple[np.ndarray, np.ndarray], fraction: float = 0.05, fallback: float = 1.0) -> float:
    mins, maxs = bounds
    ranges = maxs - mins
    max_range = float(np.max(ranges))
    if max_range <= 0:
        return fallback
    return max(fallback, fraction * max_range)


@dataclass
class RunState:
    running: bool = False
    worker_thread: Optional[threading.Thread] = None
    proc: Optional[subprocess.Popen] = None
    last_file_size_bytes: int = 0
    carry: str = ""
    opt_dirs: List[np.ndarray] = None  # type: ignore[assignment]
    base_ref: Optional[np.ndarray] = None
    opt_final_dir: Optional[np.ndarray] = None
    bf_dir: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        if self.opt_dirs is None:
            self.opt_dirs = []


class InteractiveMonteCarloViewer(tk.Tk):
    def __init__(
        self,
        *,
        repo_root: Path,
        map_csv: Path,
        pose_csv: Path,
        clusters: int,
        cmake_build: bool,
        speed_ms: int,
        max_points: int,
    ) -> None:
        super().__init__()
        self.repo_root = repo_root
        self.data_dir = repo_root / "Data"
        self.build_dir = repo_root / "Manifold_cpp" / "build"
        self.bin_path = self.build_dir / "manifold_test"

        self.map_csv = map_csv
        self.pose_csv = pose_csv
        self.clusters = clusters
        self.cmake_build = cmake_build

        self.speed_ms = speed_ms
        self.max_points = max_points

        self.state = RunState()

        self.title("Monte Carlo Interactive Viewer (On-Manifold Optimization)")
        self.geometry("1200x800")

        self._build_ui()
        self._load_map_and_poses()
        self._display_frame_idx = 0

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        main = ttk.Frame(self, padding=6)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main, padding=6)
        left.grid(row=0, column=0, sticky="ns")

        ttk.Label(left, text="Pose / Cluster Location").grid(row=0, column=0, sticky="w")

        # Pose list
        self.pose_listbox = tk.Listbox(left, height=28, width=48, exportselection=False)
        self.pose_listbox.grid(row=1, column=0, sticky="nsew", pady=(6, 6))

        # Scrollbar
        sb = ttk.Scrollbar(left, orient="vertical", command=self.pose_listbox.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.pose_listbox.configure(yscrollcommand=sb.set)

        # Selected pose label
        self.pose_label = ttk.Label(left, text="Selected: (none)")
        self.pose_label.grid(row=2, column=0, columnspan=2, sticky="w")

        self.pose_listbox.bind("<<ListboxSelect>>", self._on_pose_selected)

        # Buttons
        btns = ttk.Frame(left)
        btns.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        self.btn_opt = ttk.Button(btns, text="Optimize", command=self.on_optimize_clicked)
        self.btn_opt.grid(row=0, column=0, sticky="ew", padx=(0, 6))

        self.btn_bf = ttk.Button(btns, text="Brute Force", command=self.on_bf_clicked)
        self.btn_bf.grid(row=0, column=1, sticky="ew", padx=(6, 0))

        # Playback speed
        speed_frame = ttk.LabelFrame(left, text="Animation")
        speed_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(12, 0))

        ttk.Label(speed_frame, text="Frame delay (ms)").grid(row=0, column=0, sticky="w")
        self.speed_scale = ttk.Scale(
            speed_frame,
            from_=10,
            to=300,
            value=float(self.speed_ms),
            command=self._on_speed_change,
        )
        self.speed_scale.grid(row=1, column=0, sticky="ew", pady=(4, 4))
        self.speed_value = ttk.Label(speed_frame, text=f"{self.speed_ms} ms")
        self.speed_value.grid(row=2, column=0, sticky="w")

        # Misc
        misc = ttk.LabelFrame(left, text="Status")
        misc.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(misc, textvariable=self.status_var, wraplength=250).grid(
            row=0, column=0, sticky="w", padx=6, pady=6
        )
        self.iter_var = tk.StringVar(value="Frames displayed: 0 / 0")
        ttk.Label(misc, textvariable=self.iter_var).grid(
            row=1, column=0, sticky="w", padx=6
        )

        # Right: plot
        plot_frame = ttk.Frame(main, padding=6)
        plot_frame.grid(row=0, column=1, sticky="nsew")
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)

        self.fig = Figure(figsize=(7, 7), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Quiver animation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Quivers: red (optimized stream), blue (BF final)
        self.q_opt = None
        self.q_bf = None

        self.scatter_map = None
        self.scatter_pose = None

        # Polling loop for streaming file updates
        self.after(100, self._poll_stream)

    def _on_speed_change(self, _val: str) -> None:
        self.speed_ms = int(float(self.speed_scale.get()))
        self.speed_value.config(text=f"{self.speed_ms} ms")

    def _load_map_and_poses(self) -> None:
        if not self.bin_path.exists():
            messagebox.showerror(
                "Binary missing",
                f"Could not find binary: {self.bin_path}",
            )
            raise SystemExit(1)

        self.map_points = _load_xyz_csv(self.map_csv)
        self.pose_points = _load_xyz_csv(self.pose_csv)

        # Optional downsample for faster rendering
        if self.map_points.shape[0] > self.max_points:
            idx = np.random.choice(self.map_points.shape[0], self.max_points, replace=False)
            self.map_points_sample = self.map_points[idx]
        else:
            self.map_points_sample = self.map_points

        self.pose_listbox.delete(0, tk.END)
        for i in range(self.pose_points.shape[0]):
            p = self.pose_points[i]
            self.pose_listbox.insert(i, f"{i}: ({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")

        # Select first pose by default
        if self.pose_points.shape[0] > 0:
            self.pose_listbox.selection_set(0)
            self.current_pose_idx = 0
        else:
            self.current_pose_idx = -1

        self._redraw_scene(clear_quivers=True)
        self.status_var.set("Ready. Select a pose and click Optimize.")

    def _on_pose_selected(self, _event: tk.Event) -> None:
        if self.state.running:
            return
        sel = self.pose_listbox.curselection()
        if not sel:
            return
        self.current_pose_idx = int(sel[0])
        self.state = RunState()  # reset
        self.state.opt_dirs = []
        self.state.base_ref = None
        self._redraw_scene(clear_quivers=True)

    def _redraw_scene(self, *, clear_quivers: bool) -> None:
        self.ax.clear()

        # Scatter map points
        self.scatter_map = self.ax.scatter(
            self.map_points_sample[:, 0],
            self.map_points_sample[:, 1],
            self.map_points_sample[:, 2],
            c="gray",
            alpha=0.35,
            s=3,
            depthshade=False,
        )

        # Selected pose marker
        if self.current_pose_idx >= 0:
            p = self.pose_points[self.current_pose_idx]
            self.scatter_pose = self.ax.scatter([p[0]], [p[1]], [p[2]], c="black", s=60, marker="x")
            self.pose_label.config(text=f"Selected: {self.current_pose_idx} @ ({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")
        else:
            self.pose_label.config(text="Selected: (none)")

        # Bounds / aspect
        bounds = _compute_bounds(self.map_points_sample)
        mins, maxs = bounds
        # Include the selected pose in bounds.
        if self.current_pose_idx >= 0:
            p = self.pose_points[self.current_pose_idx]
            mins = np.minimum(mins, p)
            maxs = np.maximum(maxs, p)

        scale = _quiver_length_from_bounds((mins, maxs))

        # Set limits
        self.ax.set_xlim(float(mins[0]), float(maxs[0]))
        self.ax.set_ylim(float(mins[1]), float(maxs[1]))
        self.ax.set_zlim(float(mins[2]), float(maxs[2]))

        # Box aspect can distort if z range is tiny; provide z_scale.
        dx, dy, dz = (maxs - mins).tolist()
        dz = max(dz, 1e-6)
        self.ax.set_box_aspect((max(dx, 1e-6), max(dy, 1e-6), dz * 1.5))

        self.ax.set_title("Quiver animation (red: optimize stream, blue: BF final)")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        if clear_quivers:
            self.q_opt = None
            self.q_bf = None

        self.canvas.draw_idle()

    def _init_quivers_if_needed(self) -> None:
        if self.current_pose_idx < 0:
            return
        p = self.pose_points[self.current_pose_idx]
        base_ref = np.asarray(p, dtype=float)
        if self.state.base_ref is None:
            self.state.base_ref = base_ref

        mins, maxs = _compute_bounds(self.map_points_sample)
        if self.current_pose_idx >= 0:
            mins = np.minimum(mins, base_ref)
            maxs = np.maximum(maxs, base_ref)
        scale = _quiver_length_from_bounds((mins, maxs))
        self._scale_cached = scale

        # Initialize quivers (single-arrow each). We draw BF first so it stays visually behind.
        if self.q_bf is None:
            self.q_bf = self.ax.quiver(
                base_ref[0],
                base_ref[1],
                base_ref[2],
                0.0,
                0.0,
                0.0,
                length=scale * 0.85,  # BF slightly longer
                normalize=True,
                color="blue",
                alpha=0.0,  # hidden until user clicks BF
                linewidth=2.0,
                arrow_length_ratio=0.35,  # bigger head
            )
        if self.q_opt is None:
            self.q_opt = self.ax.quiver(
                base_ref[0],
                base_ref[1],
                base_ref[2],
                0.0,
                0.0,
                0.0,
                length=scale * 0.65,
                normalize=True,
                color="red",
                alpha=0.95,
                linewidth=2.5,
                arrow_length_ratio=0.35,  # bigger head
            )
        self._scale_cached = scale

    def _set_quiver(self, which: str, dir_vec: np.ndarray, *, show: bool) -> None:
        if self.current_pose_idx < 0:
            return
        self._init_quivers_if_needed()
        base_ref = self.state.base_ref
        if base_ref is None:
            return

        u = _unit(dir_vec)
        scale = getattr(self, "_scale_cached", 1.0)

        if which == "bf":
            if self.q_bf is not None:
                try:
                    self.q_bf.remove()
                except Exception:
                    pass
            # Draw BF first (back), lower alpha, slightly longer.
            self.q_bf = self.ax.quiver(
                base_ref[0],
                base_ref[1],
                base_ref[2],
                float(u[0]),
                float(u[1]),
                float(u[2]),
                length=scale * 0.90,
                normalize=True,
                color="blue",
                alpha=0.28 if show else 0.0,
                linewidth=2.2,
                arrow_length_ratio=0.40,  # bigger head
            )
        else:
            if self.q_opt is not None:
                try:
                    self.q_opt.remove()
                except Exception:
                    pass
            self.q_opt = self.ax.quiver(
                base_ref[0],
                base_ref[1],
                base_ref[2],
                float(u[0]),
                float(u[1]),
                float(u[2]),
                length=scale * 0.70,
                normalize=True,
                color="red",
                alpha=0.95 if show else 0.0,
                linewidth=2.8,
                arrow_length_ratio=0.40,  # bigger head
            )

    def _update_opt_line(self, dir_vec: np.ndarray) -> None:
        if self.current_pose_idx < 0:
            return
        self._set_quiver("opt", dir_vec, show=True)

    def _show_bf_line(self, bf_dir_vec: np.ndarray) -> None:
        if self.current_pose_idx < 0:
            return
        self._set_quiver("bf", bf_dir_vec, show=True)

    def _set_status(self, s: str) -> None:
        self.status_var.set(s)
        self.canvas.draw_idle()

    def on_optimize_clicked(self) -> None:
        if self.state.running:
            return
        if self.current_pose_idx < 0:
            messagebox.showwarning("No pose selected", "Select a pose first.")
            return

        self.state = RunState()
        self.state.opt_dirs = []
        self.state.base_ref = np.asarray(self.pose_points[self.current_pose_idx], dtype=float)
        self._display_frame_idx = 0
        self._redraw_scene(clear_quivers=True)
        self._init_quivers_if_needed()

        self._set_status("Running optimize (streaming per-iteration quivers)...")
        self.btn_opt.config(state=tk.DISABLED)
        self.btn_bf.config(state=tk.DISABLED)

        # Optional build step.
        if self.cmake_build:
            self._set_status("Building C++ binary (this may take a while)...")
            self._build_binary_blocking()
            self._set_status("Binary built. Starting optimize...")

        worker = threading.Thread(target=self._run_cpp_and_stream_setup, daemon=True)
        self.state.running = True
        self.state.worker_thread = worker
        worker.start()

    def on_bf_clicked(self) -> None:
        if self.state.running:
            return
        if self.state.bf_dir is not None:
            self._show_bf_line(self.state.bf_dir)
            self.canvas.draw_idle()
            self._set_status("Brute-force quiver shown.")
            return

        # If BF is not available, run optimize once (binary computes BF as part of the run).
        self._set_status("BF not cached yet; running optimize once to compute BF...")
        self.on_optimize_clicked()

    def _build_binary_blocking(self) -> None:
        # Keep it minimal and reuse existing CMake cache.
        # We intentionally avoid complicated argument handling here.
        cmd_cmake = ["cmake", "-DUSE_MANIFOLD_BACKUP=OFF", ".."]
        subprocess.check_call(cmd_cmake, cwd=str(self.build_dir))
        subprocess.check_call(["make", "-j"], cwd=str(self.build_dir))

    def _run_cpp_and_stream_setup(self) -> None:
        """
        Worker thread: runs the binary; the UI thread polls the quivers file.
        """
        assert self.state.base_ref is not None

        # Create a temp pose file with a single pose.
        tmp_pose_path = self.data_dir / "viewer_pose_tmp.csv"
        tmp_pose_path.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(tmp_pose_path, np.asarray([self.state.base_ref], dtype=float), delimiter=",", fmt="%.8f")

        quivers_path = self.data_dir / "quiversforonepoint.csv"
        # Start reading from the current end-of-file to capture only this run's appended rows.
        try:
            self.state.last_file_size_bytes = quivers_path.stat().st_size if quivers_path.exists() else 0
        except OSError:
            self.state.last_file_size_bytes = 0
        self.state.carry = ""
        self.state.opt_final_dir = None
        self.state.bf_dir = None

        # Start subprocess.
        env = os.environ.copy()
        env["FOV_MAP_PATH"] = str(self.map_csv.resolve())
        # Keep bounds aligned if present (helps internal consistency).
        # If bounds are missing, it's okay: pose-based optimization doesn't rely on the grid.
        manifest_path = None
        map_manifest_candidates = list((self.repo_root / "Map").glob("**/*_map_manifest.yaml"))
        if len(map_manifest_candidates) == 1:
            manifest_path = map_manifest_candidates[0]
        if manifest_path is not None and yaml is not None:
            try:
                manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
                bounds = manifest.get("bounds")
                if isinstance(bounds, list) and len(bounds) == 6:
                    env["FOV_BOUNDS"] = ",".join(str(float(x)) for x in bounds)
            except Exception:
                pass

        # Critical: enable per-iteration logs for streaming.
        env["FOV_NO_IO"] = "0"
        env["FOV_USE_OPENMP"] = "0"
        env["FOV_MINIMAL_LOG"] = "1"

        cmd = [
            str(self.bin_path),
            "1",  # resolution -> feature_count_resolution
            "1",  # import_map
            str(self.clusters),
            str(tmp_pose_path.resolve()),
        ]
        try:
            proc = subprocess.Popen(cmd, cwd=str(self.build_dir), env=env)
        except Exception as exc:
            self.state.running = False

            def _fail_ui() -> None:
                self._set_status(f"Failed to start binary: {exc}")
                self.btn_opt.config(state=tk.NORMAL)
                self.btn_bf.config(state=tk.NORMAL)

            self.after(0, _fail_ui)
            return

        self.state.proc = proc
        exit_code = None
        try:
            exit_code = proc.wait()
        finally:
            pass

        # Load final dirs from output files.
        opt_quivers_path = self.data_dir / "0_1_single_run_rotated_quivers.csv"
        bf_quivers_path = self.data_dir / "0_1_single_run_brute_force_rotated_quivers.csv"

        opt_quivers = _read_csv_numeric_rows(opt_quivers_path)
        bf_quivers = _read_csv_numeric_rows(bf_quivers_path)

        if opt_quivers.shape[0] >= 1 and opt_quivers.shape[1] >= 6:
            self.state.opt_final_dir = opt_quivers[0, 3:6]
        if bf_quivers.shape[0] >= 1 and bf_quivers.shape[1] >= 9:
            # columns: ref_x,ref_y,ref_z,bf_feat_x,bf_feat_y,bf_feat_z,bf_vis_x,bf_vis_y,bf_vis_z
            self.state.bf_dir = bf_quivers[0, 6:9]

        frames_streamed = len(self.state.opt_dirs)
        opt_final = self.state.opt_final_dir
        self.state.running = False

        def _finish_ui() -> None:
            self._display_frame_idx = frames_streamed
            # Ensure the red line ends at the final optimized dir.
            if opt_final is not None:
                self._update_opt_line(opt_final)
            # Final BF line remains hidden until BF button is clicked.
            self._set_status(
                f"Optimize finished. exit_code={exit_code}. Frames streamed: {frames_streamed}"
            )
            self.iter_var.set(f"Frames displayed: {frames_streamed} / {frames_streamed}")
            self.btn_opt.config(state=tk.NORMAL)
            self.btn_bf.config(state=tk.NORMAL)
            self.canvas.draw_idle()

        self.after(0, _finish_ui)

    def _poll_stream(self) -> None:
        """
        UI thread poll: read new appended lines and update the red quiver.
        """
        if not self.state.running:
            # When not streaming, still allow redraw if final dirs exist.
            self.canvas.draw_idle()
            self.after(120, self._poll_stream)
            return

        quivers_path = self.data_dir / "quiversforonepoint.csv"
        if not quivers_path.exists():
            self.after(120, self._poll_stream)
            return

        try:
            current_size = quivers_path.stat().st_size
        except OSError:
            self.after(120, self._poll_stream)
            return

        if current_size <= self.state.last_file_size_bytes:
            self.after(80, self._poll_stream)
            return

        # Read only the new bytes appended since last poll.
        with quivers_path.open("rb") as f:
            f.seek(self.state.last_file_size_bytes)
            chunk = f.read(current_size - self.state.last_file_size_bytes)

        self.state.last_file_size_bytes = current_size

        chunk_text = chunk.decode("utf-8", errors="ignore")
        new_dirs, self.state.carry = _append_csv_chunk_to_dirs(
            chunk_text, self.state.carry
        )
        if new_dirs:
            for d in new_dirs:
                self.state.opt_dirs.append(d)

        # Display exactly one frame per tick (sequential playback).
        if self._display_frame_idx < len(self.state.opt_dirs):
            self._init_quivers_if_needed()
            self._update_opt_line(self.state.opt_dirs[self._display_frame_idx])
            self._display_frame_idx += 1
            self.iter_var.set(
                f"Frames displayed: {self._display_frame_idx} / {len(self.state.opt_dirs)}"
            )
            self.canvas.draw_idle()
        else:
            self.iter_var.set(f"Frames displayed: {self._display_frame_idx} / {len(self.state.opt_dirs)}")

        self.after(max(30, self.speed_ms), self._poll_stream)


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive Monte Carlo quiver viewer")
    parser.add_argument("--repo-root", default=None, help="Path to My_FoV_Optimization (default: script parent).")
    parser.add_argument("--map-manifest", default=None, help="Path to *_map_manifest.yaml")
    parser.add_argument("--map", default=None, help="Override map CSV path (pose/clusters still from manifest).")
    parser.add_argument("--map-name", default=None, help="Map folder under Map/ (optional convenience)")
    parser.add_argument("--build", action="store_true", help="Build the C++ binary before running.")
    parser.add_argument("--speed-ms", type=int, default=60, help="Animation frame delay in milliseconds.")
    parser.add_argument("--max-points", type=int, default=6000, help="Max map points to render.")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve() if args.repo_root else Path(__file__).resolve().parents[1]

    map_manifest_path = Path(args.map_manifest).resolve() if args.map_manifest else None
    map_path_override = Path(args.map).resolve() if args.map else None

    map_csv, pose_csv, clusters = _resolve_map_and_manifest(
        repo_root,
        map_manifest=map_manifest_path,
        map_path=map_path_override,
        map_name=args.map_name,
    )

    app = InteractiveMonteCarloViewer(
        repo_root=repo_root,
        map_csv=map_csv,
        pose_csv=pose_csv,
        clusters=clusters,
        cmake_build=args.build,
        speed_ms=args.speed_ms,
        max_points=args.max_points,
    )
    app.mainloop()


if __name__ == "__main__":
    main()

