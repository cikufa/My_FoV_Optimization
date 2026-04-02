#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


def run_cmd(cmd, cwd=None, env=None):
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd, env=env)


def read_cmake_cache_value(build_dir, key):
    cache_path = Path(build_dir) / "CMakeCache.txt"
    if not cache_path.exists():
        return None
    with cache_path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line or line.startswith("//") or line.startswith("#"):
                continue
            if line.startswith(f"{key}:"):
                parts = line.strip().split("=", 1)
                if len(parts) == 2:
                    return parts[1].strip()
                return None
    return None

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


def find_pose_for_map(root, map_path):
    map_path = Path(map_path)
    candidates = []
    candidates.append(map_path.with_name(map_path.stem + "_poses.csv"))
    match = re.match(r"^(\d+_\d+)_(.+)$", map_path.name)
    if match:
        base_name = match.group(2)
        base_stem = Path(base_name).stem
        candidates.append(map_path.with_name(base_stem + "_poses.csv"))
    for cand in candidates:
        if cand.exists():
            return cand
    map_dir = root / "Map"
    if map_dir.exists():
        for cand in candidates:
            alt = map_dir / cand.name
            if alt.exists():
                return alt
    return None


def resolve_map_relative_path(root, user_path):
    path = Path(user_path)
    if path.is_absolute():
        return path
    parts = path.parts
    if parts and parts[0] == "Map":
        return root / path
    return root / "Map" / path


def build_bf_cache_signature(map_name, pose_name, grid=None, bounds=None, bf_objective="both"):
    bf_objective = (bf_objective or "both").strip().lower()
    if pose_name:
        return f"map={map_name};pose={pose_name};bf_objective={bf_objective}"
    if grid is None or bounds is None:
        return f"map={map_name};bf_objective={bf_objective}"
    x_res, y_res, z_res = grid
    x_low, x_high, y_low, y_high, z_low, z_high = bounds
    return (
        f"map={map_name};grid={x_res},{y_res},{z_res};bounds="
        f"{x_low},{x_high},{y_low},{y_high},{z_low},{z_high};bf_objective={bf_objective}"
    )


def seed_bf_cache(root, signatures, dest_dir):
    dest_dir.mkdir(parents=True, exist_ok=True)
    expected = {f"# signature:{sig}" for sig in signatures}
    results_dir = root / "Results" / "monte_carlo"
    if not results_dir.exists():
        return []
    copied = []
    for path in results_dir.glob("*/bf_cache/*bf_cache.csv"):
        try:
            with path.open("r", encoding="utf-8") as f:
                header = f.readline().strip()
        except OSError:
            continue
        if header not in expected:
            continue
        dest = dest_dir / path.name
        if not dest.exists():
            shutil.copy2(str(path), str(dest))
            enrich_cache_with_bf_times(dest, path)
            copied.append(dest)
    return copied


def enrich_cache_with_bf_times(cache_dest_path, cache_source_path):
    """
    Upgrade legacy BF cache (9 columns) by appending per-pose BF runtime as
    a 10th column, using sibling data/<prefix>brute_force_avg_time_file.csv.
    """
    try:
        with cache_dest_path.open("r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError:
        return False
    if len(lines) < 3:
        return False
    col_header = lines[1].strip()
    if "bf_time_us" in col_header:
        return False

    data_lines = [ln for ln in lines[2:] if ln.strip() and not ln.lstrip().startswith("#")]
    if not data_lines:
        return False
    pose_count = len(data_lines)

    m = re.match(r"^(.+)_bf_cache\.csv$", cache_source_path.name)
    if not m:
        return False
    prefix = m.group(1) + "_"
    bf_time_file = cache_source_path.parents[1] / "data" / f"{prefix}brute_force_avg_time_file.csv"
    if not bf_time_file.exists():
        return False

    times = []
    try:
        with bf_time_file.open("r", encoding="utf-8", errors="ignore") as f:
            raw = f.readlines()
        for ln in raw[1:]:
            txt = ln.replace("\x00", "").strip()
            if not txt:
                continue
            try:
                times.append(float(txt.split(",")[0]))
            except ValueError:
                continue
    except OSError:
        return False

    if len(times) < pose_count:
        return False
    times = times[:pose_count]

    new_lines = []
    new_lines.append(lines[0] if lines[0].endswith("\n") else lines[0] + "\n")
    new_lines.append("# columns: ref_x,ref_y,ref_z,bf_feat_x,bf_feat_y,bf_feat_z,bf_vis_x,bf_vis_y,bf_vis_z,bf_time_us\n")
    for idx, ln in enumerate(lines[2:]):
        if not ln.strip() or ln.lstrip().startswith("#"):
            new_lines.append(ln if ln.endswith("\n") else ln + "\n")
            continue
        row = ln.rstrip("\n")
        new_lines.append(f"{row},{times[idx]:.6f}\n")

    try:
        with cache_dest_path.open("w", encoding="utf-8") as f:
            f.writelines(new_lines)
    except OSError:
        return False
    return True


def read_cache_signature_line(path):
    try:
        with path.open("r", encoding="utf-8") as f:
            return f.readline().strip()
    except OSError:
        return None


def warn_if_bf_cache_level_mismatch(cache_path, subsample_prefix, level):
    """
    Catch corrupt all-levels saves where e.g. 0_1_bf_cache.csv contains a level-9 signature
    (seeding then skips level 1 and BF recomputes for every new run).
    """
    if level is None:
        return
    header = read_cache_signature_line(cache_path)
    if not header or not header.startswith("# signature:"):
        return
    token = f"/{subsample_prefix}_{level}_"
    if token not in header:
        print(
            f"[cache] WARNING: {cache_path.name} signature does not contain map token {token!r}:\n"
            f"  {header}\n"
            f"  Fix or delete this file; otherwise seeding will not match this level.",
            file=sys.stderr,
        )


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
        description="Run Monte Carlo grid/pose experiment and save outputs."
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
        "--manifold",
        choices=["current", "backup"],
        default="current",
        help="Select which manifold implementation to compile.",
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
        help="Map file to load (relative to Map/ or absolute).",
    )
    parser.add_argument(
        "--map-name",
        default=None,
        help="Named map folder from generate_cluster_map.py (Map/<name>/<name>_map_manifest.yaml).",
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
        help="Subsample level to use when --map/--map-manifest is provided.",
    )
    parser.add_argument(
        "--all-levels",
        action="store_true",
        help="Run all subsample levels from the manifest in one run folder.",
    )
    parser.add_argument(
        "--pose-file",
        default=None,
        help="Pose CSV file to use (relative to Map/ or absolute). Overrides manifest pose_map.",
    )
    parser.add_argument(
        "--bounds",
        default=None,
        help="Grid bounds x_low,x_high,y_low,y_high,z_low,z_high (passed via env to binary).",
    )
    parser.add_argument(
        "--import-map",
        type=int,
        choices=[0, 1],
        default=1,
        help="Pass 1 to load a map from disk (0 is not supported).",
    )
    parser.add_argument(
        "--clusters",
        type=int,
        default=8,
        help="Cluster count argument (required by binary).",
    )
    parser.add_argument(
        "--label",
        default="",
        help="Optional label appended to the output folder name.",
    )
    parser.add_argument(
        "--bf-objective",
        choices=["both", "feat", "vis"],
        default="both",
        help="Brute-force objective mode: both, feature-count only, or visibility only.",
    )
    parser.add_argument(
        "--start-strategy",
        choices=["single", "multistart", "adaptive"],
        default="single",
        help="Starting-direction strategy: keep current single-start behavior, try a few starts, or stop early when progress plateaus.",
    )
    parser.add_argument(
        "--start-count",
        type=int,
        default=3,
        help="Maximum number of ranked start candidates to optimize for multistart/adaptive modes.",
    )
    parser.add_argument(
        "--start-min-rel-gain",
        type=float,
        default=0.002,
        help="Adaptive mode: treat smaller best-score improvements as plateauing progress.",
    )
    parser.add_argument(
        "--start-adaptive-min-starts",
        type=int,
        default=2,
        help="Adaptive mode: minimum number of starts to try before early stopping is allowed.",
    )
    parser.add_argument(
        "--start-adaptive-patience",
        type=int,
        default=1,
        help="Adaptive mode: number of plateaued starts to tolerate before stopping.",
    )
    parser.add_argument(
        "--no-bf-start-seed",
        action="store_true",
        help="Do not include the cached brute-force visibility direction as an optimization seed.",
    )
    parser.add_argument(
        "--move",
        action="store_true",
        help="Move outputs into run folder instead of copying.",
    )

    args = parser.parse_args()
    root = Path(args.root).resolve()
    build_dir = root / "Manifold_cpp" / "build"
    bin_path = build_dir / "manifold_test"
    data_dir = root / "Data"
    manifold_test_path = root / "Manifold_cpp" / "manifold_test.cpp"

    # Backward/UX-friendly behavior: if the user accidentally passes a map CSV
    # to --map-manifest, treat it as --map.
    if args.map_manifest and not args.map:
        manifest_candidate = Path(args.map_manifest)
        suffix = manifest_candidate.suffix.lower()
        if suffix in (".csv", ".txt"):
            args.map = args.map_manifest
            args.map_manifest = None

    use_backup = args.manifold == "backup"
    cmake_define = "ON" if use_backup else "OFF"
    cache_val = read_cmake_cache_value(build_dir, "USE_MANIFOLD_BACKUP")
    if cache_val is None or cache_val.upper() != cmake_define:
        if not args.cmake:
            print("Manifold selection changed; enabling cmake.")
        if not args.build:
            print("Manifold selection changed; enabling build.")
        args.cmake = True
        args.build = True

    data_dir.mkdir(parents=True, exist_ok=True)

    if args.all_levels and args.level is not None:
        print("Ignoring --level because --all-levels was set.", file=sys.stderr)
        args.level = None

    if args.map_name:
        if args.map or args.map_manifest:
            print("Use only one of --map-name, --map, or --map-manifest.", file=sys.stderr)
            sys.exit(2)
        name = args.map_name
        candidates = [
            root / "Map" / name / f"{name}_map_manifest.yaml",
            root / "Map" / name / f"{name}_manifest.yaml",
            root / "Map" / f"{name}_manifest.yaml",
        ]
        for cand in candidates:
            if cand.exists():
                args.map_manifest = str(cand)
                break
        else:
            print(f"Map manifest not found for name '{name}'.", file=sys.stderr)
            sys.exit(2)

    if args.map and args.map_manifest:
        print("Use only one of --map or --map-manifest.", file=sys.stderr)
        sys.exit(2)

    map_name = None
    map_output = None
    map_path = None
    pose_path = None
    bounds = parse_bounds(args.bounds) if args.bounds else None
    manifest = None
    manifest_path = None
    subsample_maps = None

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
        pose_map = manifest.get("pose_map")
        selected_map = None
        if args.level is not None:
            level_key = str(args.level)
            if level_key in subsample_maps:
                selected_map = subsample_maps[level_key]
            elif args.level in subsample_maps:
                selected_map = subsample_maps[args.level]
            else:
                print(
                    f"Level {args.level} not found in manifest. Available: {sorted(subsample_maps.keys())}",
                    file=sys.stderr,
                )
                sys.exit(2)
        else:
            selected_map = base_map

        if not selected_map:
            hint = ""
            if manifest_path is not None:
                hint = (
                    f"\nHint: --map-manifest must be the YAML manifest (e.g. "
                    f"'{manifest_path.with_name(manifest_path.stem + '_manifest.yaml').name}' or "
                    f"'{manifest_path.name}'), not a .csv map file."
                )
            print(f"Manifest did not specify a base_map.{hint}", file=sys.stderr)
            sys.exit(2)
        map_path = Path(str(selected_map))
        if not map_path.is_absolute():
            map_path = root / map_path
        if not map_path.exists():
            print(f"Map not found: {map_path}", file=sys.stderr)
            sys.exit(2)
        if pose_map:
            pose_path = Path(str(pose_map))
            if not pose_path.is_absolute():
                pose_path = root / pose_path

    if args.map:
        map_path = resolve_map_relative_path(root, args.map)
        if not map_path.exists():
            print(f"Map not found: {map_path}", file=sys.stderr)
            sys.exit(2)
        found_manifest = find_manifest_for_map(root, map_path)
        if found_manifest is not None:
            manifest_path = found_manifest
            manifest = parse_simple_yaml(found_manifest)
            subsample_maps = manifest.get("subsample_maps", {})
            if args.level is not None:
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
            if not args.pose_file:
                pose_map = manifest.get("pose_map")
                if pose_map:
                    pose_path = Path(str(pose_map))
                    if not pose_path.is_absolute():
                        pose_path = root / pose_path
        elif args.level is not None:
            print(
                f"Level {args.level} requested but no manifest found for {map_path.name}.",
                file=sys.stderr,
            )
            sys.exit(2)

    if args.pose_file:
        pose_path = resolve_map_relative_path(root, args.pose_file)
        if not pose_path.exists():
            print(f"Pose file not found: {pose_path}", file=sys.stderr)
            sys.exit(2)
    if pose_path is None and map_path is not None:
        inferred = find_pose_for_map(root, map_path)
        if inferred:
            pose_path = inferred

    if map_path is None:
        print("Map path not provided. Use --map or --map-manifest.", file=sys.stderr)
        sys.exit(2)

    args.import_map = 1
    map_root = root / "Map"
    try:
        rel_map = map_path.relative_to(map_root)
        map_name = f"../../Map/{rel_map.as_posix()}"
    except ValueError:
        map_name = str(map_path)
    map_output = map_path.name

    if pose_path is None:
        print(
            "Pose file required. Generate poses with generate_cluster_map.py or pass --pose-file.",
            file=sys.stderr,
        )
        sys.exit(2)

    try:
        rel_pose = pose_path.relative_to(map_root)
        pose_name = f"../../Map/{rel_pose.as_posix()}"
    except ValueError:
        pose_name = str(pose_path)

    grid = None
    if args.grid is not None:
        grid = parse_grid(args.grid)
        changed = patch_grid_resolution(manifold_test_path, grid)
        if changed and not args.build:
            print("Grid change detected; enabling build.")
            args.build = True

    base_map_path = map_path
    subsample_prefix = "0"
    if manifest and isinstance(manifest, dict):
        subsample_prefix = str(manifest.get("subsample_prefix", "0"))

    levels_to_run = [args.level] if args.level is not None else [None]
    if args.all_levels:
        if not subsample_maps or not isinstance(subsample_maps, dict):
            print("No subsample_levels found in manifest for --all-levels.", file=sys.stderr)
            sys.exit(2)
        level_vals = []
        for key in subsample_maps.keys():
            try:
                level_vals.append(int(key))
            except (TypeError, ValueError):
                continue
        if not level_vals:
            print("No numeric subsample levels found in manifest.", file=sys.stderr)
            sys.exit(2)
        levels_to_run = sorted(set(level_vals))

    def resolve_level_map(level):
        if level is None:
            return base_map_path
        entry = None
        if isinstance(subsample_maps, dict):
            entry = subsample_maps.get(str(level))
            if entry is None:
                entry = subsample_maps.get(level)
        if entry is None:
            print(f"Level {level} not found in manifest.", file=sys.stderr)
            sys.exit(2)
        path = Path(str(entry))
        if not path.is_absolute():
            path = root / path
        if not path.exists():
            print(f"Map not found: {path}", file=sys.stderr)
            sys.exit(2)
        return path

    ts = time.strftime("%Y%m%d_%H%M%S")
    label = f"_{args.label}" if args.label else ""
    run_dir = root / "Results" / "monte_carlo" / f"{ts}{label}"
    out_dir = run_dir / "data"
    bf_cache_dir = run_dir / "bf_cache"
    out_dir.mkdir(parents=True, exist_ok=True)
    bf_cache_dir.mkdir(parents=True, exist_ok=True)

    if args.build:
        build_dir.mkdir(parents=True, exist_ok=True)
        if args.cmake or not (build_dir / "Makefile").exists():
            run_cmd(["cmake", f"-DUSE_MANIFOLD_BACKUP={cmake_define}", ".."], cwd=str(build_dir))
        run_cmd(["make", "-j"], cwd=str(build_dir))

    if not bin_path.exists():
        print(f"Binary not found: {bin_path}", file=sys.stderr)
        print("Run with --build or build manually first.", file=sys.stderr)
        sys.exit(1)

    cmd = [str(bin_path), str(args.levels), str(args.import_map), str(args.clusters)]
    if pose_name:
        cmd.append(pose_name)
    move_outputs = args.move and not args.all_levels
    all_copied = []
    seeded_all = []
    signatures_all = []
    map_paths_run = []

    for level in levels_to_run:
        level_map_path = resolve_level_map(level)
        map_paths_run.append((level, level_map_path))

        # One folder per subsample level so outputs never overwrite between levels.
        use_level_layout = level is not None
        this_out_dir = (out_dir / f"level_{level}") if use_level_layout else out_dir
        this_bf_cache_dir = bf_cache_dir
        this_out_dir.mkdir(parents=True, exist_ok=True)
        this_bf_cache_dir.mkdir(parents=True, exist_ok=True)

        map_root = root / "Map"
        try:
            rel_map = level_map_path.relative_to(map_root)
            map_name = f"../../Map/{rel_map.as_posix()}"
        except ValueError:
            map_name = str(level_map_path)
        map_output = level_map_path.name

        signature = build_bf_cache_signature(map_name, pose_name, grid, bounds, args.bf_objective)
        alt_signature = build_bf_cache_signature(
            str(level_map_path.resolve()),
            str(pose_path.resolve()),
            grid,
            bounds,
            args.bf_objective,
        )
        signatures = [signature]
        if alt_signature != signature:
            signatures.append(alt_signature)
        for sig in signatures:
            if sig not in signatures_all:
                signatures_all.append(sig)
        seeded = seed_bf_cache(root, signatures, this_bf_cache_dir)
        seeded_all.extend(seeded)

        # C++ Data/ and BF cache use the same level prefix (FOV_MONTE_CARLO_PREFIX, FOV_BF_CACHE_FILE).
        source_prefix = "0_1_"
        if level is not None:
            target_prefix = f"{subsample_prefix}_{level}_"
        else:
            target_prefix = None
        if target_prefix:
            bf_cache_basename = f"{target_prefix}bf_cache.csv"
        else:
            bf_cache_basename = f"{source_prefix}bf_cache.csv"
        cache_file = this_bf_cache_dir / bf_cache_basename

        if cache_file.exists():
            print(
                f"[cache] level {level if level is not None else 'base'}: "
                f"using {cache_file.relative_to(run_dir)}"
            )
        if seeded:
            print(f"[cache] level {level if level is not None else 'base'}: seeded {len(seeded)} cache file(s) from prior runs")
        if cache_file.exists():
            header = read_cache_signature_line(cache_file)
            expected = {f"# signature:{sig}" for sig in signatures}
            if header not in expected:
                print(
                    f"[cache] level {level if level is not None else 'base'}: "
                    "removed stale cache due to signature mismatch"
                )
                cache_file.unlink()
            else:
                print(f"[cache] level {level if level is not None else 'base'}: cache signature match")

        before = {p: p.stat().st_mtime for p in data_dir.glob("*") if p.is_file()}
        before_lines = {}
        for path in data_dir.glob("*quiversforonepoint.csv"):
            with path.open("r", encoding="utf-8") as f:
                before_lines[path.name] = sum(1 for _ in f)
        start_time = time.time()

        env = os.environ.copy()
        env["FOV_BF_CACHE_DIR"] = str(this_bf_cache_dir)
        env["FOV_BF_CACHE_FILE"] = bf_cache_basename
        env["FOV_MONTE_CARLO_PREFIX"] = target_prefix if target_prefix else source_prefix
        env["FOV_BF_OBJECTIVE"] = args.bf_objective
        env["FOV_MINIMAL_LOG"] = "1"
        env["FOV_START_STRATEGY"] = args.start_strategy
        env["FOV_START_COUNT"] = str(max(1, args.start_count))
        env["FOV_START_MIN_REL_GAIN"] = str(args.start_min_rel_gain)
        env["FOV_START_ADAPTIVE_MIN_STARTS"] = str(max(1, args.start_adaptive_min_starts))
        env["FOV_START_ADAPTIVE_PATIENCE"] = str(max(1, args.start_adaptive_patience))
        env["FOV_START_INCLUDE_BF_SEED"] = "0" if args.no_bf_start_seed else "1"
        if map_name is not None:
            env["FOV_MAP_PATH"] = map_name
        if map_output is not None:
            env["FOV_MAP_OUTPUT"] = map_output
        if bounds is not None:
            env["FOV_BOUNDS"] = ",".join(str(x) for x in bounds)
        run_cmd(cmd, cwd=str(build_dir), env=env)

        if level is not None and cache_file.exists():
            warn_if_bf_cache_level_mismatch(cache_file, subsample_prefix, level)

        changed = []
        for p in data_dir.glob("*"):
            if not p.is_file():
                continue
            mtime = p.stat().st_mtime
            if mtime >= start_time or p not in before or mtime != before[p]:
                changed.append(p)

        source_prefix = "0_1_"

        for p in changed:
            if p.name.endswith("quiversforonepoint.csv") and use_level_layout:
                dest_name = "quiversforonepoint.csv"
                dest = this_out_dir / dest_name
                dest.parent.mkdir(parents=True, exist_ok=True)
                with p.open("r", encoding="utf-8", errors="ignore") as src:
                    lines = src.readlines()
                tail = lines[before_lines.get(p.name, 0):]
                if tail and any(ch.isalpha() for ch in tail[0]):
                    tail = tail[1:]
                with dest.open("w", encoding="utf-8") as out_f:
                    out_f.writelines(tail)
                all_copied.append(str(dest.relative_to(run_dir)))
                continue

            if use_level_layout:
                # C++ already prefixes filenames with FOV_MONTE_CARLO_PREFIX (e.g. 0_3_).
                dest_name = p.name
            else:
                dest_name = p.name
                if target_prefix:
                    if dest_name.startswith(source_prefix):
                        dest_name = target_prefix + dest_name[len(source_prefix) :]
                    else:
                        dest_name = target_prefix + dest_name
            dest = this_out_dir / dest_name
            dest.parent.mkdir(parents=True, exist_ok=True)
            if move_outputs:
                shutil.move(str(p), str(dest))
            else:
                shutil.copy2(str(p), str(dest))
            all_copied.append(str(dest.relative_to(run_dir)))

        if use_level_layout:
            level_info_path = this_out_dir / "level_info.txt"
            with level_info_path.open("w", encoding="utf-8") as lf:
                lf.write(f"level: {level}\n")
                lf.write(f"map_path: {level_map_path.resolve()}\n")
                if manifest_path is not None:
                    lf.write(f"map_manifest: {manifest_path.resolve()}\n")
                if pose_path is not None:
                    lf.write(f"pose_path: {pose_path.resolve()}\n")
                lf.write(f"subsample_prefix: {subsample_prefix}\n")
                lf.write(f"monte_carlo_csv_prefix: {target_prefix}\n")
                lf.write(
                    "plot_outputs_hint: run plot_monte_carlo_results.py --run-dir <this run>; "
                    "see plots/level_<k>/comparison_visibility.png per level.\n"
                )

    info_path = run_dir / "run_info.txt"
    with info_path.open("w", encoding="utf-8") as f:
        f.write("command: manifold_test\n")
        f.write(f"levels: {args.levels}\n")
        f.write(f"import_map: {args.import_map}\n")
        f.write(f"clusters: {args.clusters}\n")
        f.write(f"all_levels: {str(args.all_levels).lower()}\n")
        f.write(f"map_path: {base_map_path}\n")
        if manifest_path is not None:
            f.write(f"map_manifest: {manifest_path}\n")
        if args.all_levels:
            levels_str = ", ".join(str(l) for l in levels_to_run if l is not None)
            f.write(f"levels_run: [{levels_str}]\n")
            f.write("map_paths:\n")
            for level, path in map_paths_run:
                label = "base" if level is None else str(level)
                f.write(f"- {label}: {path}\n")
        if pose_path is not None:
            f.write(f"pose_path: {pose_path}\n")
        f.write(f"bf_cache_dir: {bf_cache_dir}\n")
        f.write(f"bf_objective: {args.bf_objective}\n")
        f.write(f"start_strategy: {args.start_strategy}\n")
        f.write(f"start_count: {max(1, args.start_count)}\n")
        f.write(f"start_min_rel_gain: {args.start_min_rel_gain}\n")
        f.write(f"start_adaptive_min_starts: {max(1, args.start_adaptive_min_starts)}\n")
        f.write(f"start_adaptive_patience: {max(1, args.start_adaptive_patience)}\n")
        f.write(f"start_include_bf_seed: {str(not args.no_bf_start_seed).lower()}\n")
        if signatures_all:
            if len(signatures_all) == 1:
                f.write(f"bf_cache_signature: {signatures_all[0]}\n")
            else:
                f.write("bf_cache_signatures:\n")
                for sig in signatures_all:
                    f.write(f"- {sig}\n")
        if seeded_all:
            f.write("bf_cache_seeded_from:\n")
            for path in seeded_all:
                f.write(f"- {path}\n")
        if bounds is not None:
            f.write(f"bounds: {','.join(str(x) for x in bounds)}\n")
        if grid is not None:
            f.write(f"grid: {grid[0]},{grid[1]},{grid[2]}\n")
        f.write(f"binary: {bin_path}\n")
        f.write(f"data_dir: {data_dir}\n")
        f.write(f"output_dir: {out_dir}\n")
        if any(lev is not None for lev in levels_to_run):
            f.write("data_layout: per_level (Monte Carlo CSVs under data/level_<k>/; level_info.txt per folder)\n")
            f.write(
                "plots_after_run: python scripts/plot_monte_carlo_results.py --run-dir "
                f"{run_dir}  -> run-level comparison_*.png + plots/level_<k>/ per subsample\n"
            )
        prev_lines_val = 0 if args.all_levels else before_lines.get("quiversforonepoint.csv", 0)
        f.write(f"quiversforonepoint_prev_lines: {prev_lines_val}\n")
        f.write("files:\n")
        for name in sorted(set(all_copied)):
            f.write(f"- {name}\n")

    print(f"Run outputs: {out_dir}")


if __name__ == "__main__":
    main()
