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


def build_bf_cache_signature(map_name, pose_name, grid=None, bounds=None):
    if pose_name:
        return f"map={map_name};pose={pose_name}"
    if grid is None or bounds is None:
        return f"map={map_name}"
    x_res, y_res, z_res = grid
    x_low, x_high, y_low, y_high, z_low, z_high = bounds
    return (
        f"map={map_name};grid={x_res},{y_res},{z_res};bounds="
        f"{x_low},{x_high},{y_low},{y_high},{z_low},{z_high}"
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
            copied.append(dest)
    return copied


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
        "--pose-file",
        default=None,
        help="Pose CSV file to use (relative to Map/ or absolute). Overrides manifest pose_map.",
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
            print("Manifest did not specify a base_map.", file=sys.stderr)
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
        map_path = Path(args.map)
        if not map_path.is_absolute():
            map_path = root / "Map" / args.map
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
        pose_path = Path(args.pose_file)
        if not pose_path.is_absolute():
            pose_path = root / "Map" / args.pose_file
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

    map_changed = False
    if map_name is not None or map_output is not None or bounds is not None:
        map_changed = patch_map_settings(
            manifold_test_path,
            map_name=map_name,
            map_output=map_output,
            bounds=bounds,
            mode="import",
        )
        if map_changed and not args.build:
            print("Map settings changed; enabling build.")
            args.build = True

    if args.build:
        build_dir.mkdir(parents=True, exist_ok=True)
        if args.cmake or not (build_dir / "Makefile").exists():
            run_cmd(["cmake", f"-DUSE_MANIFOLD_BACKUP={cmake_define}", ".."], cwd=str(build_dir))
        run_cmd(["make", "-j"], cwd=str(build_dir))

    if not bin_path.exists():
        print(f"Binary not found: {bin_path}", file=sys.stderr)
        print("Run with --build or build manually first.", file=sys.stderr)
        sys.exit(1)

    ts = time.strftime("%Y%m%d_%H%M%S")
    label = f"_{args.label}" if args.label else ""
    run_dir = root / "Results" / "monte_carlo" / f"{ts}{label}"
    out_dir = run_dir / "data"
    bf_cache_dir = run_dir / "bf_cache"
    out_dir.mkdir(parents=True, exist_ok=True)
    bf_cache_dir.mkdir(parents=True, exist_ok=True)

    signature = build_bf_cache_signature(map_name, pose_name, grid, bounds)
    alt_signature = build_bf_cache_signature(str(map_path.resolve()), str(pose_path.resolve()), grid, bounds)
    signatures = [signature]
    if alt_signature != signature:
        signatures.append(alt_signature)
    seeded = seed_bf_cache(root, signatures, bf_cache_dir)

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

    cmd = [str(bin_path), str(args.levels), str(args.import_map), str(args.clusters)]
    if pose_name:
        cmd.append(pose_name)
    env = os.environ.copy()
    env["FOV_BF_CACHE_DIR"] = str(bf_cache_dir)
    env["FOV_MINIMAL_LOG"] = "1"
    run_cmd(cmd, cwd=str(build_dir), env=env)

    changed = []
    for p in data_dir.glob("*"):
        if not p.is_file():
            continue
        mtime = p.stat().st_mtime
        if mtime >= start_time or p not in before or mtime != before[p]:
            changed.append(p)

    for p in changed:
        dest = out_dir / p.name
        dest.parent.mkdir(parents=True, exist_ok=True)
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
        f.write(f"map_path: {map_path}\n")
        if manifest_path is not None:
            f.write(f"map_manifest: {manifest_path}\n")
        if pose_path is not None:
            f.write(f"pose_path: {pose_path}\n")
        f.write(f"bf_cache_dir: {bf_cache_dir}\n")
        f.write(f"bf_cache_signature: {signature}\n")
        if seeded:
            f.write("bf_cache_seeded_from:\n")
            for path in seeded:
                f.write(f"- {path}\n")
        if bounds is not None:
            f.write(f"bounds: {','.join(str(x) for x in bounds)}\n")
        if grid is not None:
            f.write(f"grid: {grid[0]},{grid[1]},{grid[2]}\n")
        f.write(f"binary: {bin_path}\n")
        f.write(f"data_dir: {data_dir}\n")
        f.write(f"output_dir: {out_dir}\n")
        f.write(f"quiversforonepoint_prev_lines: {before_lines.get('quiversforonepoint.csv', 0)}\n")
        f.write("files:\n")
        for p in sorted(changed, key=lambda x: x.name):
            f.write(f"- {p.name}\n")

    print(f"Run outputs: {out_dir}")


if __name__ == "__main__":
    main()
