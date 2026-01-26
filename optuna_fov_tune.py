#!/usr/bin/env python3
import argparse
import os
import statistics
import subprocess
import time
from pathlib import Path

import optuna


STAT_INDEX = {
    "min": 0,
    "max": 1,
    "avg": 2,
    "std": 3,
}


def parse_error_stat(path: Path, stat: str):
    idx = STAT_INDEX[stat]
    with path.open("r", encoding="utf-8") as handle:
        lines = [line.strip() for line in handle.readlines() if line.strip()]
    if len(lines) < 3:
        raise RuntimeError(f"error_stat file looks short: {path}")
    t_vals = [float(v) for v in lines[1].split()]
    r_vals = [float(v) for v in lines[2].split()]
    if len(t_vals) < 4 or len(r_vals) < 4:
        raise RuntimeError(f"error_stat format unexpected: {path}")
    return t_vals[idx], r_vals[idx]


def parse_pose_errors(path: Path, stat: str):
    te_vals = []
    re_vals = []
    total = 0
    with path.open("r", encoding="utf-8") as handle:
        for line_idx, line in enumerate(handle):
            if line_idx == 0:
                continue
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            total += 1
            if parts[1] != "nan":
                te_vals.append(float(parts[1]))
            if parts[2] != "nan":
                re_vals.append(float(parts[2]))
    if not te_vals or not re_vals:
        raise RuntimeError(f"pose_errors file has no usable entries: {path}")
    return compute_stat(te_vals, stat), compute_stat(re_vals, stat)


def compute_stat(values, stat):
    if stat == "min":
        return min(values)
    if stat == "max":
        return max(values)
    if stat == "std":
        return statistics.stdev(values)
    return statistics.mean(values)


def build_search_space(trial, args):
    params = {}
    if args.tune_max_iter:
        params["max_iter"] = trial.suggest_int(
            "max_iter", args.max_iter_range[0], args.max_iter_range[1]
        )
    if args.tune_ks:
        params["ks"] = trial.suggest_float(
            "ks", args.ks_range[0], args.ks_range[1], log=args.ks_log
        )
    if args.tune_visibility:
        params["visibility_angle_deg"] = trial.suggest_float(
            "visibility_angle_deg",
            args.visibility_range[0],
            args.visibility_range[1],
        )
    if args.tune_base_step:
        params["base_step_scale"] = trial.suggest_float(
            "base_step_scale",
            args.base_step_scale_range[0],
            args.base_step_scale_range[1],
            log=args.base_step_scale_log,
        )
    if args.tune_step_limits:
        params["min_step_deg"] = trial.suggest_float(
            "min_step_deg", args.min_step_range[0], args.min_step_range[1]
        )
        params["max_step_deg"] = trial.suggest_float(
            "max_step_deg", args.max_step_range[0], args.max_step_range[1]
        )
    if args.tune_traj_jac_step:
        params["traj_jac_step"] = trial.suggest_float(
            "traj_jac_step",
            args.traj_jac_step_range[0],
            args.traj_jac_step_range[1],
        )
    return params


def make_env(params, base_env):
    env = dict(base_env)
    if "max_iter" in params:
        env["FOV_OPT_MAX_ITER"] = str(params["max_iter"])
    if "ks" in params:
        env["FOV_OPT_KS"] = f"{params['ks']:.8f}"
    if "visibility_angle_deg" in params:
        env["FOV_OPT_VIS_ANGLE_DEG"] = f"{params['visibility_angle_deg']:.8f}"
    if "base_step_scale" in params:
        env["FOV_OPT_BASE_STEP_SCALE"] = f"{params['base_step_scale']:.8f}"
    if "min_step_deg" in params:
        env["FOV_OPT_MIN_STEP_DEG"] = f"{params['min_step_deg']:.8f}"
    if "max_step_deg" in params:
        env["FOV_OPT_MAX_STEP_DEG"] = f"{params['max_step_deg']:.8f}"
    if "traj_jac_step" in params:
        env["FOV_OPT_TRAJ_JAC_STEP"] = f"{params['traj_jac_step']:.8f}"
    return env


def run_trial_command(cmd, workdir, env):
    subprocess.run(cmd, shell=True, check=True, cwd=workdir, env=env)


def main():
    parser = argparse.ArgumentParser(description="Optuna tuning for FoV optimization.")
    parser.add_argument("--run-cmd", required=True, help="Command that runs the full pipeline.")
    parser.add_argument("--workdir", default=".", help="Working directory for run-cmd.")
    parser.add_argument("--error-stat", default="", help="Path to error_stat.txt.")
    parser.add_argument("--pose-errors", default="", help="Path to pose_errors.txt.")
    parser.add_argument("--stat", default="avg", choices=STAT_INDEX.keys())
    parser.add_argument("--n-trials", type=int, default=20)
    parser.add_argument("--timeout", type=int, default=0, help="Seconds before stopping.")
    parser.add_argument("--study-name", default="fov_optuna")
    parser.add_argument("--storage", default="")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--multi-objective", action="store_true")
    parser.add_argument("--te-weight", type=float, default=1.0)
    parser.add_argument("--re-weight", type=float, default=1.0)
    parser.add_argument("--trial-dir-root", default="", help="Optional base dir for trial outputs.")
    parser.add_argument("--log-jacobian", type=int, default=0, choices=[0, 1])

    parser.add_argument("--tune-max-iter", action="store_true")
    parser.add_argument("--max-iter-range", type=int, nargs=2, default=[10, 50])
    parser.add_argument("--tune-ks", action="store_true")
    parser.add_argument("--ks-range", type=float, nargs=2, default=[5.0, 40.0])
    parser.add_argument("--ks-log", action="store_true")
    parser.add_argument("--tune-visibility", action="store_true")
    parser.add_argument("--visibility-range", type=float, nargs=2, default=[5.0, 45.0])
    parser.add_argument("--tune-base-step", action="store_true")
    parser.add_argument("--base-step-scale-range", type=float, nargs=2, default=[0.2, 2.0])
    parser.add_argument("--base-step-scale-log", action="store_true")
    parser.add_argument("--tune-step-limits", action="store_true")
    parser.add_argument("--min-step-range", type=float, nargs=2, default=[0.05, 1.0])
    parser.add_argument("--max-step-range", type=float, nargs=2, default=[0.5, 3.0])
    parser.add_argument("--tune-traj-jac-step", action="store_true")
    parser.add_argument("--traj-jac-step-range", type=float, nargs=2, default=[0.1, 0.7])

    args = parser.parse_args()

    if not args.error_stat and not args.pose_errors:
        raise SystemExit("Provide --error-stat or --pose-errors.")

    if args.storage:
        storage = args.storage
    else:
        storage = None

    if args.multi_objective:
        study = optuna.create_study(
            directions=["minimize", "minimize"],
            study_name=args.study_name,
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=args.seed if args.seed else None),
        )
    else:
        study = optuna.create_study(
            direction="minimize",
            study_name=args.study_name,
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=args.seed if args.seed else None),
        )

    workdir = Path(args.workdir).resolve()
    error_stat_path = Path(args.error_stat).resolve() if args.error_stat else None
    pose_errors_path = Path(args.pose_errors).resolve() if args.pose_errors else None
    trial_root = Path(args.trial_dir_root).resolve() if args.trial_dir_root else None

    def objective(trial):
        params = build_search_space(trial, args)
        env = make_env(params, os.environ)
        env["FOV_OPT_LOG_JACOBIAN"] = str(args.log_jacobian)

        trial_dir = None
        if trial_root:
            trial_dir = trial_root / f"trial_{trial.number:04d}"
            trial_dir.mkdir(parents=True, exist_ok=True)
            env["FOV_OPT_TRIAL_DIR"] = str(trial_dir)

        start = time.time()
        run_trial_command(args.run_cmd, workdir, env)
        elapsed = time.time() - start

        if error_stat_path and error_stat_path.exists():
            te, re = parse_error_stat(error_stat_path, args.stat)
        elif pose_errors_path and pose_errors_path.exists():
            te, re = parse_pose_errors(pose_errors_path, args.stat)
        else:
            raise RuntimeError("No error file found after run.")

        trial.set_user_attr("te", te)
        trial.set_user_attr("re", re)
        trial.set_user_attr("elapsed_sec", elapsed)
        if trial_dir:
            trial.set_user_attr("trial_dir", str(trial_dir))

        if args.multi_objective:
            return te, re
        return args.te_weight * te + args.re_weight * re

    study.optimize(objective, n_trials=args.n_trials, timeout=args.timeout or None)

    if args.multi_objective:
        best_trials = study.best_trials
        print(f"Finished {len(study.trials)} trials. Pareto front size: {len(best_trials)}")
        for t in best_trials:
            print(f"trial={t.number} te={t.values[0]:.6f} re={t.values[1]:.6f} params={t.params}")
    else:
        print(f"Finished {len(study.trials)} trials. Best value: {study.best_value:.6f}")
        print(f"Best params: {study.best_params}")


if __name__ == "__main__":
    main()
