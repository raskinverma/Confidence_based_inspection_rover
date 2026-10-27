#!/usr/bin/env python3
"""
Batch Runner for Inspection Rover (tools/run_batch.py)
Automates end-to-end evaluation runs across seeds and conditions:
  1. Generates world & ground truth using tools/generate_world.py
  2. Launches the full simulation stack headless with process group isolation
  3. Monitors /mission_status or run_telemetry.yaml for completion (DONE/FAILED/TIMEOUT)
  4. Robustly terminates all Gazebo and ROS2 processes
  5. Evaluates defect detections with tools/rover_eval.py
  6. Appends comprehensive run metrics to results/runs.csv
"""

import os
import sys
import time
import signal
import subprocess
import argparse
import yaml
import csv

# Import evaluation module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from rover_eval import evaluate_run


CONDITIONS = {
    'clean': {
        'num_distractors': 0,
        'camera_noise': 0.007
    },
    'hard': {
        'num_distractors': 4,
        'camera_noise': 0.02
    }
}


def kill_all_sim_processes():
    """Aggressively kills any stray gz, ros2, or nav2 processes."""
    try:
        subprocess.run(['pkill', '-9', '-f', 'gz sim|ros2|nav2|mission_manager|defect_detector|pose_projector'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
    except Exception:
        pass


def generate_world(seed: int, condition: str, num_distractors: int, camera_noise: float, workspace_root: str):
    world_name = f"run_{condition}_seed{seed}"
    output_dir = os.path.join(workspace_root, 'src', 'rover_gazebo', 'worlds')
    gen_script = os.path.join(workspace_root, 'tools', 'generate_world.py')

    sdf_path = os.path.join(output_dir, f"{world_name}.sdf")
    gt_path = os.path.join(output_dir, f"{world_name}_ground_truth.yaml")

    cmd = [
        sys.executable, gen_script,
        '--seed', str(seed),
        '--num-defects', '6',
        '--num-distractors', str(num_distractors),
        '--camera-noise', str(camera_noise),
        '--output-sdf', sdf_path,
        '--output-gt', gt_path,
        '--validate'
    ]
    print(f"[{world_name}] Generating world (seed={seed}, distractors={num_distractors}, noise={camera_noise})...")
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"World generation failed: {res.stderr}")

    return sdf_path, gt_path


def run_single_simulation(method: str, seed: int, condition: str, sdf_path: str, gt_path: str,
                          workspace_root: str, timeout_sec: int = 360):
    run_id = f"{method}_{condition}_seed{seed}"
    run_output_dir = os.path.join(workspace_root, 'results', run_id)
    os.makedirs(run_output_dir, exist_ok=True)

    defects_csv = os.path.join(run_output_dir, 'inspection_defects.csv')
    telemetry_yaml = os.path.join(run_output_dir, 'run_telemetry.yaml')

    # Remove any stale outputs from previous runs
    for f in (defects_csv, telemetry_yaml):
        if os.path.exists(f):
            os.remove(f)

    # Launch command
    launch_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {os.path.join(workspace_root, 'install', 'setup.bash')} && "
        f"ros2 launch rover_bringup full_sim.launch.py "
        f"mode:=nav run_perception:=true run_mission:=true "
        f"world:={sdf_path} output_dir:={run_output_dir} "
        f"headless:=true use_rviz:=false"
    )

    log_file_path = os.path.join(run_output_dir, 'sim.log')
    print(f"[{run_id}] Launching simulation stack (headless=True, timeout={timeout_sec}s)...")
    start_wall_time = time.time()

    with open(log_file_path, 'w') as log_file:
        proc = subprocess.Popen(
            launch_cmd,
            shell=True,
            executable='/bin/bash',
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid  # New process group for clean teardown
        )

    mission_status = "RUNNING"
    try:
        poll_interval = 2.0
        while time.time() - start_wall_time < timeout_sec:
            # Check if telemetry file is written with status DONE or FAILED
            if os.path.exists(telemetry_yaml):
                try:
                    with open(telemetry_yaml, 'r') as tf:
                        tel_data = yaml.safe_load(tf) or {}
                        if tel_data.get('status') in ('DONE', 'FAILED'):
                            mission_status = tel_data.get('status')
                            print(f"[{run_id}] Mission completed with status: {mission_status}")
                            break
                except Exception:
                    pass

            # Check if process unexpectedly died
            if proc.poll() is not None:
                mission_status = "PROCESS_DIED"
                print(f"[{run_id}] Process unexpectedly terminated early (exit code {proc.returncode})")
                break

            time.sleep(poll_interval)
        else:
            mission_status = "TIMEOUT"
            print(f"[{run_id}] Run timed out after {timeout_sec}s")
    finally:
        # Tear down process group
        print(f"[{run_id}] Terminating simulation process group...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            time.sleep(3.0)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        kill_all_sim_processes()

    wall_time = round(time.time() - start_wall_time, 2)
    print(f"[{run_id}] Simulation finished. Wall time: {wall_time}s, Status: {mission_status}")

    # If telemetry wasn't saved by mission_manager (e.g. timeout), write fallback telemetry
    if not os.path.exists(telemetry_yaml):
        with open(telemetry_yaml, 'w') as tf:
            yaml.safe_dump({
                'status': mission_status,
                'mission_duration_sec': wall_time,
                'path_length_m': 0.0,
                'min_clearance_m': 0.0,
                'waypoints_completed': 0,
                'waypoints_total': 7
            }, tf)

    # Run evaluation
    metrics = evaluate_run(gt_path, defects_csv, telemetry_yaml)
    metrics['method'] = method
    metrics['condition'] = condition
    metrics['wall_time_sec'] = wall_time
    metrics['run_id'] = run_id

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Batch Runner for Inspection Rover")
    parser.add_argument('--method', default='baseline', help="Inspection method name")
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 2, 3, 4, 5], help="List of seeds to run")
    parser.add_argument('--conditions', nargs='+', default=['clean', 'hard'], choices=['clean', 'hard'],
                        help="Conditions to run (clean, hard)")
    parser.add_argument('--timeout', type=int, default=320, help="Per-run timeout in seconds")
    parser.add_argument('--output-csv', default='results/runs.csv', help="CSV file to append results")
    args = parser.parse_args()

    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    csv_path = os.path.join(workspace_root, args.output_csv)
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    # Initial safety kill
    kill_all_sim_processes()

    runs_to_execute = []
    for cond in args.conditions:
        for seed in args.seeds:
            runs_to_execute.append((cond, seed))

    print(f"=== Starting Batch Run: {len(runs_to_execute)} runs scheduled for method '{args.method}' ===")
    all_metrics = []

    for cond, seed in runs_to_execute:
        params = CONDITIONS[cond]
        sdf_path, gt_path = generate_world(
            seed=seed,
            condition=cond,
            num_distractors=params['num_distractors'],
            camera_noise=params['camera_noise'],
            workspace_root=workspace_root
        )

        metrics = run_single_simulation(
            method=args.method,
            seed=seed,
            condition=cond,
            sdf_path=sdf_path,
            gt_path=gt_path,
            workspace_root=workspace_root,
            timeout_sec=args.timeout
        )
        all_metrics.append(metrics)

        # Append to CSV immediately
        file_exists = os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(metrics.keys()))
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)
        print(f"Saved run results to {csv_path}\n")

    print(f"\n🎉 All {len(runs_to_execute)} batch runs completed successfully!")


if __name__ == '__main__':
    main()
