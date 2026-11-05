#!/usr/bin/env python3
"""
Batch Runner for Inspection Rover (tools/run_batch.py)
Automates end-to-end evaluation runs across seeds and conditions:
  1. Process group isolation and signal handling (SIGTERM -> SIGKILL after 5s)
  2. Startup stray process gating
  3. Unique ROS_DOMAIN_ID per run
  4. Generates world & ground truth using tools/generate_world.py
  5. Monitors /mission_status or run_telemetry.yaml for completion
  6. Evaluates defect detections with tools/rover_eval.py
  7. Tracks wall time, sim time, and RTF (sim / wall)
  8. Appends comprehensive run metrics to results/runs.csv
"""

import os
import sys
import time
import signal
import subprocess
import argparse
import yaml
import csv
import psutil

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

ACTIVE_PGID = None


def find_stray_sim_processes():
    """Finds any stray simulation or ROS2/Gazebo processes."""
    stray = []
    sim_patterns = [
        'gz sim', 'gz-sim',
        'ros2 launch',
        'nav2_', 'bt_navigator', 'controller_server', 'planner_server',
        'behavior_server', 'amcl', 'lifecycle_manager',
        'mission_manager', 'defect_detector', 'pose_projector', 'twist_relay',
        'parameter_bridge', 'ros_gz_bridge',
        'robot_state_publisher'
    ]
    for p in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmdline = ' '.join(p.cmdline())
            name = p.name()
            # Ignore self, language servers, antigravity IDE tools
            if p.pid == os.getpid() or 'language_server' in cmdline or 'antigravity' in cmdline or 'run_batch.py' in cmdline:
                continue
            for pat in sim_patterns:
                if pat in name or pat in cmdline:
                    stray.append({
                        'pid': p.pid,
                        'name': name,
                        'cmdline': p.cmdline()
                    })
                    break
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return stray


def kill_process_group(pgid: int, timeout: float = 5.0):
    """Cleanly terminates a process group: SIGTERM, then SIGKILL after timeout seconds."""
    if pgid is None:
        return
    try:
        os.killpg(pgid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return
    except Exception as e:
        print(f"Error sending SIGTERM to pgid {pgid}: {e}")

    start = time.time()
    while time.time() - start < timeout:
        alive = False
        for p in psutil.process_iter(['pid']):
            try:
                if os.getpgid(p.pid) == pgid:
                    alive = True
                    break
            except (ProcessLookupError, psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        if not alive:
            return
        time.sleep(0.2)

    # Force kill with SIGKILL if still running
    try:
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    except Exception as e:
        print(f"Error sending SIGKILL to pgid {pgid}: {e}")


def kill_all_sim_processes():
    """Fallback teardown: terminates any remaining stray simulation processes."""
    stray = find_stray_sim_processes()
    for s in stray:
        try:
            p = psutil.Process(s['pid'])
            p.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    time.sleep(1.0)
    stray = find_stray_sim_processes()
    for s in stray:
        try:
            p = psutil.Process(s['pid'])
            p.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass


def signal_handler(signum, frame):
    """Catches SIGINT (Ctrl-C) and SIGTERM to ensure clean process teardown."""
    global ACTIVE_PGID
    sig_name = "SIGINT (Ctrl-C)" if signum == signal.SIGINT else "SIGTERM"
    print(f"\n[run_batch] Caught {sig_name}. Initiating clean shutdown...")
    if ACTIVE_PGID is not None:
        print(f"[run_batch] Terminating active simulation process group (PGID {ACTIVE_PGID})...")
        kill_process_group(ACTIVE_PGID, timeout=5.0)
        ACTIVE_PGID = None
    kill_all_sim_processes()
    sys.exit(130 if signum == signal.SIGINT else 143)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


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
                          workspace_root: str, timeout_sec: int = 900):
    global ACTIVE_PGID
    run_id = f"{method}_{condition}_seed{seed}"
    run_output_dir = os.path.join(workspace_root, 'results', run_id)
    os.makedirs(run_output_dir, exist_ok=True)

    defects_csv = os.path.join(run_output_dir, 'inspection_defects.csv')
    telemetry_yaml = os.path.join(run_output_dir, 'run_telemetry.yaml')

    # Remove any stale outputs from previous runs
    for f in (defects_csv, telemetry_yaml):
        if os.path.exists(f):
            os.remove(f)

    # Isolated ROS_DOMAIN_ID per run to eliminate cross-talk
    domain_id = (seed * 17 + (1 if condition == 'clean' else 2) + (os.getpid() % 50)) % 101 + 1
    run_env = os.environ.copy()
    run_env['ROS_DOMAIN_ID'] = str(domain_id)

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
    print(f"[{run_id}] Launching simulation stack (headless=True, timeout={timeout_sec}s, ROS_DOMAIN_ID={domain_id})...")
    start_wall_time = time.time()

    with open(log_file_path, 'w') as log_file:
        proc = subprocess.Popen(
            launch_cmd,
            shell=True,
            executable='/bin/bash',
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,  # New session & process group
            env=run_env
        )
        ACTIVE_PGID = proc.pid

    mission_status = "RUNNING"
    try:
        poll_interval = 2.0
        while time.time() - start_wall_time < timeout_sec:
            # Check if telemetry file is written with status DONE or FAILED
            if os.path.exists(telemetry_yaml):
                try:
                    with open(telemetry_yaml, 'r') as tf:
                        tel_data = yaml.safe_load(tf) or {}
                        if tel_data.get('status') in ('DONE', 'COMPLETED', 'FAILED'):
                            raw_st = tel_data.get('status')
                            mission_status = 'COMPLETED' if raw_st in ('DONE', 'COMPLETED') else raw_st
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
        # Tear down process group: SIGTERM, then SIGKILL after 5s
        print(f"[{run_id}] Terminating simulation process group (PGID {proc.pid})...")
        kill_process_group(proc.pid, timeout=5.0)
        ACTIVE_PGID = None

        # Verify no processes from this run or simulation remain
        stray = find_stray_sim_processes()
        if stray:
            print(f"[{run_id}] Cleaning up {len(stray)} residual simulation processes...")
            kill_all_sim_processes()

    wall_time = round(time.time() - start_wall_time, 2)

    # Read telemetry written by mission_manager (which flushes periodically and on exit)
    tel_data = {}
    if os.path.exists(telemetry_yaml):
        try:
            with open(telemetry_yaml, 'r') as tf:
                tel_data = yaml.safe_load(tf) or {}
        except Exception:
            pass

    sim_time = float(tel_data.get('sim_time_sec', tel_data.get('mission_duration_sec', 0.0)))
    rtf = round(sim_time / wall_time, 3) if wall_time > 0 else 0.0

    print(f"[{run_id}] Simulation finished. Wall time: {wall_time}s, Sim time: {sim_time}s (RTF: {rtf}), Status: {mission_status}")

    # If telemetry wasn't saved by mission_manager (e.g. fatal crash), write fallback telemetry
    if not os.path.exists(telemetry_yaml):
        with open(telemetry_yaml, 'w') as tf:
            yaml.safe_dump({
                'status': mission_status,
                'mission_duration_sec': sim_time,
                'sim_time_sec': sim_time,
                'path_length_m': 0.0,
                'min_clearance_m': 0.0,
                'waypoints_completed': 0,
                'waypoints_total': 7,
                'initial_loc_error_m': 0.0,
                'mean_loc_error_m': 0.0,
                'max_loc_error_m': 0.0
            }, tf)

    # Run evaluation
    metrics = evaluate_run(gt_path, defects_csv, telemetry_yaml)
    metrics['method'] = method
    metrics['condition'] = condition
    metrics['run_id'] = run_id
    metrics['sim_time_sec'] = sim_time
    metrics['wall_time_sec'] = wall_time
    metrics['rtf'] = rtf

    # Count rejected observations logged by pose_projector
    rejected_count = 0
    if os.path.exists(log_file_path):
        try:
            with open(log_file_path, 'r', errors='replace') as lf:
                for line in lf:
                    if 'Rejected observation' in line:
                        rejected_count += 1
        except Exception:
            pass
    metrics['rejected_observations'] = rejected_count

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Batch Runner for Inspection Rover")
    parser.add_argument('--method', '--methods', dest='method', default='baseline', help="Inspection method name")
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 2, 3, 4, 5], help="List of seeds to run")
    parser.add_argument('--conditions', nargs='+', default=['clean', 'hard'], choices=['clean', 'hard'],
                        help="Conditions to run (clean, hard)")
    parser.add_argument('--timeout', type=int, default=900, help="Per-run timeout in seconds (default: 900s)")
    parser.add_argument('--output-csv', default='results/runs.csv', help="CSV file to append results")
    parser.add_argument('--force-clean', action='store_true', help="Clean stray processes automatically at startup")
    args = parser.parse_args()

    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    csv_path = os.path.join(workspace_root, args.output_csv)
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    # Startup Stray Process Gating
    stray = find_stray_sim_processes()
    if stray:
        if args.force_clean:
            print(f"Startup: found {len(stray)} stray processes. Cleaning as requested by --force-clean...")
            kill_all_sim_processes()
        else:
            print("=" * 70)
            print("ERROR: Stray simulation processes detected running at startup:")
            print(f"{'PID':<8} {'NAME':<20} {'COMMAND':<40}")
            print("-" * 70)
            for p in stray:
                cmd_str = ' '.join(p['cmdline'])[:40]
                print(f"{p['pid']:<8} {p['name']:<20} {cmd_str:<40}")
            print("=" * 70)
            print("Refusing to run batch. Please terminate stray processes first or use --force-clean.")
            sys.exit(1)

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
