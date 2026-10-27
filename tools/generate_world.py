#!/usr/bin/env python3
"""
Randomized World Generator for Active Inspection Rover
Generates Gazebo Harmonic SDF world + ground_truth.yaml

Usage:
  python3 tools/generate_world.py --seed 42 --num-defects 6 --num-distractors 2 --camera-noise 0.007 \
    --output-sdf src/rover_gazebo/worlds/inspection_facility_seed42.sdf
"""

import os
import sys
import math
import random
import argparse
import subprocess
import yaml

BASE_WORLD_HEADER = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Essential Gazebo Harmonic system plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>30 30</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>30 30</size></plane>
          </geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.75 0.75 0.75 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ========================================================================= -->
    <!-- Facility Walls (Closed loop: Room A, Room B, Corridor, Narrow Doorway)    -->
    <!-- Outer bounds: X: [-5.0, 5.0], Y: [-4.0, 4.0], Wall Height: 1.5m           -->
    <!-- ========================================================================= -->

    <!-- Outer North Wall -->
    <model name="wall_outer_north">
      <static>true</static>
      <pose>0 4.0 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>10.0 0.2 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>10.0 0.2 1.5</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Outer South Wall -->
    <model name="wall_outer_south">
      <static>true</static>
      <pose>0 -4.0 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>10.0 0.2 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>10.0 0.2 1.5</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Outer West Wall -->
    <model name="wall_outer_west">
      <static>true</static>
      <pose>-5.0 0 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.2 8.0 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.2 8.0 1.5</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Outer East Wall -->
    <model name="wall_outer_east">
      <static>true</static>
      <pose>5.0 0 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.2 8.0 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.2 8.0 1.5</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Interior Dividing Wall North (Leaves Doorway at Y in [-0.4, 0.4]) -->
    <model name="wall_inner_north">
      <static>true</static>
      <pose>0 2.2 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.2 3.6 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.2 3.6 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Interior Dividing Wall South (Forms Corridor loop) -->
    <model name="wall_inner_south">
      <static>true</static>
      <pose>0 -1.6 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.2 2.4 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.2 2.4 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Room A North Partition -->
    <model name="wall_room_a_sub">
      <static>true</static>
      <pose>-2.5 0.5 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>3.0 0.2 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>3.0 0.2 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Corridor Separator Wall (creates loop through south) -->
    <model name="wall_corridor_div">
      <static>true</static>
      <pose>2.5 -2.0 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>4.8 0.2 1.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>4.8 0.2 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- ========================================================================= -->
    <!-- Obstacles: Pipes, Shelves, Dynamic Box                                    -->
    <!-- ========================================================================= -->

    <!-- Vertical Pipe 1 -->
    <model name="pipe_1">
      <static>true</static>
      <pose>2.0 2.5 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.12</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="v">
          <geometry><cylinder><radius>0.12</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Vertical Pipe 2 -->
    <model name="pipe_2">
      <static>true</static>
      <pose>2.6 2.5 0.75 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.12</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="v">
          <geometry><cylinder><radius>0.12</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Shelf Rack -->
    <model name="shelf_rack">
      <static>true</static>
      <pose>3.5 0.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.6 1.8 1.0</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.6 1.8 1.0</size></box></geometry>
          <material><ambient>0.2 0.4 0.5 1</ambient><diffuse>0.2 0.4 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Movable Box (Dynamic Obstacle test) -->
    <model name="movable_box">
      <static>false</static>
      <pose>0.0 -3.0 0.25 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>5.0</mass>
          <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia>
        </inertial>
        <collision name="c"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

BASE_WORLD_FOOTER = """
  </world>
</sdf>
"""


def get_box_size_for_surface(surface: str, defect_type: str):
    """
    Computes an appropriate visual box size depending on the surface orientation.
    """
    if surface == "floor":
        if defect_type == "leak":
            return "0.35 0.35 0.015"
        return "0.25 0.25 0.015"
    elif "north" in surface or "south" in surface or "sub" in surface or "div" in surface:
        # Wall aligned with X-axis; surface normal points along Y
        if defect_type == "leak":
            return "0.30 0.04 0.25"
        return "0.25 0.04 0.25"
    else:
        # Wall aligned with Y-axis; surface normal points along X
        if defect_type == "leak":
            return "0.04 0.30 0.25"
        return "0.04 0.25 0.25"


def generate_defect_model_sdf(name: str, defect_type: str, pos, rpy, surface: str, aruco_id: int = 1) -> str:
    p_str = f"{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f} {rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f}"
    box_size = get_box_size_for_surface(surface, defect_type)

    if defect_type == "rust":
        return f"""
    <!-- Defect: {name} (Rust on {surface}) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{box_size}</size></box></geometry>
          <material>
            <ambient>0.9 0.1 0.1 1.0</ambient>
            <diffuse>0.9 0.1 0.1 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    elif defect_type == "leak":
        return f"""
    <!-- Defect: {name} (Leak on {surface}) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{box_size}</size></box></geometry>
          <material>
            <ambient>0.95 0.90 0.05 1.0</ambient>
            <diffuse>0.95 0.90 0.05 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    elif defect_type == "aruco":
        return f"""
    <!-- Defect: {name} (ArUco ID {aruco_id} on {surface}) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <visual name="border">
          <geometry><box><size>0.02 0.22 0.22</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="marker_pattern">
          <pose>0.012 0 0 0 0 0</pose>
          <geometry><box><size>0.005 0.16 0.16</size></box></geometry>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
        </visual>
      </link>
    </model>"""
    else:
        raise ValueError(f"Unknown defect type: {defect_type}")


def generate_distractor_model_sdf(name: str, distractor_type: str, pos, rpy, surface: str) -> str:
    # Adjust position slightly away from wall/floor so physical collision doesn't penetrate
    z_pos = max(0.20, pos[2])
    p_str = f"{pos[0]:.3f} {pos[1]:.3f} {z_pos:.3f} {rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f}"

    if distractor_type == "red_barrel":
        # Red cylinder
        return f"""
    <!-- Distractor: {name} (Red Barrel - Non-Defect) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <collision name="c">
          <geometry><cylinder><radius>0.15</radius><length>0.40</length></cylinder></geometry>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.15</radius><length>0.40</length></cylinder></geometry>
          <material>
            <ambient>0.85 0.15 0.15 1.0</ambient>
            <diffuse>0.85 0.15 0.15 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    elif distractor_type == "yellow_warning_sign":
        # Yellow rectangular warning sign
        return f"""
    <!-- Distractor: {name} (Yellow Warning Sign - Non-Defect) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>0.06 0.22 0.35</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>0.06 0.22 0.35</size></box></geometry>
          <material>
            <ambient>0.92 0.88 0.05 1.0</ambient>
            <diffuse>0.92 0.88 0.05 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    elif distractor_type == "yellow_bucket":
        # Yellow cylindrical bucket
        return f"""
    <!-- Distractor: {name} (Yellow Bucket - Non-Defect) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <collision name="c">
          <geometry><cylinder><radius>0.12</radius><length>0.25</length></cylinder></geometry>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.12</radius><length>0.25</length></cylinder></geometry>
          <material>
            <ambient>0.94 0.85 0.10 1.0</ambient>
            <diffuse>0.94 0.85 0.10 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    elif distractor_type == "red_toolbox":
        # Small red rectangular tool container
        return f"""
    <!-- Distractor: {name} (Red Toolbox - Non-Defect) -->
    <model name="{name}">
      <static>true</static>
      <pose>{p_str}</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>0.20 0.30 0.18</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>0.20 0.30 0.18</size></box></geometry>
          <material>
            <ambient>0.88 0.12 0.12 1.0</ambient>
            <diffuse>0.88 0.12 0.12 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
    else:
        raise ValueError(f"Unknown distractor type: {distractor_type}")


DISTRACTOR_TYPES = ["red_barrel", "yellow_warning_sign", "yellow_bucket", "red_toolbox"]
DEFECT_TYPES = ["rust", "leak", "aruco"]


def main():
    parser = argparse.ArgumentParser(description="Deterministic Randomized World Generator for Active Inspection")
    parser.add_argument("--seed", type=int, default=42, help="Random generator seed")
    parser.add_argument("--num-defects", type=int, default=6, help="Number of true defects to place")
    parser.add_argument("--num-distractors", type=int, default=0, help="Number of distractor objects to place")
    parser.add_argument("--camera-noise", type=float, default=0.007, help="Camera sensor Gaussian noise stddev")
    parser.add_argument("--slots-file", type=str, default="src/rover_gazebo/config/defect_slots.yaml",
                        help="Path to defect_slots.yaml")
    parser.add_argument("--output-sdf", type=str, default=None,
                        help="Path for generated SDF world")
    parser.add_argument("--output-gt", type=str, default=None,
                        help="Path for generated ground truth YAML (defaults to adjacent ground_truth.yaml)")
    parser.add_argument("--validate", action="store_true", default=True,
                        help="Run gz sdf -k on the output world")

    args = parser.parse_args()

    # Seed Python PRNG for strict determinism
    rng = random.Random(args.seed)

    # Resolve paths
    slots_path = os.path.abspath(args.slots_file)
    if not os.path.isfile(slots_path):
        print(f"Error: slots file not found: {slots_path}", file=sys.stderr)
        sys.exit(1)

    with open(slots_path, "r") as f:
        slots_data = yaml.safe_load(f)
    slots = slots_data.get("slots", [])

    total_requested = args.num_defects + args.num_distractors
    if total_requested > len(slots):
        print(f"Error: Requested {total_requested} items ({args.num_defects} defects + {args.num_distractors} distractors) "
              f"but only {len(slots)} slots available.", file=sys.stderr)
        sys.exit(1)

    # Determine output file paths
    if args.output_sdf is None:
        world_name = f"inspection_facility_seed{args.seed}"
        output_sdf = os.path.abspath(f"src/rover_gazebo/worlds/{world_name}.sdf")
    else:
        output_sdf = os.path.abspath(args.output_sdf)
        world_name = os.path.splitext(os.path.basename(output_sdf))[0]

    if args.output_gt is None:
        output_gt = os.path.join(os.path.dirname(output_sdf), f"{world_name}_ground_truth.yaml")
    else:
        output_gt = os.path.abspath(args.output_gt)

    os.makedirs(os.path.dirname(output_sdf), exist_ok=True)
    os.makedirs(os.path.dirname(output_gt), exist_ok=True)

    # Deterministically shuffle and partition slots
    shuffled_slots = list(slots)
    rng.shuffle(shuffled_slots)

    defect_slots = shuffled_slots[:args.num_defects]
    distractor_slots = shuffled_slots[args.num_defects:total_requested]

    # Generate real defects
    defects_gt = []
    models_sdf_parts = []

    type_counts = {"rust": 0, "leak": 0, "aruco": 0}
    # Ensure balanced defect distribution across types
    defect_type_cycle = DEFECT_TYPES * ((args.num_defects // len(DEFECT_TYPES)) + 1)
    rng.shuffle(defect_type_cycle)

    for i, slot in enumerate(defect_slots):
        slot_affinity = slot.get("type_affinity", DEFECT_TYPES)
        # Select type matching slot affinity
        chosen_type = defect_type_cycle[i % len(defect_type_cycle)]
        if chosen_type not in slot_affinity:
            # Fall back to first matching affinity
            valid_affinities = [t for t in slot_affinity if t in DEFECT_TYPES]
            chosen_type = valid_affinities[0] if valid_affinities else "rust"

        type_counts[chosen_type] += 1
        model_name = f"defect_{chosen_type}_{type_counts[chosen_type]}"
        pos = slot["position"]
        rpy = slot["orientation"]
        surface = slot["surface"]

        aruco_id = type_counts["aruco"] if chosen_type == "aruco" else None
        sdf_snippet = generate_defect_model_sdf(model_name, chosen_type, pos, rpy, surface, aruco_id or 1)
        models_sdf_parts.append(sdf_snippet)

        gt_record = {
            "id": i + 1,
            "name": model_name,
            "type": chosen_type,
            "is_defect": True,
            "slot_id": slot["id"],
            "surface": surface,
            "position": [round(float(c), 3) for c in pos],
            "orientation": [round(float(c), 3) for c in rpy]
        }
        if aruco_id is not None:
            gt_record["aruco_id"] = aruco_id
        defects_gt.append(gt_record)

    # Generate distractors
    distractors_gt = []
    for j, slot in enumerate(distractor_slots):
        distractor_type = rng.choice(DISTRACTOR_TYPES)
        model_name = f"distractor_{j + 1}_{distractor_type}"
        pos = slot["position"]
        rpy = slot["orientation"]
        surface = slot["surface"]

        sdf_snippet = generate_distractor_model_sdf(model_name, distractor_type, pos, rpy, surface)
        models_sdf_parts.append(sdf_snippet)

        color = "red" if "red" in distractor_type else "yellow"
        distractors_gt.append({
            "id": j + 1,
            "name": model_name,
            "category": distractor_type,
            "color": color,
            "is_defect": False,
            "slot_id": slot["id"],
            "surface": surface,
            "position": [round(float(c), 3) for c in pos],
            "orientation": [round(float(c), 3) for c in rpy]
        })

    # Assemble full SDF
    full_sdf = (
        BASE_WORLD_HEADER.format(world_name=world_name)
        + "\n    <!-- ========================================================================= -->"
        + "\n    <!-- GENERATED DEFECTS & DISTRACTORS (Seed: " + str(args.seed) + ") -->"
        + "\n    <!-- ========================================================================= -->\n"
        + "\n".join(models_sdf_parts)
        + "\n"
        + BASE_WORLD_FOOTER
    )

    with open(output_sdf, "w") as f:
        f.write(full_sdf)

    # Assemble ground truth YAML
    gt_data = {
        "metadata": {
            "world_name": world_name,
            "seed": args.seed,
            "num_defects": len(defects_gt),
            "num_distractors": len(distractors_gt),
            "camera_noise": float(args.camera_noise),
            "sdf_file": os.path.basename(output_sdf)
        },
        "defects": defects_gt,
        "distractors": distractors_gt
    }

    with open(output_gt, "w") as f:
        yaml.dump(gt_data, f, sort_keys=False, default_flow_style=False)

    print(f"Generated world SDF: {output_sdf}")
    print(f"Generated ground truth: {output_gt}")
    print(f"  Defects placed: {len(defects_gt)} ({type_counts})")
    print(f"  Distractors placed: {len(distractors_gt)}")
    print(f"  Camera noise stddev: {args.camera_noise}")

    # Validate with gz sdf -k
    if args.validate:
        res = subprocess.run(["gz", "sdf", "-k", output_sdf], capture_output=True, text=True)
        if res.returncode == 0:
            print(f"Validation: {res.stdout.strip() or 'Valid.'}")
        else:
            print(f"Validation ERROR:\n{res.stderr}", file=sys.stderr)
            sys.exit(res.returncode)


if __name__ == "__main__":
    main()
