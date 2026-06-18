# Shuttle-R

Shuttle-R is a robotics project: an autonomous robot that detects, collects,
and returns badminton shuttlecocks scattered across a court, simulated in
ROS2 Humble + Gazebo Fortress (Ignition) on a TurtleBot4.

The project builds on existing ROS2/Nav2/SLAM packages for navigation and localization,
while the shuttlecock detection, 3D localization, collection mechanics, and coordination
logic are custom nodes implemented from scratch as the core contribution of this project.

## Tech Stack
- ROS2 Humble, Gazebo Fortress (Ignition), Ubuntu 22.04
- TurtleBot4 (Standard)
- Nav2 (AMCL + NavfnPlanner + DWB controller), SLAM Toolbox (mapping)
- OpenCV (HSV-based shuttlecock detection)
- image_geometry + tf2 (pixel-to-3D deprojection)

## Package Layout (`src/shuttler_sim`)
- `worlds/empty_court.sdf` — badminton court world with boundary walls, net,
  drop-off zone (NE corner), scoop/hopper models, and 3 shuttlecock models
- `launch/empty_world.launch.py` — launches Gazebo (headless), spawns the robot,
  bridges RGB/depth/camera-info/lidar/clock topics, auto-undocks, and starts
  the detector node
- `config/nav2.yaml` — tuned Nav2 parameters (relaxed goal tolerances to fix
  final-approach oscillation)
- `maps/court_map.{pgm,yaml}` — SLAM-generated map of the court for AMCL localization

## Custom Nodes

### `shuttlecock_detector`

HSV-based color detection (H:5-20, S:120-255, V:80-255) of orange shuttlecock
skirts from the robot's RGB camera. Applies morphological open/close to clean
noise, filters contours by area (5-2000 px), and publishes
`vision_msgs/Detection2DArray` on `/shuttlecock_detections` and an annotated
debug image on `/shuttlecock_detection/debug_image`.

### `shuttlecock_seeker`

Subscribes to detections + depth image + camera intrinsics.
Deprojects the detected shuttlecock's pixel center into a 3D point using
depth sampling (5px min patch) and the pinhole camera model, transforms it
into the map frame via tf2, and sends `NavigateToPose` goals to Nav2.

Filtering logic to avoid false targets:

- Ignores detections near gather points (already-deposited shuttlecocks)
- Ignores detections within the robot's own pickup radius (self-detection)
- Ignores targets outside court boundaries

Recovery behavior when no valid target is found:

- Spins in place to scan surroundings (up to 3 times)
- If still no target, relocates to predefined search points near known shuttlecock locations

Coordinates with the collector via `/shuttlecocks_collected` and
`/shuttlecocks_deposited` topics to trigger deposit runs when a batch is full
(or when all visible shuttlecocks are collected). Stops after the configured
total (3) have been deposited.

### `shuttlecock_collector`

Manages the physical collection and deposit lifecycle. Uses Gazebo ground
truth (`ign model -m turtlebot4 -p`) to determine the robot's actual position,
calculates the scoop position, and captures any shuttlecock within 0.5m by
teleporting it into a hopper slot on the robot via `ign service set_pose`.
Keeps onboard shuttlecocks riding in the hopper as the robot moves
(re-teleported each cycle). All position checks (collection, dropoff,
tracking) use Gazebo ground truth, making them immune to AMCL drift.

When the robot reaches a gather point (within 0.6m), deposits all onboard
shuttlecocks in a grid layout within the drop-off zone. Publishes collection
and deposit counts for the seeker to coordinate batch/deposit cycles.

### `scoop_follower`

Keeps the `scoop_assembly` and `hopper_bin` Gazebo models rigidly attached to
the robot by querying Gazebo ground truth (`ign model -m turtlebot4 -p`) for
the robot's actual position and teleporting the models to fixed offsets at
1 Hz. Uses a busy guard and background threading to prevent `ign service`
subprocess pileup.

Also re-publishes the robot's ground-truth position to `/initialpose` every
5 seconds, keeping AMCL anchored to the real position. This prevents AMCL
particle filter drift on the symmetric court layout.

### `teleop_keyboard`

WASD/arrow-key teleoperation node for manual driving and testing.

## ROS2 Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/shuttlecock_detections` | `Detection2DArray` | 2D bounding boxes of detected shuttlecocks |
| `/shuttlecock_detection/debug_image` | `Image` | Annotated camera frame with detection overlays |
| `/shuttlecocks_collected` | `Int32` | Total shuttlecocks picked up (ever) |
| `/shuttlecocks_deposited` | `Int32` | Total shuttlecocks deposited at gather points |
| `/camera/image_raw` | `Image` | Bridged RGB camera from Gazebo |
| `/camera/depth/image_raw` | `Image` | Bridged depth camera (32FC1, meters) |
| `/camera/camera_info` | `CameraInfo` | Bridged camera intrinsics |
| `/scan` | `LaserScan` | Bridged RPLidar for SLAM/AMCL |

## How to Run

### Pre-requisites

- Ubuntu 22.04, ROS2 Humble, Gazebo Fortress (Ignition)
- TurtleBot4 packages (`turtlebot4_ignition_bringup`, `turtlebot4_navigation`)
- **Reboot before each run** — previous sessions leave stale FastRTPS shared
  memory files that block DDS discovery. Rebooting is the only reliable way
  to clear them.

### Build

```bash
cd ~/shuttler_ws
colcon build --symlink-install
source install/setup.bash
```

### Step-by-Step Launch

Open 7 terminals. **Every terminal** must have the environment sourced first:

```bash
source /opt/ros/humble/setup.bash && source ~/shuttler_ws/install/setup.bash
```

Then follow these steps **in order**, waiting for each readiness signal:

#### Step 1 — T1: Launch Gazebo + robot + bridges + detector

```bash
ros2 launch shuttler_sim empty_world.launch.py
```

Wait for **"Undock Goal Succeeded"** (or "OK creation of entity" + ~40s).

#### Step 2 — T2: Start localization (AMCL)

```bash
ros2 launch turtlebot4_navigation localization.launch.py \
  map:=$HOME/shuttler_ws/maps/court_map.yaml \
  use_sim_time:=true \
  params_file:=$HOME/shuttler_ws/install/shuttler_sim/share/shuttler_sim/config/nav2.yaml
```

Wait for **"Managed nodes are active"**.

#### Step 3 — Set initial pose (in any sourced terminal)

```bash
ros2 topic pub --times 10 --rate 1 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

**Important:** After it finishes, wait 30 seconds, then verify AMCL has converged:

```bash
ros2 topic echo /amcl_pose --once
```

Confirm the returned position is near `x: 1.0, y: 0.0`. If it hangs or
returns wrong values, wait longer and try again. **Do not proceed until
AMCL is confirmed correct.**

#### Step 4 — T3: Start Nav2

```bash
ros2 launch turtlebot4_navigation nav2.launch.py \
  use_sim_time:=true \
  params_file:=$HOME/shuttler_ws/install/shuttler_sim/share/shuttler_sim/config/nav2.yaml
```

Wait for **"Managed nodes are active"**.

#### Step 5 — T4: Start scoop follower

```bash
ros2 run shuttler_sim scoop_follower
```

Should print "Scoop follower started". This also begins AMCL re-seeding
from Gazebo ground truth every 5 seconds.

#### Step 6 — T5: Start collector

```bash
ros2 run shuttler_sim shuttlecock_collector
```

Should print "Shuttlecock collector started, tracking 3 shuttlecocks".

#### Step 7 — T6: Start seeker (must be last)

```bash
ros2 run shuttler_sim shuttlecock_seeker
```

The seeker begins the autonomous pipeline: detecting, navigating, collecting,
and depositing shuttlecocks. **Do not intervene** — let it run autonomously.
The run completes when it logs:
"Target of 3 shuttlecocks deposited - stopping."

### GUI (Optional — for recording/screenshots only)

The simulation runs headless by default for best performance. To view the
robot visually, open a GUI window in a separate terminal:

```bash
ign gazebo -g
```

**Warning:** The GUI drops sim speed to ~0.02-0.03x real-time on software
rendering (no GPU). Open it briefly for screenshots/recording, then close
to restore speed. The robot continues running headless even after the GUI
is closed.

### Troubleshooting

| Problem | Solution |
| --- | --- |
| Nodes can't discover each other / TF missing | Reboot the machine. Stale `/dev/shm/fastrtps_*` files block DDS. |
| AMCL pose returns wrong coordinates | Re-publish initial pose (`--times 10`), wait 30s, verify again. |
| scoop_follower crashes on startup | Gazebo not ready yet. Wait for undock to complete, then retry. |
| "No valid trajectories" spam in Nav2 | Normal during initial navigation. Nav2 recovery (spin + clear costmap) will resolve it. |
| Seeker navigates to negative coordinates | AMCL drifted. Kill seeker, re-publish initial pose, verify, restart seeker. The AMCL re-seed in scoop_follower should prevent this. |
| Sim too slow with GUI open | Close the GUI (`ign gazebo -g` window). The sim continues headless. |
| `PackageNotFoundError` when running nodes | Re-source: `source ~/shuttler_ws/install/setup.bash` |

### Performance Notes

- **Headless** (default): ~0.2-0.4x real-time, full run takes ~15-25 min
- **GUI open**: ~0.02-0.03x real-time — use briefly for visual confirmation only
- Always reboot between runs for best reliability

## Demo Scenario

The world contains 3 shuttlecocks placed on one half of the court:

- `shuttlecock_1` at (3.0, 0.8)
- `shuttlecock_5` at (4.0, 1.5)
- `shuttlecock_3` at (6.0, 3.0)

The robot starts at (1.0, 0.0), detects and navigates to each shuttlecock,
scoops it into the hopper, and deposits batches at the NE corner drop-off
zone (7.3, 4.3). The pipeline stops after all 3 are deposited.

## Status

The full autonomous pipeline (detection -> 3D localization -> navigation ->
scoop collection -> batch deposit -> search recovery) has been validated
end-to-end in headless simulation, successfully collecting and depositing
all 3 shuttlecocks.

## Limitations and Future Improvements

### Current Limitations

- **Software rendering only**: the simulation runs on `llvmpipe` (no GPU), capping real-time factor at ~0.2-0.4x headless and ~0.02-0.03x with a GUI — a full 3-shuttlecock run takes 15-25 minutes wall-clock
- **Scoop/hopper attachment is kinematic, not physical**: the scoop and hopper models are teleported to follow the robot rather than being joined via physics joints, so they do not interact with the environment realistically (e.g. no collision-based scooping)
- **HSV-only detection**: the detector relies on a fixed orange HSV range tuned to the simulation's shuttlecock material — it would not generalize to real-world lighting, shadows, or shuttlecock color variation without retraining
- **Static shuttlecock positions**: the collector tracks shuttlecocks by their known spawn coordinates in the world file rather than dynamically sensing their ground-truth position, so it cannot handle shuttlecocks that have been moved by physics or external forces
- **Single-half-court scenario**: all 3 shuttlecocks and the drop-off zone are on the same half of the court — the robot never crosses the net or handles a full-court layout
- **No obstacle avoidance for dynamic objects**: Nav2's costmap only considers the static SLAM map — another robot or a person on the court would not be avoided

### Future Improvements

- **Deep learning detection**: replace HSV filtering with a trained object detector (e.g. YOLOv8) for robust real-world shuttlecock recognition across lighting conditions and backgrounds
- **Physical scoop joint**: attach the scoop via a Gazebo joint with contact sensors so collection happens through actual physics interaction rather than teleportation
- **Dynamic shuttlecock tracking**: use the vision pipeline to continuously track shuttlecock ground positions instead of relying on hardcoded spawn coordinates
- **Full-court support**: add multiple gather points, net-crossing navigation, and a coverage-based search strategy for a regulation-size court with more shuttlecocks
- **GPU-accelerated rendering**: run on a machine with a discrete GPU to achieve real-time simulation, enabling faster iteration and realistic sensor noise
- **Real robot deployment**: port the pipeline to a physical TurtleBot4 with a real OAK-D camera, validating detection and navigation in a real badminton court environment
