# field_exploration_ws

Multi-robot frontier exploration on ROS 2 Jazzy and Gazebo Harmonic. A Husky A200 and two Jackal J100 robots (`j100_0000`, `j100_0001`) explore an Open-RMF demo world (Hotel by default) using our own mapping, frontier detection, goal assignment and Nav2 goal execution.

| Layer | Package | Language | Source |
|---|---|---|---|
| Gazebo worlds | `rmf_demos_maps`, `rmf_demos_gz` | — | your fork `ajayiyer22/rmf_demos` @ `hotel-exploration` |
| Robots | `clearpath_gz`, `clearpath_generator_gz` | — | your fork `ajayiyer22/clearpath_simulator` @ `hotel-simulation` |
| Ground-truth localization, doors | `search_sim` | Python | ours |
| YOLO26 RGB detection on PyTorch CUDA | `search_perception` | Python | ours, Ultralytics YOLO26 |
| Lidar obstacles, occupancy grid, costmaps | `exploration_map` | Python | ours |
| WFD, goal creation, Hungarian, Nav2 execution, mission | `search_exploration` | C++ | ours |
| Interfaces | `search_msgs` | — | ours |
| Launch, configs, RViz | `search_bringup` | — | ours |

## Submodules

The two simulation submodules use your GitHub forks. Frontier exploration remains
our custom `search_exploration` package; the old `frontier_exploration_ros2`
submodule is not used by this workspace's launch files.

Your fork branches originated on older ROS/Gazebo versions. Local compatibility
patches adapt their bridges, launch files and world generation to Jazzy/Harmonic.
They are saved in `scripts/submodule_patches/` so fresh checkouts are reproducible:

```bash
# Fresh checkout (GitHub access to the main repository and both forks is required):
git clone --recurse-submodules https://github.com/ajayiyer22/field_exploration_ws.git
cd field_exploration_ws
python3 scripts/apply_submodule_compatibility.py

# For an existing checkout instead:
# git submodule update --init --recursive
# python3 scripts/apply_submodule_compatibility.py
```

Apply the patches before building. They intentionally leave the two submodule
working trees modified; they have not been committed or pushed to your forks.
Their original upstream repositories remain available as local `upstream` remotes.
Nav2 settings and the custom mapping, allocation and YOLO logic remain in this
workspace. Rebuilding will not select the fork's old Hotel navigation launch files.

## Run

Activate the `field_robotics` Conda environment (Python 3.12, the version ROS 2 Jazzy is built for). The custom Python launch nodes explicitly use its interpreter, even when the ROS CLI or an installed script has a system-Python shebang. Gazebo and Nav2 are native executables inheriting the launch environment. `colcon_defaults.yaml` selects Conda Python for CMake packages; `python -m colcon build` also selects it for Python packages.

Build once, and again after code changes:

```bash
cd ~/field_robotics_ajay/field_exploration_ws
conda activate field_robotics
source /opt/ros/jazzy/setup.bash
python -m colcon build --symlink-install --packages-up-to search_bringup
```

Then run two launch commands, each in its own terminal, after `conda activate field_robotics`, `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash`:

```bash
# 1. Simulation: world, robots, localization + exploration map, Nav2, RViz
ros2 launch search_bringup simulation.launch.py

# 2. Exploration: WFD, Hungarian allocation, dispatcher, mission coordinator
ros2 launch search_bringup exploration.launch.py
```

- **Order:** start exploration any time after the simulation; the mission coordinator waits until every robot's pose, the map and Nav2 are ready.
- **Options:** `simulation.launch.py` takes `headless:=true` (no Gazebo GUI) and `rviz:=false`.
- **Camera views:** RViz opens three RGB + YOLO26 Image panels beside the map, subscribed to `/<namespace>/yolo26/annotated_image`. With `yolo:=false`, it shows the raw `/<namespace>/sensors/camera_0/color/image` streams. Both configurations use best-effort, volatile QoS. Drag the panel title bars to arrange them; use **Panels** to show a hidden panel.
- **Task allocation:** a separate `Task allocation` Image panel shows mission state, plan number, unassigned count, and one coloured row per robot: orange Husky, blue Jackal 1, green Jackal 2. Each row shows navigation status, the dispatcher's actual active frontier, and the allocator's frontier IDs in visit order. The panel subscribes to the live plan and status messages and remains available before exploration starts. It uses `/search/task_allocation/image`; its reliable, transient-local QoS preserves the latest panel for RViz. `Task allocation routes` on the map uses the same robot colours for assigned goals, connecting lines and visit-order labels. These straight connecting lines show task order; the separate Nav2 plan displays show navigable paths. Drag the panel title bar to arrange it or show it again from **Panels**.
- **YOLO26:** simulation starts the detector by default on CUDA GPU 0 (the RTX 5060 Ti). `yolo_device:=0`, `yolo_target:="red tennis ball"`, and `yolo_model:=/absolute/path/to/model.pt` override the defaults. `yolo_target:=""` enables all-class detection without target-based mission stopping. Other settings live in `config/yolo26.yaml`.
- **Stopping:** Ctrl+C. `./scripts/sim_kill.sh` clears anything left over from a crashed run.

The Conda env needs `numpy`, `scipy`, `pyyaml`, `pytest`, and the colcon build tools (`colcon-common-extensions`, `empy==3.3.4`, `lark`, `catkin-pkg`), as in the field_robotics `conda-environment-jazzy.yml`. ROS packages (`rclpy`, messages, `gz` transport bindings) come from `/opt/ros/jazzy`.

Perception additionally uses the existing CUDA-enabled PyTorch `2.11.0+cu128`, Ultralytics `8.4.144`, and OpenCV in that environment. The detector loads the local `search_perception/models/yolo26n.pt` weights copied from the older workspace; it does not download weights at startup. It warms up the actual PyTorch backend and requires CUDA for inference. Gazebo physics, mapping, frontier allocation and Nav2 continue to run on the CPU.

To attach perception to a simulation started with `yolo:=false`, run this once in another sourced terminal:

```bash
ros2 launch search_bringup perception.launch.py
```

The detector publishes per-robot `/<namespace>/yolo26/detections` (`vision_msgs/Detection2DArray`) and camera images with only a target-status line: **Red tennis ball detected** or **Target not detected** for the default target. No object class labels, confidence labels or bounding boxes are drawn. This status describes the current camera frame; `/search/target_found` is latched true only after the configured target matches three distinct consecutive frames from one camera. The coordinator then ends the mission as `TARGET_FOUND`. `/search/yolo26/status` reports the actual GPU, precision and processed-frame counters. `/search/target_detection` retains the confirming frame's robot, timestamp, boxes and confidence. With a configured target, inference results are filtered to that YOLO class. The default red tennis ball target maps to YOLO's sports ball class and requires at least 50% red among sufficiently saturated, bright pixels inside the central object box. This is a color heuristic, not a separately trained tennis-ball classifier. The optional pink qualifier remains available for legacy targets. Each camera keeps only its latest frame, and inference uses a steady-clock timer so simulation time does not throttle the processing loop.

```bash
ros2 topic echo /search/yolo26/status --once
```

Two more pieces are needed under Conda:
- `protobuf==4.21.12`. The Jazzy `gz.msgs10` bindings used by `ground_truth_localizer` need it.
- The system `python3-apt` modules, which Clearpath's `generate_param` imports. They are not on PyPI, so link them into the env:

```bash
pip install "protobuf==4.21.12"
SP=$(python -c "import site; print(site.getsitepackages()[0])")
for m in apt aptsources apt_pkg.cpython-312-x86_64-linux-gnu.so apt_inst.cpython-312-x86_64-linux-gnu.so; do
  ln -sf /usr/lib/python3/dist-packages/$m "$SP/$m"; done
```

Everything else is configuration in `src/search_bringup/config/`:
- the team: `robots.yaml`;
- the world, spawn poses and doors: `worlds/hotel.yaml`;
- the exploration parameters: `exploration.yaml`;
- the mapping parameters: `exploration_map.yaml`.

The launch pieces the simulation is assembled from are in `src/search_bringup/launch/include/`.

Results of each run (time, known area, per-robot distance) are written to `results/`.

Automatic exploration completion requires WFD to report no frontiers for 15
simulation seconds, a corresponding empty allocation, and fresh map, pose and
idle-status observations from every robot. An empty allocation while frontiers
remain does not terminate the mission: costmap rejection and the 30-second
failure blocks are not proof of unreachable space. The mission stays active
and retries; path-based proof of unreachable frontiers is not implemented yet.

### First-time setup

```bash
git clone --recurse-submodules <this repo> field_exploration_ws
cd field_exploration_ws
sudo apt install ros-jazzy-rmf-building-map-tools ros-jazzy-rmf-building-sim-gz-plugins \
    ros-jazzy-rmf-robot-sim-gz-plugins ros-jazzy-rmf-door-msgs ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-rotation-shim-controller ros-jazzy-gz-ros2-control ros-jazzy-ros-gz python3-scipy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    --skip-keys "rmf_demos rmf_demos_fleet_adapter rmf_demos_tasks rmf_demos_bridges"
```

## Data flow

```
Gazebo dynamic_pose ─► ground_truth_localizer ─► /<ns>/ground_truth/pose, map→odom on /<ns>/tf
/<ns>/sensors/lidar2d_0/scan ─► exploration_map:
      lidar_obstacle_detection (per robot) ─► occupancy_grid_generator ─► /exploration_map/occupancy
      cost_map_generation (per robot)      ─► /<ns>/exploration_costmap ─► Nav2 Smac 2D + MPPI
/exploration_map/occupancy ─► frontier_detector (WFD + goal creation) ─► /search/frontiers
/search/frontiers ─► task_allocator (C_ij = D_ij → Hungarian → distance order) ─► /search/task_plan
/search/task_plan ─► exploration_dispatcher ─► /<ns>/navigate_to_pose
exploration_dispatcher ─► /search/allocation_constraints, /search/reallocation_request ─► task_allocator
mission_coordinator ─► /search/mission_state (WAITING, EXPLORING, EXPLORATION_COMPLETE)
```

## Algorithms

**Costmap** (`exploration_map/cost_map_generation.py`). For each robot, obstacle cells cost A = 255 and the cost decays with the distance dᵢ (metres) to the nearest obstacle:

  cost = A · e^(−dᵢ · decay_rate), decay_rate = 1/S, S = robot length × width

A larger robot gets a slower decay.

**Nav2** (`search_bringup/config/nav2_*.yaml`):
- **Global costmap and planner.** The global costmap reads this costmap through a non-trinary `StaticLayer`, and Smac 2D plans on that cost. An `InflationLayer` with radius equal to the robot's inscribed radius (half its width) and very steep scaling makes cells closer than that to a wall non-traversable. This is needed because Smac 2D plans the robot centre as a point: without it, a robot gets routed into gaps narrower than itself. Beyond that band, the decay costmap is the only cost.
- **Local costmap and MPPI.** The 14 m rolling local costmap combines shared-map walls, live lidar and an `InflationLayer` with `cost_scaling_factor: 3.0`. Its radius covers the rectangle padded by 5 cm on each side, plus a 10 cm margin, rounded up to the 5 cm grid: Husky 0.80 m and each Jackal 0.55 m. MPPI uses this bounded gradient and checks the full rectangular footprint near obstacles. Inflation is needed because MPPI skips footprint checks at zero-cost centre cells; the custom area-based exploration gradient remains in the global map.

**RViz costmap displays.** Expand `Husky costmaps`, `Jackal 1 costmaps` or `Jackal 2 costmaps` in Displays. Each group contains `Global planner costmap` (the final Nav2 `/<ns>/global_costmap/costmap`) and `Local MPPI costmap`. The latter uses `/<ns>/local_costmap/rviz_costmap`: `costmap_visualizer` transforms the original local grid's origin from that robot's separate odom frame into the shared map frame at the grid timestamp, without changing or resampling cell values. The local overlay sits 3 cm above the global layer to prevent flicker. Only the Husky group is enabled initially; select one robot group at a time and toggle its two layers to compare them. Disable the global layer when inspecting local costs alone, since zero local cost is transparent and otherwise shows global cost underneath. Camera panels remain available for all robots.

Both layers use RViz's standard costmap colours: zero cost is transparent (white free-space map underneath), increasing costs run from blue through purple to red, inscribed cost 253 is cyan, lethal cost 254 is magenta, and unknown cost 255 is grey-green. Nav2 converts these raw values to OccupancyGrid display values 0, 1–98, 99, 100 and -1, respectively. These colours encode cost, not robot identity.

**WFD** (`search_exploration/src/lib/wfd.cpp`) follows Topiwala et al., *Frontier Based Exploration for Autonomous Robot*, Fig. 4. A frontier point is an unknown cell with an open-space neighbour. The outer BFS from all robots enqueues only cells with an open-space neighbour. The inner BFS extracts each frontier. The Map-Open, Map-Close, Frontier-Open and Frontier-Close marks give each frontier point exactly one frontier. Occupied cells are never enqueued, so the wavefront cannot cross a wall.

**Goal creation** (`frontier_goal.cpp`) retains the field_robotics geometry and adds a Nav2 cost check:
- centroid = the mean of the frontier's cells;
- center point = the frontier cell nearest the centroid;
- start point = the cell where the inner BFS started;
- goal = the free neighbour nearest the centroid with raw cost **0–29 in every robot's fresh Nav2 global costmap**. Local MPPI costmaps are used by Nav2 for navigation and do not gate exploration allocation. The allocator and dispatcher recheck the common global condition; an active goal is cancelled if it stops passing.

The detector reconsideres goal locations when Nav2 costmaps change, even without an occupancy-map update. If a frontier has no admissible neighbour, its geometric fallback remains in the frontier message for unassigned-task and mission accounting. It is not displayed as an eligible goal marker and cannot be assigned until it passes every robot's global cost check. RViz labels those frontiers `unassigned`. Purple centroid markers still describe frontier geometry and may lie in unknown space; they are not navigation targets.

Allocated goals use labels `H1, H2, …` for Husky, `J11, J12, …` for Jackal 1 and `J21, J22, …` for Jackal 2. The suffix is the position in the current visit queue, so a new allocation can renumber labels. RViz's map labels and Task allocation panel use the same prefixes from `robots.yaml`; internal frontier IDs remain unchanged.

**Allocation and dispatch.** Hungarian minimizes the straight-line robot-to-goal distance. Each available robot gets up to ⌈frontiers / available robots⌉ assignments; its queue is sorted by distance. Existing active goals stay with their owners to avoid duplicate execution. The dispatcher sends one Nav2 goal per robot.

A Nav2 abort, rejected goal or send error excludes that entire robot for `failure_timeout_s` (currently 30 simulation seconds), releases its route and immediately requests a new Hungarian allocation across the remaining robots. Capacity is recomputed for that smaller team. Healthy active goals continue. Once the timeout expires, the robot becomes eligible again. If no robot is available, frontiers stay unassigned and the mission remains exploring.

Global eligibility is checked every 0.25 simulation seconds. Cost changes, freshness changes, robot availability and frontier changes request a new allocation, with a 5-second periodic fallback. The historical MRTSP library and its tests remain available, but MRTSP is no longer used by the running allocator.

## Tests

```bash
conda activate field_robotics && source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select search_exploration exploration_map && colcon test-result --verbose
```

The tests cover:
- Kuhn's §5 example (optimum 27);
- brute-force checks of the assignment;
- the old allocator's balanced example (total 11);
- WFD's uniqueness and wall properties;
- the old workspace's MRTSP tests (cost terms, pruning, bounded DP versus greedy);
- the costmap decay and the occupancy updates.
- node-level regressions on an isolated ROS graph with mock Nav2 action servers:
  eligibility changes and freshness recovery, stable active goals, cancellation,
  approach feedback, terminal stopping, and guarded mission completion.

## Status

Verified:
- **Tests:** algorithm, mapping and node-level regression tests run with the command above.
- **Simulation, mapping and Nav2:** the world and all three robots come up, and ground-truth localization, the shared map and the costmaps work. Nav2 reaches goals that need a turn.
- **Exploration:** the full loop runs, from WFD through Hungarian to Nav2. In the best run (Hotel, all three robots), about 490 m² of free space was mapped in about 500 s of simulated time.

Open issues in the furnished Hotel:
- **Robots get physically stuck.** After roughly 10 simulated minutes, all three robots end up pinned against furniture or in doorways. They are commanded forward but don't move. The 2D lidar can't see beds, desks and other low furniture, and the 15°/s yaw limit makes turning in tight rooms slow. The dispatcher cancels the goal and reassigns the frontier, but a pinned robot stays pinned.
- **Team change:** the Warthog was replaced by a second Jackal to reduce doorway clearance problems. Historical run measurements above refer to the earlier Husky–Jackal–Warthog team.

Candidate fixes:
- add the RealSense depth cloud to the Nav2 obstacle layers (a first attempt did not help yet);
- raise `w_max`;
- add a stuck detector that triggers a BackUp;
- use a less cluttered RMF world, such as `office`.
