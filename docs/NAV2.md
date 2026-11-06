# Nav2 for the ROSOrin Pro

The robot navigates on its own with **Nav2** (ROS 2 Humble): give it a goal on the map, and it plans a path
and drives there around obstacles. The same setup works in two simulated worlds (the Gaussian-splat building and
the test arena), and the `rosorin_nav2` package is meant to run on the real robot too.

* Scene and map of the splat building: [SPLAT_ENVIRONMENT.md](SPLAT_ENVIRONMENT.md)
* Progress and to-do list of this work: [NAV2_TODO.md](NAV2_TODO.md)
* Demos to show, step by step: [NAV2_DEMOS.md](NAV2_DEMOS.md)

## How the pieces talk

```
 Isaac Sim 6.2 (Windows)                                   nav2\ pixi environment (Windows, RoboStack)
 ROS 2 Humble bundled with Isaac Sim, FastDDS              ROS 2 Humble, Nav2 1.1.18, slam_toolbox, RViz 2, FastDDS
 ┌─────────────────────────────────────┐   DDS, domain 0   ┌──────────────────────────────────────────────┐
 │ scenes\splat_world.usd               │ ───────────────► │ map_server ─► AMCL (or slam_toolbox)         │
 │   robot graphs: /scan /odom /tf      │  /scan /odom /tf │ planner (NavFn A*) ─► controller (MPPI Omni)  │
 │   /clock /imu /depth_cam/...         │  /clock          │ behaviors, BT navigator, velocity smoother   │
 │   mecanum IK  ◄── /cmd_vel           │ ◄─────────────── │ ─► /cmd_vel                                  │
 └─────────────────────────────────────┘                   └──────────────────────────────────────────────┘
```

* **No Linux, WSL or Docker is needed.** [RoboStack](https://robostack.github.io/) builds ROS 2 Humble, Nav2
  and slam_toolbox for Windows as conda packages, and [pixi](https://pixi.sh) installs them into `nav2\.pixi\`.
* It is the same distro (Humble) and middleware (FastDDS) as Isaac Sim's bundled ROS 2 and the robot's Jetson,
  so the processes find each other with default DDS discovery.
* The two ROS 2 installations must stay apart. Start the simulator from `launch\*.bat` (Isaac's own ROS
  libraries) and Nav2 from `nav2\` with `pixi run ...` (RoboStack's libraries). Do not run Nav2 from a shell
  in which `launch\rosorin_env.bat` was called, or the other way round.

## One-time setup

```bat
winget install prefix-dev.pixi                 :: user-level, no admin; open a new terminal afterwards
cd C:\isaac-sim-standalone-6.2.0\RosOrinPro-IsaacSim\nav2
pixi install                                   :: ~3.9 GB into nav2\.pixi\ (pinned by pixi.lock)
pixi run build                                 :: builds the rosorin_nav2 package into nav2\install\
launch\build_splat_world.bat                   :: (from the repo root) only if scenes\splat_world.usd must be rebuilt
```

## Run it (three terminals)

```bat
:: 1. the simulator (GUI) with the splat building; press Play (or set ROSORIN_AUTOPLAY=1)
launch\run_isaacsim.bat scenes\splat_world.usd
::    or without a window:   launch\run_headless.bat --scene scenes\splat_world.usd --no-cameras

:: 2. Nav2 + RViz (AMCL on the saved map, initial pose = the spawn pose)
cd nav2
pixi run nav world:=splat

:: 3. goals: click "Nav2 Goal" in RViz, or
cd nav2
pixi run goal --world splat --place north_hall
pixi run goal --world splat --tour               :: conference_room, north_hall, east_lounge, south_hall, hall
pixi run goal 7.2 12.7 70                        :: x y yaw_deg in the map frame
```

The same with the **test arena**: `launch\run_isaacsim.bat scenes\rosorin_world.usd` (or the user scene
`RosOrin_Claude.usd`, which contains the same arena) and `pixi run nav world:=arena`, places `north_east`,
`south_west`, `behind_pillar`, `by_crate`, `start`.

### Mapping instead of a saved map (SLAM)

```bat
pixi run nav world:=splat slam:=True
:: drive around: send goals, or ros2_tools\run.bat ros2_tools\teleop_keyboard.py from another terminal
pixi run ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true
pixi run nav world:=splat map:=C:/path/to/my_map.yaml
```

* **The SLAM map frame starts where the robot starts**, not at the scene's origin. Named places are scene
  coordinates, so add `--slam` to the goal tool: `pixi run goal --world splat --slam --place east_lounge`.
  (With a saved map, AMCL's map frame is the scene frame and no flag is needed.)
* **Goals must lie in the part already mapped.** Nav2's global costmap only covers the SLAM map so far, so a
  goal in unexplored space fails to plan (`GridBased: failed to create plan`). Map step by step: nearby goals or
  teleop first.
* **Accuracy (measured):** in the splat building, slam_toolbox's pose drifted to 0.3–0.6 m from ground truth
  over 25 m. AMCL on the saved map stays within 2–10 cm. Use SLAM to map a new place, and AMCL to navigate.
  Tuning slam_toolbox for this robot is an open item ([WORKLOG.md](WORKLOG.md)).

### Automated test

```bat
launch\run_nav2_test.bat                         :: splat building, AMCL, full tour
launch\run_nav2_test.bat --world arena --slam    :: arena, slam_toolbox
launch\run_nav2_test.bat --place north_hall --obstacle 5.37,6.0   :: a box that is not on the map
```

`--obstacle X,Y[,SIZE]` (repeatable) adds a box through `launch\run_headless.py --obstacle`, and `--tag NAME`
adds a suffix to the report and log names, so that extra runs do not overwrite the main report.

It starts the headless simulation, waits for `/scan`, starts Nav2, runs the tour, stops everything and writes
`tests/results/report_nav2_<world>.json` plus the logs of the three processes.

## What is in `nav2/`

```
nav2/pixi.toml, pixi.lock         ROS 2 Humble + Nav2 environment (RoboStack), tasks: build, nav, goal, topics
nav2/activate.bat / .sh           sets ROS_DOMAIN_ID=0 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp unless already set
nav2/rosorin_nav2/                the ROS 2 package (ament_python), also for the real robot
  config/nav2_params.yaml         all Nav2 parameters (+ slam_toolbox section)
  config/worlds.yaml              per scene: map, spawn pose (= AMCL initial pose), named places, tour
  launch/bringup.launch.py        wraps nav2_bringup/bringup_launch.py: world:= slam:= map:= initial_pose:= rviz:= use_sim_time:=
  maps/                           splat_building (from the splat), test_arena (from the arena's colliders)
  rosorin_nav2/goal.py            `ros2 run rosorin_nav2 goal`: goals, places, tours, JSON report
  rviz/rosorin_nav2.rviz          Nav2's default RViz layout; top-down view that follows the robot, and a
                                  "Robot camera" group (off by default): wrist RGB image + depth points
```

The goal tool is part of the package and not in `ros2_tools\`, because it needs `nav2_msgs`, which Isaac
Sim's bundled ROS 2 does not have.

## The parameters, and why

| Item | Setting | Reason |
|---|---|---|
| Controller | MPPI, `motion_model: Omni`, vx −0.10…0.30, vy ±0.25, wz ±1.0 | Mecanum base: it can strafe. The motors top out at about 0.46 m/s, and 0.30 leaves margin for the IK and slip. |
| Driving direction | `PreferForwardCritic`, `PathAngleCritic`, vx_min −0.10 | The lidar is blind behind the robot (see below), so the robot faces where it goes and reverses only slowly. |
| Localization | AMCL `nav2_amcl::OmniMotionModel`, 80 beams, `laser_min_range` 0.06 | Holonomic odometry model. |
| Footprint | rectangle ±0.155 × ±0.118 m | Chassis plus wheels span x −0.143…0.142 m, y ±0.106 m (URDF meshes), plus ~1 cm. |
| Costmaps | 5 cm, obstacle layer from `/scan`, inflation 0.35 m (local) / 0.45 m (global) | 5 cm = map resolution. The splat building has 0.9 m doorways. |
| Planner | NavFn, A*, `allow_unknown` | Simple, and works for SLAM too. |
| Velocity smoother | same limits, open loop | Nav2 Humble's default chain: controller → `cmd_vel_nav` → smoother → `/cmd_vel`. |
| Goal tolerance | 0.15 m, 0.25 rad | |
| Time | `use_sim_time: true` everywhere | The simulation runs at about 0.5–0.6× real time. |

**Lidar blind sectors.** About 147 of the 400 simulated beams return nothing, because the beam at the URDF's
`lidar_frame` height grazes the robot's own body (open item in [WORKLOG.md](WORKLOG.md)). They are published
as `-1.0`, below `range_min`. The costmap's projector drops them, and AMCL treats them as max-range readings
and skips them. So they are blind spots, not phantom obstacles: the robot's rear ±72° and two narrow wedges at
about ±55° to the front. If a real `/scan` shows that the real lidar sees more, lowering the emitter (see
WORKLOG) gives Nav2 more coverage without changing anything here.

## On the real robot

The package does not depend on the simulator. On the Jetson (Humble):

```bash
cp -r nav2/rosorin_nav2 ~/ros2_ws/src/ && cd ~/ros2_ws && colcon build --packages-select rosorin_nav2
source install/setup.bash
ros2 launch rosorin_nav2 bringup.launch.py use_sim_time:=False slam:=True rviz:=False   # map the room
ros2 launch rosorin_nav2 bringup.launch.py use_sim_time:=False map:=/path/room.yaml initial_pose:="0 0 0"
```

* The robot's own bringup must be running: `/scan`, `/odom` and the TF `odom → base_footprint → … →
  lidar_frame` (Hiwonder's controller and `robot_state_publisher`). Do not run Hiwonder's own navigation at
  the same time.
* Not yet tested on the robot. Check the speed limits and the footprint first (the arm must be folded in the
  `init` pose).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ros2 topic list` hangs on Windows | The ros2 CLI daemon. Use `pixi run topics` (`--no-daemon`), and `pixi run ros2 daemon stop` to end a stuck daemon. |
| `Failed to load map yaml file: C:isaac-sim...` (no backslashes) | nav2_bringup drops backslashes in the map path. `bringup.launch.py` converts paths to `/`. Pass your own maps with forward slashes too. |
| `[goal] rejected (is Nav2 still starting?), trying again in 2 s` | Normal for a few seconds after `pixi run nav`: localization is up before the navigator is active. The goal tool retries for 60 s. |
| Nav2 waits forever for the transform `map → base_footprint` | Is the simulation playing? Is the initial pose right (`world:=` must match the scene that is open)? |
| No topics between the simulator and Nav2 | Both must use domain 0 (or the same `ROS_DOMAIN_ID`) and `rmw_fastrtps_cpp`. Allow Isaac Sim and pixi's `python.exe` in the Windows firewall. |
| `Control loop missed its desired rate` | The MPPI controller needs more CPU than available. Lower `batch_size` (1500 → 1000) or `controller_frequency` (20 → 10) in `nav2_params.yaml`. |
| RViz shows no robot model | The meshes are `package://rosorin_pro_sim_description/...`, and that package is not in the pixi environment. TF, scan, map and paths still show. |
