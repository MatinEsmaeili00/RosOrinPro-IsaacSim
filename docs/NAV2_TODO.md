# Nav2 + splat environment: to-do list and progress

> **Request (5 November 2026):** add Nav2 to the simulation so the robot can navigate on its own, in the
> Gaussian-splat environment in `C:\isaac-sim-standalone-6.2.0\Splats\`. Work step by step, keep a to-do
> list and the work in progress on GitHub, do not remove anything, add new files.

This file is the live tracker. It is updated and pushed after every step on the branch
`feature/nav2-splat` (draft pull request on GitHub). Nothing that existed before is removed; all changes are
new files or additions.

Legend: ✅ done · 🔄 in progress · ⬜ to do · ⚠️ blocked or needs a decision from you

## Decisions taken so far

| Question | Decision | Why |
|---|---|---|
| Where does Nav2 run? | **Natively on Windows**, ROS 2 **Humble** from [RoboStack](https://robostack.github.io/) installed with [pixi](https://pixi.sh) | RoboStack ships Nav2 1.1.20 and slam_toolbox 2.6.10 for win-64. Same distro (Humble) and middleware (FastDDS) as the simulator and the robot's Jetson, so no Docker or WSL network bridging is needed. The NVIDIA skills in this install mark WSL2 as deprecated for Isaac Sim ROS work. |
| Which ROS distro for the Nav2 config? | Humble | The real robot runs Humble. The same `rosorin_nav2` package should run on the Jetson later. |
| Does the 946 MB splat go on GitHub? | **No.** The scripts that turn it into a scene and the small derived files (occupancy map, collision proxy) go on GitHub. | GitHub refuses files over 100 MB. The PLY stays in `C:\isaac-sim-standalone-6.2.0\Splats\`. |

## Facts found about the splat

`Splats/splat (1).ply`: standard 3D Gaussian Splatting PLY (INRIA layout), **3 998 372 Gaussians**, spherical
harmonics degree 3, 946 MB.

* Already **Z-up and metric**: the densest layer is the floor at z ≈ 0, there is a second peak at
  z ≈ 0.71 m (table tops), and the ceiling is at about 3.0 m.
* Indoor building, about 20 m × 24 m: rooms with tables and chairs, a corridor, curved walls.
* A few hundred far-away "floater" Gaussians reach ±440 m and must be cropped.

## Plan and status

### Phase 0: tracking
* ✅ Branch `feature/nav2-splat`, this file, draft pull request.

### Phase 1: Nav2 on this PC
* ✅ Install pixi 0.81 (winget, user-level, no admin).
* ✅ `nav2/pixi.toml` + `pixi.lock`: ROS 2 Humble with Nav2 1.1.18, slam_toolbox 2.6.10, RViz 2,
  FastDDS 2.6.10 (3.9 GB in `nav2\.pixi\`, not committed).
* ✅ The pixi ROS 2 sees every simulator topic (`pixi run topics` lists all 19 in 4 s).
  Note: the `ros2` CLI daemon hangs on Windows, so use `--no-daemon` (the `topics` task does).

### Phase 2: `rosorin_nav2` ROS 2 package
* ✅ Nav2 parameters for the mecanum base (`nav2/rosorin_nav2/config/nav2_params.yaml`): MPPI controller
  with the Omni motion model, AMCL `OmniMotionModel`, measured footprint, costmaps from `/scan`,
  slam_toolbox section.
* ✅ Launch file `bringup.launch.py`: AMCL on a saved map or `slam:=True`, initial pose per scene from
  `config/worlds.yaml`, RViz.
* ✅ Goal tool `ros2 run rosorin_nav2 goal` (x y yaw, `--place`, `--tour`, `--json`). It lives in the
  package, not in `ros2_tools/`, because Isaac Sim's bundled ROS 2 has no `nav2_msgs`.

### Phase 3: Nav2 in the existing test arena (known geometry)
* ✅ First live run (SLAM + Nav2 against the headless arena simulation): goal 1 m ahead **SUCCEEDED**,
  0.95 m driven in 4.5 s sim time, final error 0.14 m.
* ✅ Automated test `launch\run_nav2_test.bat` (`tests/test_nav2.py`): starts the simulation and Nav2,
  runs a tour, stops everything, writes `tests/results/report_nav2_<world>.json`.
* ✅ Arena map from the arena's colliders (`scripts/build_usd_map.py` → `maps/test_arena.{pgm,yaml}`).
* ✅ **Arena, AMCL: 4/4 goals**, true final error 4–8 cm, AMCL localization error 5–7 cm
  (`tests/results/report_nav2_arena.json`).
* ⚠️ **Arena, SLAM: 3/4 goals.** slam_toolbox's pose drifted to 0.77 m / 28° from ground truth on the third
  leg and that goal aborted (`report_nav2_arena_slam.json`). The simulated odometry is exact, so this is
  scan matching in a small, square room with the lidar's blind sectors.

### Phase 4: the splat environment
* ✅ Analyse the PLY (orientation, scale, extent).
* ✅ Convert the PLY to USD with Isaac Sim's converter (`omni.kit.converter.gsplat`, half precision):
  3.94 M Gaussians after cropping, `usd/splat/building.usd` (512 MB, not in git, rebuilt by
  `launch\build_splat_world.bat`). It renders in RTX: see `docs/images/splat/hall_view.png`.
* ✅ Occupancy grid from the Gaussians (`scripts/splat_occupancy.py`): floor layer = free, robot-height band =
  obstacles, ring around the scanned floor = walls. 166 m² free.
* ✅ Nav2 map `nav2/rosorin_nav2/maps/splat_building.{pgm,yaml}` + preview `docs/images/splat_map.png`.
* ✅ Proxy geometry `scenes/splat_proxy.usd`: invisible collision boxes + a thin camera-hidden strip at the
  lidar height. Five variants tested (details in the log below).
* ✅ Scene `scenes/splat_world.usd`: splat + proxy + robot in the hall (spawn 2.98, 0.12, 70°).
* ✅ Sensors in the splat world: `sensor_snapshot.py` **6/6 PASS** (camera sees the splat, depth 99 % valid,
  lidar 237 beams on the walls).
* ✅ All 5 named places free (clearance 0.9–1.9 m) and reachable from the spawn
  (`tests/results/splat_build_report.json`). The lab is not reachable (its doorway is too narrow in the scan).

### Phase 5: Nav2 through the splat building
* ✅ **Tour of the building with AMCL on the generated map: 5/5 goals SUCCEEDED** (twice, identical:
  62.6–62.8 m driven in 226 s simulation time, ~7.5 min wall), no recovery behaviour needed.
* ✅ Checked against ground truth (the simulated `/odom` is exact): true final position error **4–11 cm**,
  AMCL localization error **2–10 cm** and ≤ 1.1° (`tests/results/report_nav2_splat.json`).

| Leg | Distance | Sim time | True error | AMCL error |
|---|---|---|---|---|
| hall → conference_room | 11.5 m | 41 s | 0.04 m | 0.02 m |
| → north_hall | 17.9 m | 61 s | 0.08 m | 0.03 m |
| → east_lounge | 15.8 m | 58 s | 0.08 m | 0.10 m |
| → south_hall | 10.4 m | 39 s | 0.11 m | 0.09 m |
| → hall | 7.2 m | 27 s | 0.06 m | 0.05 m |

* ⚠️ **Splat building, SLAM: 3/5 goals** (`report_nav2_splat_slam.json`). The two far places (6.5 m and
  13 m, unexplored at the start) cannot be planned, because SLAM's costmap only covers what is mapped
  so far. The three reachable ones succeeded, with the SLAM pose drifting up to 0.64 m. **Use the
  generated map + AMCL to navigate** (5/5, 2–10 cm); SLAM is for mapping new places.

### Phase 6: documentation
* ✅ `docs/NAV2.md` (how to run, parameters, SLAM notes, real robot, troubleshooting).
* ✅ `docs/SPLAT_ENVIRONMENT.md` (how the scene is made, the proxy tests, using your own splat).
* ✅ README, TESTING (§ 6) and WORKLOG (§ 16 + open items): new sections and rows only.
* ✅ Docs images: `tools/render_splat_docs.py` and `sensor_snapshot.py` → `docs/images/splat/`.

### Phase 7: demos (asked for on 2026-11-05, after the first push)
* ✅ `docs/NAV2_DEMOS.md`: eight demos with steps, things to point out and how each was tested, plus a
  10-minute show.
* ✅ Surprise obstacle: `--obstacle X,Y[,SIZE]` in `launch/run_headless.py` and `tests/test_nav2.py`; `--tag` keeps
  extra reports apart. Box on the baseline path → the robot passed 0.69 m to the side, 1/1, no recovery
  (`report_nav2_splat_obstacle.json` vs `report_nav2_splat_baseline.json`).
* ✅ Waypoints: Nav2's `FollowWaypoints` and `NavigateThroughPoses` (the RViz waypoint buttons) both succeeded,
  17.5 m past the box.
* ✅ Goal tool retries while Nav2 is still activating. Found because a goal sent right after bringup was
  rejected: localization was up 7 s before bt_navigator.
* ✅ Ctrl+C (or Ctrl+Break) on the goal tool now cancels the goal. Before, the tool exited but Nav2 kept driving:
  measured, the robot went on from (3.59, 1.75) to (5.72, 6.82). Now it stops where it is (5 cm). The signal
  only sets a flag, because an exception raised inside rclpy's C++ code arrives as a RuntimeError.
* ✅ RViz: top-down view follows the robot (x right, like the map pictures); a "Robot camera" group (off) with
  the wrist RGB image and depth points.
* ✅ Wrist camera "look ahead" poses for the demos, worked out from the URDF: `joint4` = 0 is level, 0.35 is 20° down.

### Open items (for you to decide or for a next session)
* ⬜ No camera images in a **headless** run of the splat building with cameras on: `sensor_snapshot.py` got
  no `/depth_cam/...` messages, although the camera graph started. In the GUI the camera works (6/6). It
  matters only for `run_nav2_test.bat --cameras`.
* ⬜ Run `rosorin_nav2` on the real robot (`use_sim_time:=False`), see `docs/NAV2.md` "On the real robot".
* ⬜ SLAM accuracy: slam_toolbox drifts 0.3–0.8 m in the simulation. Suspects are listed in WORKLOG
  "Not done / open items".
* ⬜ Lidar beam height (older open item): a real `/scan` would show whether the 147 blind beams are real.
  Fewer blind beams would also help SLAM and AMCL.
* ⬜ The lab in the splat building is unreachable (narrow doorway in the reconstruction). Lowering
  `occupancy.obstacle_min_weight` or editing the map by hand could open it.
* ⬜ Merge the pull request when you are happy with it (nothing on `main` was changed).

## Log

* **2026-11-05:** read the repo, analysed the splat, chose native-Windows Nav2 (RoboStack Humble).
  Created this tracker.
* **2026-11-05:** Nav2 environment installed and the `rosorin_nav2` package built. First Nav2 goal reached
  in the simulation. Found: in `/scan` the ~147 beams blocked by the robot's own body are `-1.0`, below
  `range_min`, so the costmaps drop them and AMCL treats them as max-range. They are blind spots (rear
  ±72° and two small wedges at about ±55° to the front), not phantom obstacles.
* **2026-11-05:** splat scene built and checked. Problems found and fixed on the way:
  1. The splat did not show in the scene: `UsdGeom.Xform.Define()` on the referencing prim wrote a local
     `Xform` type that overrode `ParticleField3DGaussianSplat`. Now a typeless prim.
  2. Gaussians have no collision and no lidar return, so a proxy is needed. Tests of the proxy's render flags:
     `invisible` and `guide` are ignored by the RTX lidar (no `/scan` is published at all);
     `hideForCamera` is seen by the lidar but cuts the Gaussians behind it out of camera images (dark 50 cm
     bands, `docs/images/splat/hall_view_full_height_proxy.png`); adding `doNotCastShadows` removes the bands
     but also hides it from the lidar; switching off light shadows does not help.
     Solution: invisible full-height collision boxes + a 6.5 cm camera-hidden strip around the beam height.
* **2026-11-05:** first splat tour failed before driving: nav2_bringup's `RewrittenYaml` drops the
  backslashes of a Windows map path (`C:isaac-sim-standalone-6.2.0RosOrinPro...`). The launch file now passes
  forward slashes. The test harness now stops at once when Nav2's bringup aborts. Second run: **5/5 goals**.
* **2026-11-05:** ground-truth check added to the goal tool (`--ground-truth`). Results: splat 5/5 (true error
  4–11 cm), arena AMCL 4/4 (4–8 cm), arena SLAM 3/4 (slam_toolbox drift in the small square arena).
* **2026-11-05:** SLAM tests. A first SLAM run in the splat building was meaningless: SLAM's map frame starts
  at the spawn pose, not at the scene origin. Fixed with the goal tool's `--slam` flag. Then 3/5 (far
  unexplored goals cannot be planned, the SLAM pose drifts up to 0.64 m). Documented. AMCL on the generated
  map is the recommended way to navigate the building.
* **2026-11-05 (evening):** demos guide (`docs/NAV2_DEMOS.md`). New checks for it: surprise obstacle (box on
  the path, 1/1, passed 0.69 m to the side), RViz waypoint actions (both succeeded), goal right after bringup
  (found and fixed a race: rejected while bt_navigator was still activating), Ctrl+C on the goal tool (found
  and fixed: the goal was not cancelled, so the robot drove on). Put back the `r` prefix of
  `teleop_keyboard.py`'s docstring in the local copy; without it, `\r` and `\t` in its Windows paths are
  control characters. The version on GitHub was already right.
