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
* ⬜ Install pixi (user-level, no admin).
* ⬜ `nav2/pixi.toml`: ROS 2 Humble environment with Nav2, slam_toolbox, RViz 2 and FastDDS.
* ⬜ Check that the pixi ROS 2 and Isaac Sim's bundled ROS 2 see each other (topics, `/clock`).

### Phase 2: `rosorin_nav2` ROS 2 package
* ⬜ Nav2 parameters for the mecanum base: holonomic controller, AMCL omni motion model, footprint,
  costmaps from `/scan`, `use_sim_time`.
* ⬜ Launch files: navigation on a saved map, SLAM mapping, RViz.
* ⬜ Goal tool `ros2_tools/nav2_goal.py` (send a goal or a list of waypoints, report the result).

### Phase 3: Nav2 in the existing test arena (known geometry)
* ⬜ Occupancy map of the test arena.
* ⬜ Nav2 reaches goals in the arena; automated test with results in `tests/results/`.

### Phase 4: the splat environment
* ✅ Analyse the PLY (orientation, scale, extent).
* ⬜ Convert the PLY to USD with Isaac Sim's Gaussian-splat converter and check that it renders.
* ⬜ Build a collision and lidar proxy from the splat (walls, furniture), because physics and the lidar
  do not see Gaussians.
* ⬜ Occupancy map (`map.yaml` + `map.pgm`) from the splat.
* ⬜ Scene `scenes/splat_world.usd`: splat + proxy + robot at a free spawn pose.
* ⬜ Check that the robot camera sees the splat and the lidar sees the proxy.

### Phase 5: Nav2 through the splat building
* ⬜ Navigate between rooms (several goals or waypoints), with results and pictures.

### Phase 6: documentation
* ⬜ `docs/NAV2.md` (how to run), `docs/SPLAT_ENVIRONMENT.md` (how the scene is made).
* ⬜ Additions to README, TESTING and WORKLOG (new sections only, nothing removed).

## Log

* **2026-11-05:** read the repo, analysed the splat, chose native-Windows Nav2 (RoboStack Humble).
  Created this tracker.
