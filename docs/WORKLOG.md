# Work log

What was done to put the ROSOrin Pro into Isaac Sim, in order (3–4 November 2026, Claude Code on this PC,
autonomously after the initial request). Rationale and data for each choice are in
[DECISIONS.md](DECISIONS.md).

> **Request:** add the robot from `rosorin_description-claude/` to the Isaac Sim scene `RosOrin_Claude.usd`
> with all sensors and features of the real robot (feature list:
> [Robot-ROSOrin-Pro](https://github.com/MatinEsmaeili00/Robot-ROSOrin-Pro)), make a new repo, and document
> everything.

## 1. Survey

* **Workspace:** Isaac Sim **6.2.0** standalone at `C:\isaac-sim-standalone-6.2.0`, Windows 11, RTX A6000
  (48 GB). Isaac Sim was not running; everything was driven through `python.bat` (headless `SimulationApp`)
  and later through the GUI launcher.
* **`RosOrin_Claude.usd`:** an essentially empty stage: `/World`, a distant light (3000, exposure 1.5) and
  viewport render settings (RealTimePathTracing). No ground and no physics. Backed up to
  `scenes/RosOrin_Claude.original.usda`.
* **`rosorin_description-claude/`:** Hiwonder's `rosorin_description` ROS 2 package (xacro + STL), plus a
  copy of the private RosOrinProMesh repo (Blender/Unreal model, flattened URDF, joint map with measured
  mecanum geometry).
* **Feature list:** 19 feature groups and about 50 topics observed on the real robot. Two key finds: the lidar
  driver node is `/sclidar_scan_publisher`, and the camera topics live under `/depth_cam/...`.
* **Hardware specs** from Hiwonder's ROSOrin Pro manual and product page: COIN-D6 ToF lidar (4000 Hz, 10 Hz,
  0.9°, 0.05–12 m); Aurora 930 Pro camera (640×400, 71°×46°, 0.15–3 m); HX-12H bus servos (1.18 N·m,
  0.2 s/60°, 0–240°); 1:90 motors (110 rpm); MPU6050; 11.1 V 6000 mAh battery; 3.6 kg; 284 × 212 × 544 mm.
  The size matches the URDF exactly.
* **Isaac Sim 6.2 APIs:** from the bundled `skills/` and the extension sources. One skill script
  (`multi_robot_namespacing.py`) uses node inputs that no longer exist in 6.2, so the official
  `isaacsim.ros2.nodes` `graph_api.py` was used as the reference for every node type and connection.
* **ROS 2 on Windows:** Isaac Sim bundles Humble and Jazzy with FastDDS, Cyclone and Zenoh. It defaults to
  Jazzy + Zenoh. Humble + FastDDS (the Jetson's middleware) needs environment variables set before launch.
  The bundled `rclpy` works standalone, so ROS 2 tools run on this PC without a ROS install.

## 2. Simulation URDF (`scripts/build_sim_urdf.py`)

* Wheels split out of the Pro chassis mesh with the RosOrinProMesh rule. The result is identical to the
  Blender build: 72,354 chassis triangles and the same per-wheel counts and roller angles.
* Every wheel split again into a hub and 9 rollers, each with a continuous joint about its PCA axis (8 mm ×
  31.5 mm rollers). Convex-hull envelope round to 0.11 mm.
* Masses set to reach the specified 3.6 kg (CAD masses unrealistic; `imu_link`'s duplicated inertial
  removed). Joint limits set from the servo and motor specs. Lidar yaw set for SCLIDAR.

## 3. Robot asset (`scripts/import_to_usd.py`, `scripts/configure_robot.py`)

* Isaac Sim 6.2's URDF importer writes a layered package with a `Physics` variant set
  (physx/mujoco/physics/none). Gripper mimic joints exist only as **Newton** mimic joints.
* `configure_robot.py` flattens the PhysX variant into one self-contained file and renames the mesh holders
  (the importer had used `camera_link` / `gripper_link` for mesh prims). It re-creates the 8 URDF frames that
  `merge_fixed_joints` removed and checks every body and frame against URDF FK: **2.7 µm, 1e-5 rad**.
* Physics: articulation solver settings, servo position drives, motor velocity drives, free rollers,
  **`PhysxMimicJointAPI`** for the 4-bar gripper, convex hulls (chassis: convex decomposition), and rubber /
  pad / body materials.
* Sensors: a **COIN-D6 RTX lidar** authored locally (single emitter, 4000 Hz firing, 10 Hz scan), the
  **Aurora 930 Pro** RGB and depth cameras on the optical frame, and an **MPU6050** IMU sensor.
* OmniGraph graphs for the ROS 2 interface: cmd_vel → wheels, joint commands, joint states, odometry, TF,
  IMU, lidar and camera topics with the real robot's names.

## 4. Scene (`scripts/build_scene.py`)

Added to `RosOrin_Claude.usd`: a physics scene, a ground with a floor material, the robot (referenced), a
`/clock` graph and a soft fill dome. Also a **test arena** (separate file, one prim): 4 × 4 m walls, a pillar
and a crate for the lidar and SLAM, a black line loop for line following, and red/green/blue 3 cm blocks for
color tracking and grasping. `scenes/rosorin_world.usd` is a self-contained copy inside the repo.

## 5. First end-to-end test: 28 / 33

`tests/test_sim_headless.py` drives the robot through ROS 2 like a user would. Every topic arrived, the lidar
returned 400 beams, camera HFOV was 71.00° and the IMU read 9.80 m/s². Arm and gripper reached their targets.
Failures:

1. **Driving:** only about 35 % of the commanded speed, and rotation in the wrong direction.
2. **Depth image empty:** in the URDF zero pose the arm points up and the wrist camera sees sky.
3. A **test-harness bug:** `rclpy.spin_once` handles one callback per call, so 17 topics appeared to run at
   3.3 Hz and `/odom` values were stale. Fixed with an executor drained every frame.

## 6. Mecanum physics

* A direct wheel-command diagnostic showed the physics basically worked: forward 0.18–0.20 m/s, correct
  directions. The problems were **wheel speed spikes up to 16 rad/s** and **60 % strafe efficiency**: a rigid
  four-wheel chassis on rigid contacts keeps lifting one wheel.
* Tuning sweep (`tests/tune_mecanum.py`). The first sweep looked hopeless, until height logging showed the
  robot had driven **onto the test blocks** (`z` +7 mm, wheels spinning freely). Blocks are now disabled in
  tests and moved off the robot's path.
* **Compliant (rubber-like) roller contacts** fixed it: strafe 60 % → 88 %, yaw noise 0.2–0.4 → 0.02–0.05
  rad/s.

## 7. cmd_vel → wheels

Isaac's `HolonomicController` QP scaled forward speed by 1/(4·cos 45°) = 0.354 and yaw by about 2.6 for
this geometry: it treats the wheel contributions as summing. It was replaced by the **exact mecanum IK** from
the measured roller axes, built as one `MatrixMultiply` in OmniGraph (the v·M convention was verified with
a scratch graph). Through `/cmd_vel`: forward 97 %, strafe 91 %, rotate 96 % (at 480 Hz; 96/88/94 % at the
final 240 Hz).

## 8. A scene-rebuild bug (OmniGraph connection deletes)

After rebuilding the scene, every robot graph was silent: the `Tick` nodes ran, nothing downstream did.
Cause: `stage.RemovePrim("/World/rosorin_pro")` on the live stage made OmniGraph "disconnect" the referenced
graphs by writing connection-**delete** opinions into the scene layer, and those stayed after the robot was
re-added. Fix: edit the layer with `Sdf` before opening it, then strip any override below the robot after
saving.

## 9. Arm, gripper, cameras

* **Init pose** like the real robot's `/init_pose`, chosen by an FK search: camera 0.28 m high, looking 45°
  down at the floor 0.26–0.69 m ahead.
* **Gripper calibrated** from the finger meshes: +0.9 rad opens to 58 mm, −0.65 closes. The config
  placeholder had the sign backwards. `r_joint` is limited so the fingers cannot pass through each other.
* **Camera exposure:** images were blown out (default 1 s exposure), so RTX auto exposure is used. **Publish
  rate** is set with `omni:sensor:tickRate` instead of the deprecated `frameSkipCount`, which had let the
  depth stream run unthrottled.

## 10. Python runtime: battery, `/robot_description`, digital twin

* **Battery** (`std_msgs/UInt16` in mV, like the STM32 board): a 3S 6000 mAh model with load-dependent
  current and sag.
* **`/robot_description`**, latched. `description/` became a ROS 2 package so RViz can resolve the meshes.
* **Digital twin:** follows the real robot's `/joint_states` and `/odom` on the robot's ROS domain. The first
  version teleported the base every physics step, which made the arm's joint2 sag to its limit within 1 s.
  It now drives the base physically through its wheels (feed-forward plus pose feedback): **4 mm /
  0.014 rad** base error, **0.018 rad** arm error against a fake robot driving a circle.

## 11. Launchers and ROS 2 tools

* `launch/run_isaacsim.bat` (GUI), `run_headless.bat`, `build_all.bat`, `run_tests.bat`, `rosorin_env.bat`.
* `ros2_tools/`: keyboard teleop with mecanum, gripper and arm presets; arm CLI; topic checker; square demo;
  QoS dump. They run on Windows with the bundled ROS 2 and also work on Linux.
* Fixed along the way: the bundled `rclpy` folder ships a numpy without its DLLs, so Isaac's numpy goes
  first on `PYTHONPATH`. In the GUI startup script, an `import` inside the async function shadowed `omni`, and
  the scene was opened before the ROS 2 bridge was loaded ("Could not find node type interface"). The viewport
  is now framed on the robot, because the scene's saved camera looked away from it.
* **Cross-process checks** (separate ROS 2 programs, as from a laptop or the Jetson): 18/18 topics alive;
  square demo loop error 8 mm and 1°; arm CLI reaches its poses.

## 12. Performance

The first headless benchmark gave 48.9 ms per frame (0.34× real time). Profiling showed the robot's cost is
mostly per-frame USD write-back of 52 moving bodies, not the solver. **Fabric** (Isaac Sim's own
`isaacsim.exp.full.fabric.kit` setup) plus **240 Hz** physics brought it to **27.6 ms (0.60×)**, at a cost of
about 2 percentage points of tracking compared with 480 Hz. With Fabric, poses must be read through the
physics API: `XformPrim` returned stale USD values, which had broken the first grasp test.

## 13. Grasping and TF

* Grasp poses solved with FK including the finger meshes. The first `pick` guess left the fingertips 4.2 cm
  above the floor. `tests/test_pick.py` now opens, descends, closes on a 3 cm block (fingers stop at
  −0.23 rad), lifts it to 5.8 cm and carries it at 19.8 cm.
* `/tf` changed from a flat tree to the **URDF tree** (12 transform-tree nodes). The test composes `/tf` and
  matches URDF FK to under 0.05 mm.

## 14. Final state

| Check | Result |
|---|---|
| Build checks (`configure_robot.py`) | 9 / 9 |
| End-to-end ROS 2 test | 35 / 35 |
| Grasp test | 3 / 3 |
| Twin + battery + robot_description | 5 / 5 |
| GUI launcher | scene opens, runtime starts, 18 / 18 topics seen by an external ROS 2 program, ~34 FPS |

## 15. Sources and testing guide (follow-up request, 3 November 2026)

> **Request:** put all the sources used into the repository, and explain how to test that it works (basic).

* **`sources/`:** the inputs copied unchanged, so the repository builds on its own. These are Hiwonder's
  `rosorin_description`, the four RosOrinProMesh files the build reads or checks against, and the feature list
  with the real robot's topic screenshots. `config/rosorin_pro_sim.yaml` now points there. The copies match the
  originals byte for byte, and rebuilding `description/` from them gives an identical URDF and meshes.
  Hiwonder publishes the package on GitHub ([Hiwonder/ROSOrin-Pro](https://github.com/Hiwonder/ROSOrin-Pro),
  also with no license). 48 of its 52 files are identical to the robot's copy; the 4 that differ are not used.
* **[SOURCES.md](SOURCES.md):** every specification with the page it came from and where it is used, plus
  the Isaac Sim 6.2 files used as API references.
* **[TESTING.md](TESTING.md):** five basic checks with real expected output (GUI, topics, sensors,
  driving and arm, automated test), checking from another computer, and troubleshooting.
* **New tool `ros2_tools/sensor_snapshot.py`:** one message per sensor, with PASS/FAIL checks and PNG
  pictures of the camera, depth and lidar. The lidar picture is drawn in the robot frame through TF, because
  the SCLIDAR frame is turned 180°. It needs no imaging library and runs on the real robot too.
* **Re-verified on 2026-11-03:**

  | Check | Result |
  |---|---|
  | `check_topics.py` | 18/18 topics |
  | `sensor_snapshot.py` | 6/6 sensors |
  | `arm.py` | poses within 0.017 rad |
  | `demo_square.py` | 8 mm, −0.9° |
  | `run_tests.bat` | 35/35 in 124 s |
* **Found while writing the test guide:** in the arena about 150 of the 400 lidar beams return nothing.
  A ray cast of the URDF meshes shows that the beam, placed at the URDF's `lidar_frame` on the top face of the
  lidar housing, grazes the chassis. The data is in [DECISIONS.md](DECISIONS.md) § 8, and the item is added
  to the open items below.

## Not done / open items

* **Not tested against the physical robot.** No robot topics were visible on the network during this
  session. Twin mode was tested with a fake robot. To test, run twin mode with the robot on its domain.
* Masses, the init pose and the battery current model are estimates. Calibrate them against the real
  robot (weigh the arm, read `/joint_states` after `/init_pose`, log `/ros_robot_controller/battery`).
* Hiwonder's custom-message topics (servo controller, RGB, buzzer, OLED, buttons) would need their
  `ros_robot_controller_msgs` built for Windows. The standard-message equivalents are documented.
* The real lidar's angle convention (COIN-D6 rotates clockwise) and the depth encoding of the Aurora driver
  should be confirmed with a recording from the robot, e.g. `ros2 bag record /scan /depth_cam/depth0/image_raw`.
* **Lidar beam height:** the simulated beam sits on top of the lidar housing (URDF `lidar_frame`) and
  ~150/400 beams hit the chassis. Run `ros2_tools/sensor_snapshot.py` on the real robot and compare
  `scan.png`. If the real scan has fewer gaps, lower the emitter, which is the `coin_d6` translate in
  `scripts/configure_robot.py`. About 16 mm lower leaves only the 64–69 beams blocked by the structure behind it.
* The physics runs at 0.5–0.6× real time on this PC's CPU (Xeon, Cascade Lake). Use `use_sim_time:=true`.
