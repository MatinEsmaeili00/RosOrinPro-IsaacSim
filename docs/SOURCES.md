# Sources

Everything this simulation was built from: input files, the real robot's specifications, and the Isaac Sim
references. Values that are estimates rather than sourced are marked as such in
`config/rosorin_pro_sim.yaml` and explained in [DECISIONS.md](DECISIONS.md).

## 1. Input files (in this repository)

Copied unchanged into [`sources/`](../sources/README.md), with their origin and the checks done when copying:

| Input | Origin |
|---|---|
| `sources/rosorin_description/`: Hiwonder description package (xacro, STL) | the robot's copy; upstream [Hiwonder/ROSOrin-Pro](https://github.com/Hiwonder/ROSOrin-Pro) `ROS2/src_ros2/src/simulations/rosorin_description` |
| `sources/RosOrinProMesh/rosorinpro_resolved.urdf`: flattened Pro URDF, plus the Blender build script, joint map and export report | [MatinEsmaeili00/RosOrinProMesh](https://github.com/MatinEsmaeili00/RosOrinProMesh) @ `ef71eab` |
| `sources/Robot-ROSOrin-Pro/Features/`: feature list and the real robot's ROS 2 node and topic list | [MatinEsmaeili00/Robot-ROSOrin-Pro](https://github.com/MatinEsmaeili00/Robot-ROSOrin-Pro) @ `42c5dbe` |
| `RosOrin_Claude.usd`: your Isaac Sim scene, before any change | backup in `scenes/RosOrin_Claude.original.usda` |

## 2. Real robot specifications

Pages read on 2026-11-03. "Used as" points to where the number ends up in the simulation.

| Item | Value | Used as | Source |
|---|---|---|---|
| Weight | Ultimate kit 3.6 kg (Standard 3.1, Advanced 3.2) | total of `masses` in the config | [Hiwonder ROSOrin Pro product page](https://www.hiwonder.com/products/rosorin-pro) |
| Size | 284 × 212 × 544 mm (Ultimate) | check only: the URDF matches | product page |
| Lidar model | COIN-D6, ToF, 360°, 12 m | `lidar_variant: SCLIDAR`, sensor model | product page; node `/sclidar_scan_publisher` in the [feature list](../sources/Robot-ROSOrin-Pro/Features/README.md) |
| Lidar performance | 4000 samples/s, 10 Hz, 0.9°, 0.05–12 m, ±10 mm (0.05–1 m) / ±40 mm (1–12 m), rotates clockwise, UART 230400 | `sensors.lidar`, RTX lidar settings | [COIN-D6 on a reseller page (rcdrone.top)](https://rcdrone.top/products/witmotion-dtof-lidar). **Not the manufacturer's datasheet** (none was found). |
| Depth camera | Aurora 930 Pro: depth, RGB and IR at 640×400 (also 320×240), 5–15 fps, H71° × V46°, 30–300 cm (ROSOrin Pro manual: 15–300 cm), 8 mm at 1 m, 40 mm baseline, USB 2.0 | `sensors.depth_camera`: 640×400, HFOV 71°, 15 fps, 0.15–3 m | [Hiwonder Aurora 930 Pro docs](https://docs.hiwonder.com/projects/Aurora930-Pro/en/latest/docs/1.Introduction_to_Depth_Camera.html), [Aurora 930 Pro in ROS](https://wiki.hiwonder.com/projects/Aurora930-Pro/en/latest/docs/2.Configuration_and_Usage_in_ROS.html) (topics). This is Hiwonder's documentation, not a Deptrum datasheet. |
| Drive motors | 12 V, 1:90, 110 rpm no-load (85 rpm rated), stall torque 15 kg·cm, Hall encoder (11 lines) | wheel `max_velocity` 11.52 rad/s (= 110 rpm), `max_effort` 1.47 N·m (= 15 kg·cm) | [ROSOrin Pro user manual](https://wiki.hiwonder.com/projects/rosorin-pro/en/latest/docs/1_ROSOrin_Pro_User_Manual.html) |
| Arm servos | HX-12H: 12 kg·cm stall torque at 11 V, 0.2 s/60°, 0–240°, 0.3° accuracy, 36.3 g | arm `max_effort` 1.18 N·m, `max_velocity` 5.24 rad/s, limits ±2.09 rad, link mass ≥ servo mass | user manual |
| Battery | 11.1 V 6000 mAh (3S), charges to 12.6 V, ≤ 10 A, buzzer warning below 10 V | `battery` model: 12.6 V full, 10.0 V warning | user manual |
| IMU | MPU6050 (6-axis, I2C) | IMU sensor on `imu_link` | user manual |
| Controller board and UI | STM32F407, buzzer, LED, KEY1 button, 0.91" OLED, PS2 receiver | not simulated (hardware only); listed in the README feature table | user manual |
| Speeds | no rated top speed published. Nav2 example 0.5 m/s, "under 0.3 m/s" for SLAM, voice-control default 0.2 m/s | teleop default 0.2 m/s; tests use 0.2 m/s | [Hiwonder forum: ROSOrin Pro pipeline](https://forum.hiwonder.com/t/rosorin-pro-explained-the-full-ros-2-ai-agent-pipeline-openclaw-slam-yolo-nav2-and-moveit2/272), [Large AI Model courses](https://wiki.hiwonder.com/projects/rosorin-pro/en/latest/docs/12_Large_AI_Model_Courses.html) |

The two camera ranges disagree: the Aurora docs say 30–300 cm, the ROSOrin Pro manual says 15–300 cm. The
simulation uses 0.15 m as the near limit (see [DECISIONS.md](DECISIONS.md) § 8). Hiwonder's ROSOrin Pro
listings say 12 fps and the camera docs 5–15 fps; the simulation runs at 15 fps.

Also seen, but no numbers taken from them: reseller listings of the kit variants
([OpenELAB](https://openelab.io/products/hiwonder-rosorin-pro-smart-car),
[OpenELAB kit guide](https://openelab.io/blogs/learn/rosorin-pro-ros2-robot-kit-guide),
[Zeta42](https://www.zeta42.com/robots/hiwonder/rosorin-pro)).

## 3. Your repositories

* [Robot-ROSOrin-Pro](https://github.com/MatinEsmaeili00/Robot-ROSOrin-Pro): the 19 feature groups and about
  50 topics observed on the real robot. These set the scope and every topic name
  ([ROS2_INTERFACE.md](ROS2_INTERFACE.md)).
* [RosOrinProMesh](https://github.com/MatinEsmaeili00/RosOrinProMesh): the flattened URDF, the wheel-split rule
  (`triangle_components()` in `scripts/build_rosorinpro_blender.py`) and the measured roller geometry.

## 4. Isaac Sim 6.2 references

Isaac Sim **6.2.0** (`6.2.0-rc.14+release.52805`), standalone, at `C:\isaac-sim-standalone-6.2.0`. The APIs
were taken from the installed sources, because some online examples are older than 6.2.

| File in the Isaac Sim install | Used for |
|---|---|
| `exts/isaacsim.ros2.nodes/isaacsim/ros2/nodes/impl/graph_api.py` | reference for every ROS 2 OmniGraph node type, input name and connection in `scripts/ros_graphs.py` |
| `exts/isaacsim.ros2.nodes/isaacsim/ros2/nodes/nodes/OgnROS2RtxLidarHelper.py` | lidar → `LaserScan` helper (`/scan`, `/scan_raw`) |
| `exts/isaacsim.ros2.core/` (`humble/`, `jazzy/`, bundled `rclpy`), `setup_ros_env.bat` | ROS 2 environment on Windows: Humble + FastDDS instead of the default Jazzy + Zenoh; standalone `rclpy` for `ros2_tools/` |
| `exts/isaacsim.asset.importer.urdf/` | `URDFImporter` / `URDFImporterConfig` (`scripts/import_to_usd.py`) |
| `exts/isaacsim.core.experimental.prims/` | `Articulation`, `RigidPrim` (runtime, tests) |
| `exts/isaacsim.core.simulation_manager/.../simulation_manager.py` | physics step callbacks (`launch/rosorin_runtime.py`) |
| `exts/isaacsim.sensors.experimental.physics/.../imu.py` | IMU sensor |
| `exts/isaacsim.sensors.experimental.rtx/`, `extscache/omni.sensors.nv.common-3.4.0/` | RTX lidar schema (`OmniSensorGenericLidarCoreAPI`, emitter state) for the COIN-D6 |
| `exts/isaacsim.robot.wheeled_robots.nodes/`, `exts/isaacsim.robot.experimental.wheeled_robots/` | `HolonomicController`: evaluated and replaced by the exact mecanum IK ([DECISIONS.md](DECISIONS.md) § 5) |
| `apps/isaacsim.exp.full.fabric.kit` | Fabric settings copied into the runtime (`enable_fabric()`) |
| `skills/isaac-sim-ros2-bridge`, `isaac-sim-sensor`, `urdf-mjcf-to-usd-conversion`, `usd-articulation` (`SKILL.md`) | workflow guides shipped with Isaac Sim. One script in the ROS 2 skill (`multi_robot_namespacing.py`) uses node inputs that 6.2 no longer has, so `graph_api.py` was used instead. |
| `standalone_examples/api/isaacsim.ros2.bridge`, `isaacsim.asset.importer.urdf`, `isaacsim.sensors.experimental.rtx`, `isaacsim.sensors.experimental.physics`, `isaacsim.robot.wheeled_robots.examples` | working examples for the APIs above |

## 5. Worked out here (no external source)

* Mecanum inverse kinematics from the measured roller axes, wheel radius and positions:
  `ω_i = ((vx − ωz·y_i)·ux_i + (vy + ωz·x_i)·uy_i) / (r_i·ux_i)` ([DECISIONS.md](DECISIONS.md) § 5).
* Link masses (sum = 3.6 kg), the init pose and the grasp poses (forward-kinematics searches), the gripper
  calibration from the finger meshes, the contact parameters (tuning sweeps in `tests/tune_mecanum.py`), and the
  battery current model. Each has its data in [DECISIONS.md](DECISIONS.md).
