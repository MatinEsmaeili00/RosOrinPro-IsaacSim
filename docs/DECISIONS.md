# Engineering decisions

Each decision lists what was chosen, why, and the data behind it. Parameters live in
`config/rosorin_pro_sim.yaml`; after changing one, re-run `launch\build_all.bat`.

## 1. Source model and lidar variant

* **Source:** Hiwonder `rosorin_description` with `MACHINE_TYPE=ROSOrin_Mecanum_Pro`. The flattened URDF
  from the RosOrinProMesh build is used as-is, with the same 20 links and the same joint origins and axes.
* **Lidar = SCLIDAR (`lidar_joint` yaw = π), not MS200.** The robot's node list contains
  `/sclidar_scan_publisher`. Hiwonder's spec for the ROSOrin Pro names a **COIN-D6** ToF lidar, and its driver
  is the "sclidar" one. The variants differ only in the mounting yaw (MS200 0, LD19 3.1416, SCLIDAR π).
  RosOrinProMesh had assumed MS200. Change it with `scripts\build_sim_urdf.py --lidar MS200|LD19|SCLIDAR`.

## 2. Masses

The CAD inertials in the URDF are unrealistic: arm links weigh 1–14 g, gripper parts under 1 g, the
chassis 0.81 kg, and `imu_link` carries a copy of the chassis inertial (a copy-paste error that would add
0.81 kg). The robot weighs **3.6 kg** (Hiwonder spec, Ultimate kit). Chosen values:

| Part | Mass | Basis |
|---|---|---|
| base_link (chassis, 4 motors, 3S 6000 mAh battery, Jetson, STM32) | 2.690 kg | remainder to reach 3.6 kg |
| link1 / link2 / link3 / link4 / link5 | 0.090 / 0.080 / 0.080 / 0.070 / 0.045 kg | HX-12H servo 36.3 g (spec) + brackets |
| gripper_link, 6 gripper parts | 0.060 kg, 4–8 g each | estimate |
| camera (Aurora 930 Pro), mount | 0.035, 0.005 kg | estimate |
| lidar (COIN-D6) | 0.045 kg | estimate |
| each wheel hub / roller | 0.045 kg / 0.005 kg | estimate |
| imu_link | removed | massless frame |

The inertia tensors are **scaled with the mass** (`I_new = I_urdf · m_new / m_urdf`). This keeps the CAD
center of mass and mass distribution. The total checks out at 3.600 kg.

## 3. Wheels and rollers

The Pro URDF has no wheel links: the wheels are baked into `meshes/pro/mecanum/base_link.stl`.
`build_sim_urdf.py` uses the same rule as the RosOrinProMesh Blender build: whole loose parts within 45 mm of
a wheel axis, outboard of the wheel frame, with frames from `urdf/rosorin_old.xacro`. The result matches
triangle for triangle (chassis 72,354 triangles; wheels 20,926 / 20,975 / 20,715 / 20,893).

Each wheel is then split again into a **hub and 9 rollers** (the parts reaching the outer radius). Each roller
becomes a link with a **continuous joint about its own axis** (PCA of the roller mesh). Rollers measure 8 mm
radius by 31.5 mm, on a 32 mm ring, giving a 40 mm outer radius. The roller axis at the ground contact is
−44.7°, +44.5°, +44.2° and −44.6° (LF, LB, RF, RB), the same values as the joint map. The envelope of the 9
convex roller hulls is round to within **0.11 mm** as the wheel turns.

Why real rollers rather than "move the base kinematically" (Hiwonder's Gazebo file uses `planar_move`):
strafing and slip come out of the physics; the wheels turn the way the motors command; collisions behave
physically; and the whole robot runs from OmniGraph without a Python script. The articulation has 52 bodies,
under PhysX's limit of 64.

## 4. Mecanum contact tuning

`tests/tune_mecanum.py` commands the wheels with exact IK (bypassing ROS) and measures the body velocity
(mean ± std over 2 s, after 0.5 s of acceleration).

| Variant | Forward (cmd 0.20) | Strafe (cmd 0.20) | Rotate (cmd 1.00) | Yaw-rate noise | Wheel jitter p95 | ms/frame |
|---|---|---|---|---|---|---|
| rigid contacts, 240 Hz (start) | 0.189 | **0.120** | 0.92 | 0.11–0.39 | 6–15 rad/s | – |
| compliant k=2e4, c=150 | 0.193 | 0.173 | 0.950 | 0.03–0.06 | 1.5–3 | – |
| **compliant k=8000, c=100, 240 Hz (chosen)** | **0.193** | **0.177** | **0.945** | 0.02–0.05 | 1.3–1.7 | 27.6 |
| same at 480 Hz | 0.194 | 0.180 | 0.963 | 0.02–0.05 | 0.7–1.2 | 34.1 |
| 480 Hz with 8 solver iterations | 0.191 | 0.169 | 0.908 | – | 1.1–2.1 | 30.1 |
| 240 Hz with 4 solver iterations | 0.176 | 0.131 | 0.772 | – | 2.6–5 | 25.1 |
| + roller armature, + 128-vertex hulls, wheel damping 1, μ = 1.2 | no better | | | | | |

With rigid contacts, the rigid four-wheel chassis keeps lifting one wheel, which then spins freely (16 rad/s
spikes) and loses traction. Rubber-like **compliant contacts** (PhysX `compliantContactStiffness`/`Damping`
on the roller material, about 1 mm of compression) keep all four wheels loaded.

**Chosen: compliant k = 8000 N/m, c = 100 N·s/m, 240 Hz, 32/4 solver iterations.** 480 Hz is about 2
percentage points better and costs 6 ms per frame; set `physics.steps_per_second: 480` for maximum fidelity.

A note on the tuning history: the first sweep looked catastrophic (robots "stuck" with wheels spinning).
Logging the height showed the robot had **driven onto the 3 cm test blocks**. The tests now disable the
blocks, and the blocks were moved off the straight-ahead path.

## 5. Mecanum inverse kinematics (cmd_vel → wheels)

Isaac Sim's `HolonomicController` node solves a QP that treats the body velocity as the **sum** of the
wheel contributions. For this robot it drove at **35 %** of the commanded forward speed and **260 %** of the
yaw rate (forward scale 1/(4·cos 45°) = 0.354). It was replaced by the exact IK. The free bottom roller only
lets the contact point slide across its axis u = (ux, uy), so:

```
omega_i = ((vx − wz·y_i)·ux_i + (vy + wz·x_i)·uy_i) / (r_i · ux_i)
```

`(x_i, y_i)` is the contact point (wheel frame plus roller-ring center) relative to `base_footprint`,
`r_i = 0.040 m`, and `u_i` is the measured bottom-roller axis. In OmniGraph this is **one
`MatrixMultiply` (v·M, row-vector convention, verified)**. The rows for vx, vy, wz are
`[25.0, 24.95, 25.0, 24.95]`, `[−24.73, 24.48, 24.33, −24.61]` and `[−4.42, −4.14, 4.38, 4.15]`. Result
through `/cmd_vel`: forward 0.193, strafe 0.177, rotate 0.941, diagonal (0.144, −0.130) for the commanded
(0.2, 0.2, 1.0, (0.15, −0.15)).

The three `cmd_vel` topics each write a graph variable on message arrival, and the IK reads it every tick.
So the last message wins and the command is held, like Isaac's own examples. There is no timeout.

## 6. Arm, servo drives and init pose

* **HX-12H spec** (Hiwonder manual): 12 kg·cm at 11 V = **1.18 N·m**, 0.2 s per 60° = **5.24 rad/s**,
  0–240°, which matches the URDF limits of ±2.09 rad. Position drives use stiffness 60 N·m/rad, damping
  2 N·m·s/rad, max force 1.18 N·m and max velocity 5.24 rad/s. The steady-state gravity sag is about 1°.
* **Init pose** (`config.init_pose`, applied as the initial joint state and drive targets): the URDF zero
  pose points the arm straight up, so the wrist camera saw only sky (the first depth test failed with 0 %
  valid pixels). The real robot runs `/init_pose` at boot. A forward-kinematics search picked joint2 = 0,
  joint3 = 90°, joint4 = 45°: the camera is 0.28 m high, looks 45° down and sees the floor 0.26–0.69 m
  ahead, and the gripper stays 0.2 m above the floor.
* **Grasp poses** come from FK with the finger meshes. The first guess, joint2 = 105°, put the
  *fingertips* 4.2 cm above the floor, which would close on air above a 3 cm block. Final `pick` =
  (0, 2.0246, 0, 0.9599, 0): fingertips 6 mm above the floor, 0.222 m ahead. `pregrasp` sits 7 cm above it.
  `tests/test_pick.py` lifts and carries a 3 cm block (z 0.015 → 0.058 → 0.198 m).

## 7. Gripper (4-bar linkage)

Only `r_joint` is independent. `l_joint`, `l_in_joint`, `l_out_joint`, `r_in_joint` and `r_out_joint` are
URDF mimics with multipliers +1, −1, +1, +1 and −1. The URDF importer writes them only as Newton mimic
joints, so `configure_robot.py` adds `PhysxMimicJointAPI` with gearing = −multiplier, which encodes the
PhysX constraint `q + gearing·q_ref + offset = 0`. Measured mimic error: 0.0000 rad.

The finger gap was measured from the meshes as a function of `r_joint`: 0 → 37 mm, **+0.9 → 58 mm (open)**,
**−0.65 → fingers touch (closed)**. Positive opens; the placeholder config had it backwards. Self-collision is
off (as with Isaac importer defaults), so `r_joint` is limited to [−0.70, 1.00] rad to keep the fingers from
passing through each other. The finger pads use a friction-1.2 material ("max" combine). The gripper closes
on a 30 mm block at about −0.2 rad.

## 8. Sensors

| Sensor | Model | Simulation |
|---|---|---|
| Lidar | COIN-D6: 360°, 4000 samples/s, 10 Hz, 0.9°, 0.05–12 m, ±10/40 mm | RTX `OmniLidar`: 1 emitter at elevation 0, `patternFiringRateHz = 4000`, `scanRateBaseHz = 10`, accuracy 0.01 m, authored locally (no download needed) |
| Camera | Aurora 930 Pro: RGB/depth/IR 640×400, H71°×V46°, 0.15–3 m, ≤ 15 fps | two USD cameras on `depth_cam_frame`: RGB (clip 0.01–1000 m) and depth (clip 0.15–3 m); focal length 2.524 / aperture 3.6 mm gives HFOV 71.0°; square pixels make the VFOV 48° (spec 46°); `omni:sensor:tickRate = 15` |
| IMU | MPU6050 on the STM32 board | `IsaacImuSensor` on `imu_link`, read every frame |
| Battery | 3S 11.1 V 6000 mAh, charge 12.6 V, buzzer below 10 V | runtime model: idle 1.2 A + 3 A per m/s, R_int 0.05 Ω, linear 12.6 → 9.9 V; `time_scale` for testing |

* **Camera exposure:** the RTX camera's default `exposure:time` of 1 s over-exposed everything (a grey floor
  rendered white). Isaac Sim 6.2's tonemapper ignores the manual exposure values here, so the cameras use
  **RTX auto exposure** (`OmniRtxCameraAutoExposureAPI_1`), like the real camera's AE.
* **Publish rate:** `frameSkipCount` is deprecated in 6.2. With it, the depth stream ran at the frame rate
  while RGB was throttled. Using `omni:sensor:tickRate = 15` on the camera prims gives 15 Hz on every stream.
* **Depth registration:** RGB and depth share one optical frame (depth registered to color). The real module
  has a 40 mm stereo baseline.
* **Lidar beam height (open):** the emitter sits at the `lidar_frame` origin, which Hiwonder's URDF puts
  on the top face of the lidar housing (housing z 0.097–0.132 m above `base_footprint`). At that height the
  beam grazes the chassis, and about 150 of 400 beams return nothing in the test arena (back corners and about
  ±55° to the front). A ray cast of the URDF meshes at lower beam heights gives 265 blocked beams at the top
  face, 162 at 8 mm below it, 106 at 12 mm and 64–69 at 16–24 mm. The remaining 64–69 are the structure
  behind the lidar. The real optical height is not published, so the URDF frame was kept. A `/scan` recorded
  on the real robot will show which sectors it really loses.
* **IR:** Isaac has no IR rendering. `/depth_cam/iro/image_raw` is the rendering of the depth camera, whose
  clipping limits it to 0.15–3 m, much like an active-IR image fading with distance.

## 9. ROS 2 middleware

Isaac Sim 6.2 on Windows defaults to **Jazzy with Zenoh** (`setup_ros_env.bat`), which needs a Zenoh router
and does not interoperate with the Jetson (Ubuntu 22.04, Humble, FastDDS). The bundled **Humble** libraries
work when `ROS_DISTRO=humble`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `PATH` includes
`exts/isaacsim.ros2.core/humble/lib` before launch (`launch/rosorin_env.bat`).

The same bundle provides `rclpy` for standalone tools (`ros2_tools/ros2_env.bat`). The numpy copied into the
bundle has no DLLs outside Isaac Sim, so Isaac's own numpy (`pip_prebundle`) goes first on `PYTHONPATH`.

## 10. TF tree

`IsaacComputeTransformTree` with one parent gives a flat tree (every frame under `base_footprint`). The graph
uses **one compute/publish node pair per URDF parent** (12 groups), so `/tf` has the real robot's structure.
The test composes `/tf` to `end_effector_link`, `depth_cam_frame`, `lidar_frame` and `l_out_link` and
compares with URDF FK at the current joint angles: error below 0.05 mm. The odometry graph does **not**
publish Isaac's default `world → odom`, which would conflict with SLAM's `map → odom`.

## 11. Performance

`tests/bench_perf.py`, headless, ms per simulated frame (1/60 s):

| Configuration | Default app | **Fabric** |
|---|---|---|
| everything, 480 Hz | 48.9 (0.34× real time) | 34.1 (0.49×) |
| everything, **240 Hz (default)** | 41.5 | **27.6 (0.60×)** |
| robot physics only, 480 Hz | 34.3 | 22.8 |
| scene without the robot | 10.0 | 10.6 |

The robot's cost is per-frame USD write-back of 52 moving bodies plus the 36-roller articulation. Solver
iterations barely matter (32 → 8 iterations: −4 ms). **Fabric** (`omni.physx.fabric`, as in
`apps/isaacsim.exp.full.fabric.kit`) removes the USD write-back. The launchers therefore use
`isaac-sim.fabric.bat` / `enable_fabric()` and 240 Hz. One consequence of Fabric: USD transforms of moving
prims are not updated, so read poses through the physics API (`Articulation`, `RigidPrim`). `XformPrim`
returns stale values; this broke the first version of the grasp test.

## 12. Scene building

* The scene only **references** the robot (`./RosOrinPro-IsaacSim/usd/rosorin_pro.usd`) and the arena, so
  rebuilding the robot needs no scene change.
* **Bug found and fixed:** re-running the scene builder removed the robot from the *live* stage. OmniGraph
  then "disconnected" the referenced graphs by writing **connection-delete opinions** into the scene layer,
  and after the robot was re-added every robot graph was silent (the `Tick` node ran 70 times, its downstream
  nodes 0 times). The builder now edits the layer with `Sdf` before opening it, and strips any override
  below `/World/rosorin_pro` afterwards.
* The GUI startup script must enable `isaacsim.ros2.bridge` **before** opening the scene. Otherwise OmniGraph
  logs "Could not find node type interface for isaacsim.ros2.bridge.*" and the graphs never run.

## 13. Digital-twin mode

The first version **teleported** the base to the real robot's odometry pose every physics step. That made
joint2 sag steadily to its limit within 1 s, because overwriting the root pose and velocity 240–480 times a
second fights the arm drives. The base is now **driven through its wheels** (odometry twist feed-forward plus
2/s pose feedback, through the exact IK). The real robot's joint positions become position targets. The real
robot is read on its own ROS domain with a separate `rclpy` context, so the simulator's topics never collide
with the robot's. Result: base 4 mm / 0.014 rad, arm 0.018 rad.
