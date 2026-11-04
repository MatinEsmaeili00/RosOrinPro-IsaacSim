# ROS 2 interface of the simulated ROSOrin Pro

The names match the real robot's topics (from
[Robot-ROSOrin-Pro/Features](https://github.com/MatinEsmaeili00/Robot-ROSOrin-Pro)), so robot software runs
against the simulator unchanged. All names are **relative**, so a node namespace on the graphs namespaces the
whole robot. With the default empty namespace they resolve to the names below.

* Middleware: ROS 2 **Humble**, `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0` by default (see `launch/rosorin_env.bat`).
  To use Jazzy, `set ROS_DISTRO=jazzy` before launching.
* QoS (measured with `ros2_tools/qos_info.py`): every publisher is **RELIABLE / VOLATILE**, except
  `/robot_description`, which is TRANSIENT_LOCAL (latched). Reliable and best-effort subscribers both connect.
* Rates are in **simulation time**. The simulator steps 1/60 s per frame and runs at about 0.5–0.6× real time
  on this PC, so use `use_sim_time:=true`.

## Published by the simulator

| Topic | Type | Rate (sim) | `frame_id` | Notes |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 60 Hz | – | simulation time, resets when the simulation stops |
| `/tf` | `tf2_msgs/TFMessage` | 60 Hz | – | URDF tree (below) + `odom → base_footprint` |
| `/odom` | `nav_msgs/Odometry` | 60 Hz | `odom` → `base_footprint` | ground-truth physics odometry, starts at 0 when Play is pressed |
| `/odom_raw` | `nav_msgs/Odometry` | 60 Hz | `odom` → `base_footprint` | same data (the real robot publishes the raw encoder odometry here) |
| `/joint_states` | `sensor_msgs/JointState` | 60 Hz | – | 51 joints: `joint1..5`, `r_joint`, 5 gripper mimics, 4 wheels, 36 rollers |
| `/controller_manager/joint_states` | `sensor_msgs/JointState` | 60 Hz | – | same data |
| `/imu` | `sensor_msgs/Imu` | 60 Hz | `imu_link` | orientation, angular velocity, linear acceleration (gravity included: +9.81 on z at rest) |
| `/imu_corrected` | `sensor_msgs/Imu` | 60 Hz | `imu_link` | no orientation (like a calibrated raw IMU) |
| `/ros_robot_controller/imu_raw` | `sensor_msgs/Imu` | 60 Hz | `imu_link` | no orientation |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | `lidar_frame` | COIN-D6: 400 beams, 0.9°, angle −π…π, 0.05–12 m |
| `/scan_raw` | `sensor_msgs/LaserScan` | 10 Hz | `lidar_frame` | same data |
| `/depth_cam/rgb/image_raw` | `sensor_msgs/Image` (`rgb8`) | 15 Hz | `depth_cam_frame` | 640×400, HFOV 71°, auto exposure |
| `/depth_cam/rgb/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `depth_cam_frame` | fx = fy = 448.6, cx = 320, cy = 200, no distortion |
| `/depth_cam/depth0/image_raw` | `sensor_msgs/Image` (`32FC1`) | 15 Hz | `depth_cam_frame` | meters, valid 0.15–3 m (outside: no return) |
| `/depth_cam/depth0/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `depth_cam_frame` | same intrinsics (depth is registered to RGB) |
| `/depth_cam/depth0/points` | `sensor_msgs/PointCloud2` | 15 Hz | `depth_cam_frame` | 640×400 points, xyz |
| `/depth_cam/iro/image_raw` | `sensor_msgs/Image` (`rgb8`) | 15 Hz | `depth_cam_frame` | IR stand-in: rendering limited to the 0.15–3 m depth range |
| `/ros_robot_controller/battery` | `std_msgs/UInt16` | 1 Hz | – | battery voltage in **mV** (runtime, see below) |
| `/robot_description` | `std_msgs/String` | latched | – | the simulation URDF (runtime) |

`/ros_robot_controller/battery` and `/robot_description` come from the Python runtime
(`launch/rosorin_runtime.py`). They are present with `launch\run_isaacsim.bat` and `launch\run_headless.bat`,
but not when you open the scene in a plain Isaac Sim. Everything else lives in the OmniGraph graphs of
`usd/rosorin_pro.usd` and works whenever the scene plays with the ROS 2 bridge loaded.

## Subscribed by the simulator

| Topic | Type | Effect |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | body velocity: `linear.x` forward, `linear.y` left (strafe), `angular.z` yaw (REP-103) |
| `/controller/cmd_vel` | `geometry_msgs/Twist` | same (the topic Hiwonder's Gazebo plugin listens to) |
| `/app/cmd_vel` | `geometry_msgs/Twist` | same (the topic the Hiwonder app uses) |
| `/joint_command` | `sensor_msgs/JointState` | joint targets by name: `position` (rad) for `joint1..joint5` and `r_joint`; `velocity` / `effort` are also accepted |

* The last message on any of the three `cmd_vel` topics wins, and the command is **held until the next one**.
  Publish a zero Twist to stop (the teleop does this on exit).
* Velocities beyond the motors' limit (110 rpm, 11.5 rad/s, about 0.46 m/s) are clipped by the wheel joints.
* `/joint_command` replaces Hiwonder's `/servo_controller` and `/ros_robot_controller/bus_servo/*`, which
  use custom message types that are not available here. Gripper: `r_joint` +0.9 = open (58 mm),
  −0.65 = closed. Only `r_joint` is commanded; the other five gripper joints follow it as mimic joints.

Named arm poses (radians, used by `ros2_tools/arm.py` and the teleop):

| Pose | joint1 | joint2 | joint3 | joint4 | joint5 | Meaning |
|---|---|---|---|---|---|---|
| `init` | 0 | 0 | 1.5708 | 0.7854 | 0 | start pose: wrist camera 0.28 m high, looking 45° down, sees the floor 0.26–0.69 m ahead |
| `pregrasp` | 0 | 1.4399 | 0.2967 | 1.3614 | 0 | gripper pointing down, 7 cm above the pick point |
| `pick` | 0 | 2.0246 | 0 | 0.9599 | 0 | fingertips 6 mm above the floor, 0.222 m in front of `base_footprint` |
| `up` | 0 | 0 | 0 | 0 | 0 | URDF zero pose, arm straight up |

## TF tree

Published on `/tf` with the same parent/child structure as the URDF (that is, as `robot_state_publisher`
publishes it on the real robot). Verified against URDF forward kinematics to within 0.05 mm:

```
odom
└── base_footprint
    └── base_link
        ├── lidar_frame              (yaw = pi: SCLIDAR / COIN-D6 mounting)
        ├── imu_link
        ├── left_front_wheel_link / left_back_wheel_link / right_front_wheel_link / right_back_wheel_link
        └── link1
            └── link2
                └── link3
                    └── link4
                        ├── camera_link0
                        │   └── camera_link
                        │       └── depth_cam_frame   (optical frame: z forward, x right, y down)
                        └── link5
                            ├── end_effector_link
                            └── gripper_link
                                ├── l_link, r_link
                                ├── l_in_link ── l_out_link
                                └── r_in_link ── r_out_link
```

* There is no `map → odom`. That comes from SLAM or AMCL on the ROS side, as on the real robot.
* Static transforms are also published on `/tf` at 60 Hz. There is no separate `/tf_static`.
* Do **not** run a `robot_state_publisher` for the same robot on the same domain: two publishers for the same
  frames confuse TF.

## Differences from the real robot (on purpose or unavoidable)

| Item | Real robot | Simulation |
|---|---|---|
| Odometry | wheel encoders + IMU, drifts | exact physics pose |
| `/joint_states` | arm and gripper joints | also wheels, rollers and gripper mimics |
| Depth encoding | driver-specific (probably `16UC1` in mm) | `32FC1` in m |
| IR stream | real IR camera | stand-in rendering |
| Servo, LED, RGB, buzzer, OLED, button and sbus topics | `ros_robot_controller_msgs` | not available |
| `/set_odom`, `/set_pose` | resets odometry | not implemented (stop and restart the simulation to reset `/odom`) |
| `cmd_vel` timeout | firmware-dependent | none, the last command is held |
