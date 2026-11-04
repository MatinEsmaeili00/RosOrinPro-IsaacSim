# RosOrinProMesh

This repo holds a Blender model and an Unreal-ready FBX of the **Hiwonder ROSOrin Pro**: a mecanum
chassis with a 5-DOF arm, gripper, wrist depth camera and lidar. The model is built straight from the
`rosorin_description` URDF/Xacro package and is meant to be a ROS digital twin in **Unreal Engine**:

* move the whole robot from `/odom`
* drive the arm joints from `/joint_states`
* spin each wheel on its own
* control the gripper on its own

Nothing was placed by eye. Every object is a URDF link, and every local transform is the exact URDF
joint `<origin>`. Each joint pivot is the URDF joint origin and each rotation axis is the URDF `<axis>`.
The build script checks all of this automatically (28 checks). It was also test-imported into
**Unreal Engine 5.8.3** (9 more checks). All checks pass.

![ROSOrin Pro, URDF zero pose](docs/images/overview_rest_pose.png)

| Gripper and wrist camera (4-bar linkage, mimic joints) | Arm moved through the URDF axes (test pose, not saved) |
|---|---|
| ![gripper](docs/images/gripper_camera_closeup.png) | ![pose test](docs/images/arm_pose_test.png) |

## Files

| File | What it is |
|---|---|
| `ROSOrinPro_Unreal.blend` | Blender 5.2 project. Units are meters and every value matches the URDF (1 BU = 1 m, ROS axes). |
| `ROSOrinPro_Unreal.fbx` | Unreal-ready FBX 7.4 in **centimeters**, Z-up, with the full hierarchy. Moving parts are separate nodes. |
| `ROSOrinPro_Export_Report.txt` | Full report: hierarchy, link→mesh map, joints, axes, origins (ROS and UE), conversion, FBX settings, verification, warnings, Unreal test. |
| `ROSOrinPro_joint_map.json` | Machine-readable joint table for the Unreal side: parent/child, type, axis (ROS and UE), rest transforms (m/rad and cm/deg/quat), limits, mimic, wheel kinematics. |
| `rosorinpro_resolved.urdf` | The flattened URDF the model was built from (xacro resolved with `MACHINE_TYPE=ROSOrin_Mecanum_Pro`, `LIDAR_TYPE=MS200`). |
| `scripts/build_rosorinpro_blender.py` | Reproducible build: xacro → Blender → verification → `.blend` / `.fbx` / report / joint map. |
| `scripts/ue_import_test.py` | Headless Unreal test: imports the FBX with Interchange and checks names, hierarchy, poses and scale against URDF FK. |
| `scripts/ue_test_project/` | Minimal content-only `.uproject` used by the Unreal test. |
| `verification/ue_import_result.json` | Raw result of the Unreal 5.8 import test. |

## Robot structure

`base_footprint` is the root. There are 24 objects: 20 URDF links (16 meshes, 4 empty frames) plus 4 wheel links derived from other project files ([see below](#decisions-and-warnings)).

```
base_footprint (empty)                           <- drive from /odom
`-- base_link (mesh)            base_joint [fixed]
    |-- imu_link (empty)            imu_joint [fixed]
    |-- lidar_frame (mesh)          lidar_joint [fixed]
    |-- left_front_wheel_link       left_front_wheel  [continuous, derived]
    |-- left_back_wheel_link        left_back_wheel   [continuous, derived]
    |-- right_front_wheel_link      right_front_wheel [continuous, derived]
    |-- right_back_wheel_link       right_back_wheel  [continuous, derived]
    `-- link1                       joint1 [revolute]
        `-- link2                   joint2 [revolute]
            `-- link3               joint3 [revolute]
                `-- link4           joint4 [revolute]
                    |-- camera_link0            camera_connect_joint [fixed]
                    |   `-- camera_link         camera_joint [fixed]
                    |       `-- depth_cam_frame (empty)  depth_cam_joint_sim [fixed]
                    `-- link5                   joint5 [revolute]
                        |-- end_effector_link (empty)    end_effector_joint [fixed]
                        `-- gripper_link                 gripper_joint [fixed]
                            |-- r_link        r_joint     [revolute]  <- the only independent gripper joint
                            |-- l_link        l_joint     [mimic r_joint x1]
                            |-- l_in_link     l_in_joint  [mimic r_joint x-1]
                            |   `-- l_out_link   l_out_joint [mimic r_joint x1]
                            `-- r_in_link     r_in_joint  [mimic r_joint x1]
                                `-- r_out_link   r_out_joint [mimic r_joint x-1]
```

### Links and meshes

| Link | Visual mesh (from the URDF) | Triangles |
|---|---|---|
| base_link | `meshes/pro/mecanum/base_link.stl` (wheels split out, see below) | 72,354 |
| left/right_front/back_wheel_link | split out of `base_link.stl` | ≈20.9k each |
| lidar_frame | `meshes/pro/common/lidar_link.stl` | 2,822 |
| link1 … link5 | `meshes/pro/arm/link1…5.stl` | 7,827 / 9,074 / 9,093 / 6,008 / 631 |
| camera_link0 | `meshes/pro/arm/camera_connect_link.stl` | 218 |
| camera_link | `meshes/pro/arm/camera_link.stl` | 6,212 |
| gripper_link | `meshes/pro/arm/gripper_link.stl` | 5,768 |
| l_link, r_link, l_in_link, r_in_link, l_out_link, r_out_link | `meshes/pro/gripper/*.stl` | 1,208 / 1,320 / 871 / 876 / 937 / 938 |
| base_footprint, imu_link, end_effector_link, depth_cam_frame | none (Empty frame) | – |

All 16 referenced STLs were imported 1:1. The triangle counts and vertex bounds equal the STL files
exactly: the STLs are already in meters, so no scale was applied.

### Movable joints

| Joint | Parent → child | Axis (URDF, joint frame) | Limits [rad] |
|---|---|---|---|
| joint1 | base_link → link1 | −5.4513e-05 −0.0087266 −0.99996 | ±2.09 |
| joint2 | link1 → link2 | 0 0.99996 −0.0087265 | ±2.09 |
| joint3 | link2 → link3 | 0 1 0 | ±2.09 |
| joint4 | link3 → link4 | 0 1 0 | ±2.09 |
| joint5 | link4 → link5 | 0 0 −1 | ±2.09 |
| r_joint | gripper_link → r_link | 0.99998 1.524e-05 −0.0060469 | ±1.57 |
| l_joint, l_in_joint, l_out_joint, r_in_joint, r_out_joint | gripper chain | see report | ±1.57, mimic r_joint |
| *_wheel (4×) | base_link → *_wheel_link | 0 1 0 | continuous (derived) |

Every origin transform, in both ROS units (m, rad) and Unreal units (cm, FRotator, quaternion), is
listed in section 4 of `ROSOrinPro_Export_Report.txt` and in `ROSOrinPro_joint_map.json`.

## Coordinate conversion (Blender/ROS → Unreal)

| | Blender / ROS (REP-103) | Unreal |
|---|---|---|
| Handedness | right-handed | left-handed |
| Axes | +X forward, +Y left, +Z up | +X forward, +Y right, +Z up |
| Unit | 1 m | 1 cm |

The FBX is written with `UnitScaleFactor = 1` (cm), Up = +Z, Front = −Y. That is exactly the axis
system Unreal's importer converts to, so Unreal adds no root rotation or scale. It only flips Y:

```
location   UE(x, y, z) [cm]  = (100·x, −100·y, 100·z)      ROS (x, y, z) in m
rotation   UE FRotator(P, Y, R) = (−pitch, −yaw, +roll)    URDF rpy, in degrees
quaternion UE (X, Y, Z, W)   = (−qx, qy, −qz, qw)
joint axis UE axis           = (−ax, ay, −az);  angle = the /joint_states position, sign unchanged
```

### FBX export settings (Blender 5.2 `export_scene.fbx`)

`use_selection=True`, `object_types={EMPTY, MESH}`, `apply_unit_scale=True`,
`apply_scale_options=FBX_SCALE_NONE`, `global_scale=1.0`, `axis_forward=Y`, `axis_up=Z`,
`bake_space_transform=False`, `mesh_smooth_type=FACE`, `use_triangles=True`, `use_custom_props=True`,
`add_leaf_bones=False`, `bake_anim=False`.

For the length of the export the scene is switched to 1 BU = 1 cm: unit scale 0.01, with every vertex
and local translation ×100. The exporter's global matrix is then identity and the file stores plain
centimeters with scale 1 on every node. The meter values are restored bit-exactly afterwards and
re-verified. Blender's *Apply Transform* (`bake_space_transform`) option is deliberately **off**: in
Blender 5.2 it corrupted the nested hierarchy (alternating 0.01/1 node scales). The build script's FBX
read-back check caught this.

## Unreal import

1. **File → Import Into Level…** → `ROSOrinPro_Unreal.fbx` (Interchange, the UE 5.x default).
2. Keep **Force Front X Axis = OFF**, **Convert Scene = ON**, **Import Uniform Scale = 1.0**.
   *Convert Scene Unit* makes no difference because the file is already in cm. For a legacy Content
   Browser import also set *Combine Meshes = OFF*, *Transform Vertex To Absolute = OFF* and
   *Bake Pivot In Vertex = OFF*.
3. The result is one actor per URDF link, named exactly like the link and attached like the URDF tree.
   The actors sit under an identity `RootNode` actor that Interchange creates for the FBX scene root.
   Static mesh assets are named `SM_<link>` (UE 5.8 appends `_001`, which is cosmetic), and there are
   two materials, `M_rgba_cad1ee` (URDF blue-grey) and `M_black` (lidar).

Tested headless in **UE 5.8.3**. All 24 actors match URDF FK to **1.3e-5 cm** and **8e-6°**, with
correct attachment and scale 1 everywhere. Details are in section 8 of the report.

### Driving it from ROS

```cpp
// once, after spawning: remember each link's imported (rest) relative rotation
RestQuat[Link] = LinkComp->GetRelativeRotation().Quaternion();

// /joint_states: for each joint name -> child link (ROSOrinPro_joint_map.json)
const FVector AxisUE(-ax, ay, -az);                     // or read "ue_axis" from the JSON
LinkComp->SetRelativeRotation(RestQuat[Link] * FQuat(AxisUE.GetSafeNormal(), PositionRad));

// gripper: if only r_joint is published, apply the mimic joints yourself
q_child = multiplier * q_r_joint + offset;              // l_joint x1, l_in x-1, l_out x1, r_in x1, r_out x-1

// /odom (odom -> base_footprint), meters -> cm
Root->SetWorldLocationAndRotation(FVector(100*p.x, -100*p.y, 100*p.z), FQuat(-q.x, q.y, -q.z, q.w));
```

`FQuat A * B` applies B first, so `Rest * Joint` rotates about the axis in the link's own frame. That
is exactly the URDF convention `T = origin · R(axis, q)`. Because component names equal ROS frame
names, you can alternatively apply `/tf` transforms directly to each component's relative transform.

**Wheels** are not in `/joint_states` (see below). Spin them from the `/odom` twist `(vx, vy, wz)`
using the mecanum kinematics derived from the wheel geometry. The roller directions were measured
from the mesh:

```
omega_i = (vx − wz·y_i + s_i·(vy + wz·x_i)) / r          r = 0.040 m, rotate about the wheel's +Y (UE axis (0,1,0))
wheel            x_i [m]   y_i [m]   s_i
left_front      0.09097   0.08716   −1
left_back      −0.08068   0.08696   +1
right_front     0.09097  −0.08701   +1
right_back     −0.08068  −0.08722   −1
```

Positive rotation about +Y rolls the robot forward on both sides.

## Verification (automatic, in `build_rosorinpro_blender.py`)

* Every referenced visual mesh exists and was imported onto its link. Triangle counts and vertex
  bounds equal the STL files, and none is missing.
* The object set equals the URDF links plus the 4 documented wheel links. Every parent matches the
  URDF and there are no hidden parent-inverse offsets.
* Pivots: object world transforms match an independent URDF forward-kinematics computation
  (≤4e-8 m, ≤1.5e-7 rad).
* Axes: each of the 15 movable joints is rotated by 0.35 rad and the measured world axis is compared
  with the URDF `<axis>` (≤7e-7 rad). The pivot stays fixed during rotation (≤1.3e-8 m).
* Scale: metric scene, 1 BU = 1 m, every object scale is 1. The robot measures
  0.284 × 0.212 × 0.545 m with the arm straight up.
* Duplicates: no `.001` names, no stray objects (default Cube, Camera and Light deleted), no shared or
  duplicated meshes, no identical geometry placed twice.
* FBX read-back: cm units, Z-up/−Y-front axes, all 24 nodes present once, hierarchy intact, world
  transforms equal URDF FK (1.3e-7 m), node scale 1, no pre/post rotations.
* Unreal 5.8 import: see above.

## Decisions and warnings

These are summarized from the report; section 7 has the full text.

1. **Lidar variant is ambiguous.** `LIDAR_TYPE` comes from an environment variable that the package
   never defines. MS200, LD19 and SCLIDAR share the mesh and position and differ only in
   `lidar_joint` yaw (0, 3.1416 and π). This build uses **MS200 (yaw 0)**. Rebuild with
   `--lidar LD19` or `--lidar SCLIDAR` if your robot has one of those.
2. **The Pro URDF has no wheel links.** The wheels are baked into `base_link.stl`. They were split out
   losslessly: whole loose parts only, 29 per wheel (hub, cap, 9 rollers, roller pins). The motors
   stay in the chassis. The wheel names and frames come from the project's own `urdf/rosorin_old.xacro`
   (same chassis); the frames match the wheel geometry to within 0.2 mm and 0.1°. They are typed
   `continuous` about +Y and are **not** in `/joint_states`.
3. **The gripper is a closed-chain linkage** modelled with mimic joints. Only `r_joint` is independent.
4. URDF values are used literally, including rounded π values such as `-3.1416 -1.5681 3.1416`.
   Slightly non-unit axes are normalized, as ROS/KDL does.
5. `common.urdf.xacro` (which references non-existent `back_shell_*.stl`) and `connect.urdf.xacro`
   are commented out in `rosorinpro.xacro`. They are not part of the Pro model, so nothing is missing.
6. The non-Pro chassis meshes (`base_link_ack/diff/mec.stl`) were intentionally not imported.
7. The collision meshes are identical to the visual meshes and were not exported separately.
8. The model rests 1.7 mm above `base_footprint` (base_joint z = 0.025 m vs the lowest vertex at
   −0.0233 m). This is kept as the URDF defines it.
9. Unreal logs informational "degenerate tangent" messages for a few small CAD meshes. They are
   harmless.

## Rebuilding

The script expects the `rosorin_description` package next to this repo (`../rosorin_description`), or
you can pass `--pkg`.

```bat
:: headless
blender -b --factory-startup --python scripts\build_rosorinpro_blender.py -- --pkg ..\rosorin_description --out . --overwrite
:: other lidar
blender -b --factory-startup --python scripts\build_rosorinpro_blender.py -- --pkg ..\rosorin_description --out . --lidar LD19 --overwrite
```

The same file also runs through the Blender MCP or the Python console. Set a `ROSORIN_CFG` dict and
`exec()` the script, which is how this model was built.

Unreal test:

```bat
set ROSORIN_FBX=%CD%\ROSOrinPro_Unreal.fbx
set ROSORIN_JOINT_MAP=%CD%\ROSOrinPro_joint_map.json
set ROSORIN_UE_RESULT=%CD%\verification\ue_import_result.json
"C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" scripts\ue_test_project\ROSOrinImportTest.uproject -run=pythonscript -script="%CD%\scripts\ue_import_test.py" -unattended -nullrhi -nosplash
```

## Source

The robot description (URDF/Xacro and STL meshes) is Hiwonder's `rosorin_description` ROS 2 package.
The meshes in the `.blend` and `.fbx` come from it unchanged, apart from the lossless wheel split.
This repository adds the Blender/Unreal conversion, the verification and the documentation.
