# Input sources

Every input file the simulation was built from, copied unchanged so the repository rebuilds on its own
(`launch\build_all.bat` reads from here, see `config/rosorin_pro_sim.yaml` → `description:`). Web pages,
specifications and Isaac Sim references are listed in [docs/SOURCES.md](../docs/SOURCES.md).

| Folder | What it is | Where it came from | Used for |
|---|---|---|---|
| `rosorin_description/` | Hiwonder's ROS 2 description package for the ROSOrin family: xacro, STL meshes, launch files | the copy in `C:\isaac-sim-standalone-6.2.0\rosorin_description-claude\rosorin_description` (from the robot). Upstream: [Hiwonder/ROSOrin-Pro](https://github.com/Hiwonder/ROSOrin-Pro), branch `orin-nx`, `ROS2/src_ros2/src/simulations/rosorin_description` (last change 2c2912e, 2026-04-15) | Meshes (`meshes/pro/**`), the Pro chassis mesh that the wheels and rollers are split from, and the wheel frames (`urdf/rosorin_old.xacro`) |
| `RosOrinProMesh/` | 4 files from the RosOrinProMesh repository | [MatinEsmaeili00/RosOrinProMesh](https://github.com/MatinEsmaeili00/RosOrinProMesh) (private), commit `ef71eab`, 2026-09-29 | `rosorinpro_resolved.urdf` is the build input (the xacro flattened for `MACHINE_TYPE=ROSOrin_Mecanum_Pro`). The Blender build script defines the wheel-split rule that `scripts/build_sim_urdf.py` repeats. The joint map and export report were used to check that the wheel split is identical to the Blender model |
| `Robot-ROSOrin-Pro/` | Feature list of the real robot with the ROS 2 nodes and topics seen on it (screenshots in `Features/assets/terminal/`) | [MatinEsmaeili00/Robot-ROSOrin-Pro](https://github.com/MatinEsmaeili00/Robot-ROSOrin-Pro), commit `42c5dbe`, 2026-11-02 (MIT, `LICENSE`) | Which features to simulate, and the real topic and node names (`/sclidar_scan_publisher` → COIN-D6 lidar, `/depth_cam/...`, `/ros_robot_controller/...`) |

## Checks done when copying (2026-11-03)

* `rosorin_description/` is byte-identical to the local copy the build used before (git blob hashes of all
  52 files). Compared with the upstream Hiwonder repository, 48 of 52 files are identical. The 4 that differ
  are the top-level xacros of the non-Pro models (`urdf/rosorin.xacro`, `mecanum.xacro`, `acker.xacro`,
  `differential.xacro`): the copy from the robot selects the lidar through `LIDAR_TYPE`. The build uses none
  of these 4 files.
* `RosOrinProMesh/rosorinpro_resolved.urdf` is byte-identical to the file the build used before.
* Rebuilding `description/` from these copies gives a byte-identical `rosorin_pro_sim.urdf` and identical
  meshes.

Not copied: the Blender and FBX files of RosOrinProMesh (`ROSOrinPro_Unreal.blend` / `.fbx`, 11 MB). They are
outputs for Unreal and are not used here; they stay in that repository.

## License

Hiwonder's package declares no license (`package.xml`: "TODO: License declaration"), and neither does the
upstream Hiwonder/ROSOrin-Pro repository. That is why this repository is private. Ask Hiwonder before
publishing the meshes. `Robot-ROSOrin-Pro/` is MIT (see its `LICENSE`).
