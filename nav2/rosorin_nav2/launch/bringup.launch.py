"""Nav2 for the ROSOrin Pro: localization (AMCL on a saved map) or SLAM, plus navigation and RViz.

    ros2 launch rosorin_nav2 bringup.launch.py world:=splat              # AMCL on the splat building's map
    ros2 launch rosorin_nav2 bringup.launch.py world:=arena              # AMCL on the test arena's map
    ros2 launch rosorin_nav2 bringup.launch.py slam:=True                # build a new map while driving
    ros2 launch rosorin_nav2 bringup.launch.py map:=C:/maps/my.yaml initial_pose:="1.0 2.0 90"
    ros2 launch rosorin_nav2 bringup.launch.py use_sim_time:=False      # on the real robot

It wraps nav2_bringup/bringup_launch.py with this package's parameters. The AMCL initial pose comes from
config/worlds.yaml (where the robot spawns in that scene) unless initial_pose:="x y yaw_deg" is given.
"""

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(text):
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def _setup(context):
    share = get_package_share_directory("rosorin_nav2")
    bringup = get_package_share_directory("nav2_bringup")
    world = LaunchConfiguration("world").perform(context)
    slam = _truthy(LaunchConfiguration("slam").perform(context))
    map_arg = LaunchConfiguration("map").perform(context)
    pose_arg = LaunchConfiguration("initial_pose").perform(context).strip()
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    with open(os.path.join(share, "config", "worlds.yaml"), encoding="utf-8") as f:
        worlds = yaml.safe_load(f)
    if world not in worlds:
        raise RuntimeError("unknown world %r, choose one of %s" % (world, sorted(worlds)))
    w = worlds[world]
    map_yaml = map_arg or os.path.join(share, "maps", w["map"])
    pose = [float(v) for v in pose_arg.replace(",", " ").split()] if pose_arg else [float(v) for v in w["spawn"]]
    if len(pose) != 3:
        raise RuntimeError("initial_pose must be 'x y yaw_deg', got %r" % pose_arg)

    with open(params_file, encoding="utf-8") as f:
        params = yaml.safe_load(f)
    amcl = params["amcl"]["ros__parameters"]
    amcl["set_initial_pose"] = not slam
    amcl["initial_pose"] = {"x": pose[0], "y": pose[1], "z": 0.0, "yaw": math.radians(pose[2])}
    tmp = os.path.join(tempfile.gettempdir(), "rosorin_nav2_params_%d.yaml" % os.getpid())
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(params, f, sort_keys=False)

    actions = [
        LogInfo(msg="[rosorin_nav2] world=%s slam=%s map=%s initial_pose=%s use_sim_time=%s"
                % (world, slam, "(SLAM)" if slam else map_yaml, pose, use_sim_time)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup_launch.py")),
            launch_arguments={
                "slam": str(slam),
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": tmp,
                "autostart": LaunchConfiguration("autostart").perform(context),
                "use_composition": LaunchConfiguration("use_composition").perform(context),
                "log_level": LaunchConfiguration("log_level").perform(context),
            }.items(),
        ),
    ]
    if _truthy(LaunchConfiguration("rviz").perform(context)):
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="log",
            arguments=["-d", os.path.join(share, "rviz", "rosorin_nav2.rviz")],
            parameters=[{"use_sim_time": _truthy(use_sim_time)}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("rosorin_nav2")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="splat", description="scene: splat or arena (config/worlds.yaml)"),
        DeclareLaunchArgument("slam", default_value="False", description="True: map with slam_toolbox instead of AMCL"),
        DeclareLaunchArgument("map", default_value="", description="map yaml (default: the world's map)"),
        DeclareLaunchArgument("initial_pose", default_value="", description="'x y yaw_deg' (default: the world's spawn)"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("autostart", default_value="True"),
        DeclareLaunchArgument("use_composition", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_setup),
    ])
