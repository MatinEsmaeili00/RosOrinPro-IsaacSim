"""OmniGraph builders for the ROSOrin Pro (ROS 2 interface + mecanum drive), Isaac Sim 6.2.

All graphs are play-gated (OnPlaybackTick): nothing is published until the timeline plays.
Topic names are relative (no leading '/'), so setting a node namespace on the graphs namespaces
the whole robot. With an empty namespace they resolve to the same names as the real robot.

Node types and wiring follow the official Isaac Sim 6.2 graph API in
exts/isaacsim.ros2.nodes/isaacsim/ros2/nodes/impl/graph_api.py.
"""

import omni.graph.core as og
from pxr import Sdf

K = og.Controller.Keys

TICK = ("Tick", "omni.graph.action.OnPlaybackTick")
CTX = ("Context", "isaacsim.ros2.bridge.ROS2Context")
SIMTIME = ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime")
RUNONCE = ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame")


def _graph(path, nodes, values=(), connects=(), variables=()):
    edits = {K.CREATE_NODES: list(nodes)}
    if variables:
        edits[K.CREATE_VARIABLES] = list(variables)
    if values:
        edits[K.SET_VALUES] = list(values)
    if connects:
        edits[K.CONNECT] = list(connects)
    graph, _, _, _ = og.Controller.edit({"graph_path": path, "evaluator_name": "execution"}, edits)
    return graph


def mecanum_ik_matrix(wheels):
    """4x4 matrix M with  [w_lf, w_lb, w_rf, w_rb] = [vx, vy, wz, 0] . M   (OmniGraph MatrixMultiply is v . M).

    Exact mecanum inverse kinematics from the measured geometry of each wheel: contact point (x, y) relative
    to base_footprint, wheel radius r and the axis (ux, uy) of the roller touching the ground. The free roller
    only lets the contact point slide across its axis, so along the axis the wheel must match the body motion:
        w = ((vx - wz*y)*ux + (vy + wz*x)*uy) / (r*ux)
    """
    m = [[0.0] * 4 for _ in range(4)]
    for j, (x, y, ux, uy, r) in enumerate(wheels):
        m[0][j] = 1.0 / r
        m[1][j] = (uy / ux) / r
        m[2][j] = (-y + x * uy / ux) / r
    return m


def build_drive_graph(path, art_root, wheel_joints, ik_matrix, cmd_topics):
    """cmd_vel (several topics, last message wins) -> exact mecanum IK -> wheel velocity targets.

    Everything is stock OmniGraph: the IK is one 4x4 matrix product, so the robot drives from /cmd_vel with
    nothing but the USD file (no Python node, no extension).
    """
    nodes = [TICK, CTX,
             ("ReadCmd", "omni.graph.core.ReadVariable"),
             ("BreakCmd", "omni.graph.nodes.BreakVector3"),
             ("Zero", "omni.graph.nodes.ConstantDouble"),
             ("CmdVec4", "omni.graph.nodes.MakeVector4"),
             ("IkMatrix", "omni.graph.nodes.ConstantMatrix4d"),
             ("MecanumIK", "omni.graph.nodes.MatrixMultiply"),
             ("BreakWheels", "omni.graph.nodes.BreakVector4"),
             ("WheelArray", "omni.graph.nodes.ConstructArray"),
             ("WheelController", "isaacsim.core.nodes.IsaacArticulationController")]
    attrs = [("WheelArray.inputs:input1", "double"), ("WheelArray.inputs:input2", "double"),
             ("WheelArray.inputs:input3", "double")]
    values = [
        ("ReadCmd.inputs:variableName", "cmd"),
        ("Zero.inputs:value", 0.0),
        ("IkMatrix.inputs:value", ik_matrix),
        ("WheelArray.inputs:arraySize", 4),
        ("WheelArray.inputs:arrayType", "double[]"),
        ("WheelController.inputs:targetPrim", [Sdf.Path(art_root)]),
        ("WheelController.inputs:jointNames", list(wheel_joints)),
    ]
    connects = [
        ("Tick.outputs:tick", "WheelController.inputs:execIn"),
        ("ReadCmd.outputs:value", "BreakCmd.inputs:tuple"),
        ("BreakCmd.outputs:x", "CmdVec4.inputs:x"),
        ("BreakCmd.outputs:y", "CmdVec4.inputs:y"),
        ("BreakCmd.outputs:z", "CmdVec4.inputs:z"),
        ("Zero.inputs:value", "CmdVec4.inputs:w"),
        ("CmdVec4.outputs:tuple", "MecanumIK.inputs:a"),
        ("IkMatrix.inputs:value", "MecanumIK.inputs:b"),
        ("MecanumIK.outputs:output", "BreakWheels.inputs:tuple"),
        ("BreakWheels.outputs:x", "WheelArray.inputs:input0"),
        ("BreakWheels.outputs:y", "WheelArray.inputs:input1"),
        ("BreakWheels.outputs:z", "WheelArray.inputs:input2"),
        ("BreakWheels.outputs:w", "WheelArray.inputs:input3"),
        ("WheelArray.outputs:array", "WheelController.inputs:velocityCommand"),
    ]
    for i, topic in enumerate(cmd_topics):
        s, bl, ba, mk, wr = ("SubTwist%d" % i, "BreakLin%d" % i, "BreakAng%d" % i, "MakeCmd%d" % i, "WriteCmd%d" % i)
        nodes += [(s, "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                  (bl, "omni.graph.nodes.BreakVector3"),
                  (ba, "omni.graph.nodes.BreakVector3"),
                  (mk, "omni.graph.nodes.MakeVector3"),
                  (wr, "omni.graph.core.WriteVariable")]
        values += [(s + ".inputs:topicName", topic), (wr + ".inputs:variableName", "cmd")]
        connects += [
            ("Tick.outputs:tick", s + ".inputs:execIn"),
            ("Context.outputs:context", s + ".inputs:context"),
            (s + ".outputs:linearVelocity", bl + ".inputs:tuple"),
            (s + ".outputs:angularVelocity", ba + ".inputs:tuple"),
            (bl + ".outputs:x", mk + ".inputs:x"),
            (bl + ".outputs:y", mk + ".inputs:y"),
            (ba + ".outputs:z", mk + ".inputs:z"),
            (s + ".outputs:execOut", wr + ".inputs:execIn"),
            (mk + ".outputs:tuple", wr + ".inputs:value"),
        ]
    variables = [("cmd", og.Type(og.BaseDataType.DOUBLE, 3, 0))]
    edits = {K.CREATE_NODES: nodes, K.CREATE_VARIABLES: variables, K.CREATE_ATTRIBUTES: attrs,
             K.SET_VALUES: values, K.CONNECT: connects}
    graph, _, _, _ = og.Controller.edit({"graph_path": path, "evaluator_name": "execution"}, edits)
    return graph


def build_arm_graph(path, art_root, topic):
    """sensor_msgs/JointState commands (position/velocity/effort by joint name) -> articulation."""
    nodes = [TICK, CTX,
             ("SubJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
             ("ArmController", "isaacsim.core.nodes.IsaacArticulationController")]
    values = [("SubJointCommand.inputs:topicName", topic),
              ("ArmController.inputs:targetPrim", [Sdf.Path(art_root)])]
    connects = [
        ("Tick.outputs:tick", "SubJointCommand.inputs:execIn"),
        ("Context.outputs:context", "SubJointCommand.inputs:context"),
        ("SubJointCommand.outputs:execOut", "ArmController.inputs:execIn"),
        ("SubJointCommand.outputs:jointNames", "ArmController.inputs:jointNames"),
        ("SubJointCommand.outputs:positionCommand", "ArmController.inputs:positionCommand"),
        ("SubJointCommand.outputs:velocityCommand", "ArmController.inputs:velocityCommand"),
        ("SubJointCommand.outputs:effortCommand", "ArmController.inputs:effortCommand"),
    ]
    return _graph(path, nodes, values, connects)


def build_state_graph(path, art_root, chassis_prim, chassis_frame, tf_groups, imu_prim, imu_frame,
                      joint_state_topics, odom_topics, imu_topics):
    """/joint_states, /odom (+ odom -> base_footprint TF), /tf of the robot links and frames, /imu.

    tf_groups = [(parent_prim, [child_prims])]: one transform-tree node per parent, so /tf has the same
    parent/child structure as the URDF (robot_state_publisher on the real robot), not a flat tree.
    """
    nodes = [TICK, CTX, SIMTIME,
             ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
             ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
             ("TFOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
             ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU")]
    values = [
        ("SimTime.inputs:resetOnStop", True),
        ("ReadJointState.inputs:prim", [Sdf.Path(art_root)]),
        ("ComputeOdometry.inputs:chassisPrim", [Sdf.Path(chassis_prim)]),
        ("TFOdom.inputs:parentFrameId", "odom"),
        ("TFOdom.inputs:childFrameId", chassis_frame),
        ("TFOdom.inputs:topicName", "tf"),
        ("ReadIMU.inputs:imuPrim", [Sdf.Path(imu_prim)]),
        ("ReadIMU.inputs:readGravity", True),
    ]
    connects = [
        ("Tick.outputs:tick", "ReadJointState.inputs:execIn"),
        ("Tick.outputs:tick", "ComputeOdometry.inputs:execIn"),
        ("Tick.outputs:tick", "ReadIMU.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "TFOdom.inputs:execIn"),
        ("ComputeOdometry.outputs:position", "TFOdom.inputs:translation"),
        ("ComputeOdometry.outputs:orientation", "TFOdom.inputs:rotation"),
        ("Context.outputs:context", "TFOdom.inputs:context"),
        ("SimTime.outputs:simulationTime", "TFOdom.inputs:timeStamp"),
    ]
    for i, (parent, children) in enumerate(tf_groups):
        c, pb = "ComputeTF%d" % i, "PublishTF%d" % i
        nodes += [(c, "isaacsim.core.nodes.IsaacComputeTransformTree"),
                  (pb, "isaacsim.ros2.bridge.ROS2PublishTransformTree")]
        values += [(c + ".inputs:parentPrim", [Sdf.Path(parent)]),
                   (c + ".inputs:targetPrims", [Sdf.Path(p) for p in children]),
                   (pb + ".inputs:topicName", "tf")]
        connects += [("Tick.outputs:tick", c + ".inputs:execIn"),
                     (c + ".outputs:execOut", pb + ".inputs:execIn"),
                     (c + ".outputs:parentFrames", pb + ".inputs:parentFrames"),
                     (c + ".outputs:childFrames", pb + ".inputs:childFrames"),
                     (c + ".outputs:translations", pb + ".inputs:translations"),
                     (c + ".outputs:orientations", pb + ".inputs:orientations"),
                     ("Context.outputs:context", pb + ".inputs:context"),
                     ("SimTime.outputs:simulationTime", pb + ".inputs:timeStamp")]
    for i, topic in enumerate(joint_state_topics):
        n = "PublishJointState%d" % i
        nodes.append((n, "isaacsim.ros2.bridge.ROS2PublishJointState"))
        values.append((n + ".inputs:topicName", topic))
        for a in ("jointNames", "jointPositions", "jointVelocities", "jointEfforts", "jointDofTypes",
                  "stageMetersPerUnit", "sensorTime"):
            connects.append(("ReadJointState.outputs:" + a, n + ".inputs:" + a))
        connects += [("ReadJointState.outputs:execOut", n + ".inputs:execIn"),
                     ("Context.outputs:context", n + ".inputs:context"),
                     ("SimTime.outputs:simulationTime", n + ".inputs:timeStamp")]
    for i, topic in enumerate(odom_topics):
        n = "PublishOdom%d" % i
        nodes.append((n, "isaacsim.ros2.bridge.ROS2PublishOdometry"))
        values += [(n + ".inputs:topicName", topic), (n + ".inputs:odomFrameId", "odom"),
                   (n + ".inputs:chassisFrameId", chassis_frame)]
        for a in ("position", "orientation", "linearVelocity", "angularVelocity"):
            connects.append(("ComputeOdometry.outputs:" + a, n + ".inputs:" + a))
        connects += [("ComputeOdometry.outputs:execOut", n + ".inputs:execIn"),
                     ("Context.outputs:context", n + ".inputs:context"),
                     ("SimTime.outputs:simulationTime", n + ".inputs:timeStamp")]
    for i, (topic, with_orientation) in enumerate(imu_topics):
        n = "PublishImu%d" % i
        nodes.append((n, "isaacsim.ros2.bridge.ROS2PublishImu"))
        values += [(n + ".inputs:topicName", topic), (n + ".inputs:frameId", imu_frame),
                   (n + ".inputs:publishOrientation", bool(with_orientation))]
        connects += [("ReadIMU.outputs:execOut", n + ".inputs:execIn"),
                     ("ReadIMU.outputs:angVel", n + ".inputs:angularVelocity"),
                     ("ReadIMU.outputs:linAcc", n + ".inputs:linearAcceleration"),
                     ("ReadIMU.outputs:orientation", n + ".inputs:orientation"),
                     ("Context.outputs:context", n + ".inputs:context"),
                     ("SimTime.outputs:simulationTime", n + ".inputs:timeStamp")]
    return _graph(path, nodes, values, connects)


def build_lidar_graph(path, lidar_prim, frame_id, topics):
    """RTX lidar -> sensor_msgs/LaserScan on every topic in `topics`."""
    nodes = [TICK, CTX, RUNONCE, ("LidarRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
    values = [("LidarRenderProduct.inputs:cameraPrim", [Sdf.Path(lidar_prim)])]
    connects = [("Tick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "LidarRenderProduct.inputs:execIn")]
    for i, topic in enumerate(topics):
        n = "LaserScan%d" % i
        nodes.append((n, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
        values += [(n + ".inputs:topicName", topic), (n + ".inputs:type", "laser_scan"),
                   (n + ".inputs:frameId", frame_id)]
        connects += [("LidarRenderProduct.outputs:execOut", n + ".inputs:execIn"),
                     ("LidarRenderProduct.outputs:renderProductPath", n + ".inputs:renderProductPath"),
                     ("Context.outputs:context", n + ".inputs:context")]
    return _graph(path, nodes, values, connects)


def build_camera_graph(path, rgb_camera, depth_camera, width, height, frame_id, topics):
    """Aurora 930 Pro: RGB, depth (32FC1, metres), registered point cloud, IR stand-in, camera_info.

    The publish rate comes from omni:sensor:tickRate on the camera prims (Isaac Sim 6.2; frameSkipCount is
    deprecated)."""
    nodes = [TICK, CTX, RUNONCE,
             ("RgbRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
             ("DepthRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
    values = []
    connects = [("Tick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RgbRenderProduct.inputs:execIn"),
                ("RunOnce.outputs:step", "DepthRenderProduct.inputs:execIn")]
    for rp, cam in (("RgbRenderProduct", rgb_camera), ("DepthRenderProduct", depth_camera)):
        values += [(rp + ".inputs:cameraPrim", [Sdf.Path(cam)]),
                   (rp + ".inputs:width", int(width)), (rp + ".inputs:height", int(height))]
    helpers = [
        ("RgbInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper", "RgbRenderProduct", topics["rgb_info"], None),
        ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper", "RgbRenderProduct", topics["rgb"], "rgb"),
        ("DepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper", "DepthRenderProduct", topics["depth_info"], None),
        ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper", "DepthRenderProduct", topics["depth"], "depth"),
        ("Points", "isaacsim.ros2.bridge.ROS2CameraHelper", "DepthRenderProduct", topics["points"], "depth_pcl"),
        ("Ir", "isaacsim.ros2.bridge.ROS2CameraHelper", "DepthRenderProduct", topics["ir"], "rgb"),
    ]
    for name, ntype, rp, topic, htype in helpers:
        nodes.append((name, ntype))
        values += [(name + ".inputs:topicName", topic), (name + ".inputs:frameId", frame_id),
                   (name + ".inputs:resetSimulationTimeOnStop", True)]
        if htype:
            values.append((name + ".inputs:type", htype))
        connects += [(rp + ".outputs:execOut", name + ".inputs:execIn"),
                     (rp + ".outputs:renderProductPath", name + ".inputs:renderProductPath"),
                     ("Context.outputs:context", name + ".inputs:context")]
    return _graph(path, nodes, values, connects)


def build_clock_graph(path):
    nodes = [TICK, CTX, SIMTIME, ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock")]
    values = [("SimTime.inputs:resetOnStop", True), ("PublishClock.inputs:topicName", "clock")]
    connects = [("Tick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("SimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp")]
    return _graph(path, nodes, values, connects)
