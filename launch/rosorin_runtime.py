"""Python-side features of the ROSOrin Pro simulation (things the OmniGraph graphs in the USD cannot do).

  * Battery   /ros_robot_controller/battery  std_msgs/UInt16 [mV], like the robot's STM32 board.
              3S 6000 mAh pack: 12.6 V full -> 9.9 V empty, discharged by an idle load + the motor load,
              with voltage sag under load. Warns below 10.0 V like the real buzzer.
  * /robot_description  the simulation URDF (std_msgs/String, transient-local like robot_state_publisher), so
              RViz / MoveIt on the ROS side get the model (mesh URIs: package://rosorin_pro_sim_description).
  * Twin mode Digital-twin mirror of the REAL robot: subscribes to the robot's /joint_states and /odom on the
              robot's ROS domain (a separate DDS participant, so the sim's own topics never collide with the
              robot's) and makes the simulated arm, gripper and base follow it. The simulated sensors keep
              publishing on the sim's domain, so you can compare real and simulated lidar/camera/IMU.

Used by launch/open_scene.py (Isaac Sim GUI) and launch/run_headless.py. Everything runs inside the physics
step callback, i.e. only while the simulation is playing.
"""

import math
import os

import carb
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"]
WHEELS = ["left_front_wheel", "left_back_wheel", "right_front_wheel", "right_back_wheel"]


def enable_fabric():
    """Physics results to Fabric instead of USD, like apps/isaacsim.exp.full.fabric.kit (~25 % faster here)."""
    import omni.kit.app

    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.physx.fabric", True)
    s = carb.settings.get_settings()
    s.set("/persistent/omnigraph/updateToUsd", False)
    s.set("/persistent/physics/updateForceSensorsToUsd", False)


def _yaw(q_wxyz):
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _quat_yaw(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def mecanum_ik(build_report_path=None):
    """Rows: wheel speeds [rad/s] = M @ [vx, vy, wz] (same exact IK as the MecanumDrive graph)."""
    import json

    path = build_report_path or os.path.join(REPO, "description", "build_report.json")
    with open(path, "r", encoding="utf-8") as f:
        wheels = {w["joint"]: w for w in json.load(f)["wheels"]}
    rows = []
    for n in WHEELS:
        w = wheels[n]
        ring = np.mean([r["center"] for r in w["rollers"]], axis=0)
        bottom = min(w["rollers"], key=lambda r: r["center"][2])
        ux, uy = bottom["axis"][0], bottom["axis"][1]
        if ux < 0:
            ux, uy = -ux, -uy
        x, y, r = w["xyz"][0] + ring[0], w["xyz"][1] + ring[1], w["outer_radius"]
        rows.append([1.0 / r, (uy / ux) / r, (-y + x * uy / ux) / r])
    return np.array(rows)


class RosOrinRuntime:
    def __init__(self, robot_path="/World/rosorin_pro", config=None, battery=True, twin_domain=None,
                 twin_namespace="", log=print):
        cfg_path = config or os.path.join(REPO, "config", "rosorin_pro_sim.yaml")
        with open(cfg_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)
        self.robot_path = robot_path
        self.root_path = robot_path + "/Geometry/base_footprint"
        self.battery_enabled = battery
        self.twin_domain = twin_domain
        self.twin_ns = twin_namespace.strip("/")
        self.log = log
        self.ik = mecanum_ik()
        self.twin_gain = (2.0, 2.0)      # pose-error feedback [1/s] for position and yaw
        self.twin_max = (0.6, 3.0)       # m/s, rad/s
        self._subs = []
        self._art = None
        self._sim_ctx = self._twin_ctx = None
        self._reset_state()

    # ------------------------------------------------------------------------------------------- lifecycle
    def _reset_state(self):
        b = self.cfg["battery"]
        self.soc = float(b.get("initial_soc", 0.95))
        self._bat_t = 0.0
        self._bat_pub_t = 0.0
        self._low_warned = False
        self._twin_js = None
        self._twin_odom = None
        self._twin_ref = None
        self._art = None
        self._dof = None

    def start(self):
        import rclpy
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        self._rclpy = rclpy
        if self.battery_enabled:
            from std_msgs.msg import UInt16

            self._sim_ctx = rclpy.Context()
            rclpy.init(context=self._sim_ctx, domain_id=int(os.environ.get("ROS_DOMAIN_ID", "0")))
            self._sim_node = rclpy.create_node("rosorin_sim_runtime", context=self._sim_ctx)
            self._bat_pub = self._sim_node.create_publisher(UInt16, "ros_robot_controller/battery", 10)
            self._UInt16 = UInt16
            self._publish_robot_description()
        if self.twin_domain is not None:
            self._start_twin()
        self._subs.append(SimulationManager.register_callback(self._on_step, event=SimulationEvent.PHYSICS_POST_STEP))
        self._subs.append(SimulationManager.register_callback(lambda *_: self._reset_state(),
                                                              event=SimulationEvent.SIMULATION_STOPPED))
        self.log("[rosorin] runtime started: battery=%s twin_domain=%s" % (self.battery_enabled, self.twin_domain))

    def _publish_robot_description(self):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        urdf = os.path.join(REPO, "description", "urdf", "rosorin_pro_sim.urdf")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._desc_pub = self._sim_node.create_publisher(String, "robot_description", qos)
        with open(urdf, "r", encoding="utf-8") as f:
            self._desc_pub.publish(String(data=f.read()))

    def _start_twin(self):
        rclpy = self._rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self._twin_ctx = rclpy.Context()
        rclpy.init(context=self._twin_ctx, domain_id=int(self.twin_domain))
        self._twin_node = rclpy.create_node("rosorin_twin_listener", context=self._twin_ctx)
        ns = ("/" + self.twin_ns) if self.twin_ns else ""
        self._twin_node.create_subscription(JointState, ns + "/joint_states",
                                            lambda m: setattr(self, "_twin_js", m), qos_profile_sensor_data)
        self._twin_node.create_subscription(Odometry, ns + "/odom",
                                            lambda m: setattr(self, "_twin_odom", m), qos_profile_sensor_data)
        self._twin_exec = SingleThreadedExecutor(context=self._twin_ctx)
        self._twin_exec.add_node(self._twin_node)
        self.log("[rosorin] twin mode: following the real robot on ROS_DOMAIN_ID=%s (%s/joint_states, %s/odom)"
                 % (self.twin_domain, ns, ns))

    def stop(self):
        from isaacsim.core.simulation_manager import SimulationManager

        for s in self._subs:
            try:
                SimulationManager.deregister_callback(s)
            except Exception:
                pass
        self._subs = []
        for ctx in (self._sim_ctx, self._twin_ctx):
            if ctx is not None:
                try:
                    self._rclpy.shutdown(context=ctx)
                except Exception:
                    pass

    # --------------------------------------------------------------------------------------- per physics step
    def _articulation(self):
        if self._art is None:
            from isaacsim.core.experimental.prims import Articulation

            self._art = Articulation(self.root_path)
            names = self._art.dof_names
            self._dof = {n: names.index(n) for n in ARM_JOINTS + WHEELS if n in names}
        return self._art

    def _on_step(self, dt, _context):
        try:
            if self.battery_enabled:
                self._battery(dt)
            if self.twin_domain is not None:
                self._twin(dt)
        except Exception as e:  # never break the simulation loop
            carb.log_warn("[rosorin] runtime step error: %s" % e)

    # battery ------------------------------------------------------------------------------------------------
    def _battery(self, dt):
        b = self.cfg["battery"]
        self._bat_t += dt
        self._bat_pub_t += dt
        if self._bat_t < 0.1:
            return
        step, self._bat_t = self._bat_t, 0.0
        art = self._articulation()
        wv = art.get_dof_velocities().numpy()[0]
        wheel_speed = float(np.mean(np.abs([wv[self._dof[w]] for w in WHEELS])))   # rad/s
        speed = wheel_speed * 0.04                                                    # ~ m/s at the rim
        current = b["idle_current_a"] + b["drive_current_a_per_mps"] * speed
        scale = float(b.get("time_scale", 1.0))
        self.soc = max(0.0, self.soc - current * step * scale / 3600.0 / (b["capacity_mah"] / 1000.0))
        mv = b["empty_mv"] + (b["full_mv"] - b["empty_mv"]) * self.soc - 1000.0 * current * b.get("internal_resistance_ohm", 0.05)
        mv = int(max(0, min(65535, round(mv))))
        if self._bat_pub_t >= 1.0:
            self._bat_pub_t = 0.0
            self._bat_pub.publish(self._UInt16(data=mv))
            if mv < b["low_warning_mv"] and not self._low_warned:
                self._low_warned = True
                carb.log_warn("[rosorin] battery low: %.2f V (the real robot beeps below 10 V)" % (mv / 1000.0))

    # digital twin ---------------------------------------------------------------------------------------------
    def _twin(self, dt):
        for _ in range(20):
            self._twin_exec.spin_once(timeout_sec=0.0)
        art = self._articulation()
        js = self._twin_js
        if js is not None and js.position:
            idx, val = [], []
            for n, p in zip(js.name, js.position):
                if n in ARM_JOINTS and n in self._dof:
                    idx.append(self._dof[n])
                    val.append(p)
            if idx:
                art.set_dof_position_targets(np.array([val], np.float32), dof_indices=np.array(idx, np.int32))
        od = self._twin_odom
        if od is None:
            return
        p, q = od.pose.pose.position, od.pose.pose.orientation
        oyaw = _yaw((q.w, q.x, q.y, q.z))
        if self._twin_ref is None:  # anchor: robot odom origin <-> current sim pose
            sp, sq = art.get_world_poses()
            sp, sq = sp.numpy()[0], sq.numpy()[0]
            self._twin_ref = (np.array([p.x, p.y]), oyaw, sp.copy(), _yaw(sq))
        o0, oy0, s0, sy0 = self._twin_ref
        d = np.array([p.x, p.y]) - o0
        c, s = math.cos(sy0 - oy0), math.sin(sy0 - oy0)
        target_xy = s0[:2] + np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        target_yaw = sy0 + (oyaw - oy0)
        # The base is driven physically through its wheels (no teleport, which disturbs the arm drives):
        # feed-forward = the real robot's odometry twist, feedback = pose error, exact mecanum IK -> wheels.
        cur_p, cur_q = art.get_world_poses()
        cur_p, cur_yaw = cur_p.numpy()[0], _yaw(cur_q.numpy()[0])
        ex, ey = target_xy - cur_p[:2]
        eyaw = math.atan2(math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw))
        cb, sb = math.cos(cur_yaw), math.sin(cur_yaw)
        t = od.twist.twist
        kp, kyaw = self.twin_gain
        vx = t.linear.x + kp * (cb * ex + sb * ey)
        vy = t.linear.y + kp * (-sb * ex + cb * ey)
        wz = t.angular.z + kyaw * eyaw
        v = math.hypot(vx, vy)
        if v > self.twin_max[0]:
            vx, vy = vx * self.twin_max[0] / v, vy * self.twin_max[0] / v
        wz = max(-self.twin_max[1], min(self.twin_max[1], wz))
        wheels = self.ik @ np.array([vx, vy, wz])
        art.set_dof_velocity_targets(np.array([wheels], np.float32),
                                     dof_indices=np.array([self._dof[w] for w in WHEELS], np.int32))


def deactivate_command_graphs(stage, robot_path="/World/rosorin_pro"):
    """Twin mode: the robot is driven by the real one, so switch off the /cmd_vel and /joint_command graphs
    (session layer only, the scene file is not modified)."""
    from pxr import Usd

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for g in ("MecanumDrive", "ArmGripper"):
            p = stage.GetPrimAtPath("%s/Graphs/%s" % (robot_path, g))
            if p:
                p.SetActive(False)
