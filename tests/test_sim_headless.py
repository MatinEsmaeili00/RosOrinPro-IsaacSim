r"""Headless end-to-end test of the ROSOrin Pro simulation through its ROS 2 interface.

    launch\run_tests.bat            (sets the ROS 2 Humble/FastDDS environment and runs this file)

Drives the robot exactly like a ROS 2 user would (/cmd_vel, /joint_command), checks the motion against
the physics ground truth and /odom, listens to every sensor topic of the real robot, and writes
tests/results/report.json plus snapshots (camera, depth, lidar) used in the documentation.
"""

import argparse
import json
import math
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd"))
ap.add_argument("--out", default=os.path.join(REPO, "tests", "results"))
ap.add_argument("--quick", action="store_true", help="shorter motion phases")
args, _ = ap.parse_known_args()
os.makedirs(args.out, exist_ok=True)

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.kit.app

em = omni.kit.app.get_app().get_extension_manager()
em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
sys.path.insert(0, os.path.join(REPO, "launch"))
if "--no-fabric" not in sys.argv:
    import rosorin_runtime as _rt

    _rt.enable_fabric()  # same configuration as the launchers

import numpy as np
import omni.timeline
import omni.usd
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

RESULTS = {"checks": [], "topics": {}, "phases": {}}


def check(name, ok, detail=""):
    RESULTS["checks"].append({"check": name, "pass": bool(ok), "detail": detail})
    print("[%s] %s  %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


ctx = omni.usd.get_context()
ctx.open_stage(args.scene)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(10):
    app.update()
stage = ctx.get_stage()
check("scene opened", stage.GetPrimAtPath("/World/rosorin_pro").IsValid(), args.scene)
blocks = stage.GetPrimAtPath("/World/TestArena/Blocks")
if blocks:
    blocks.SetActive(False)  # the motion tests would drive into them

# ------------------------------------------------------------------------------------------------ ROS 2 side
if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("rosorin_sim_test")
pub_cmd = node.create_publisher(Twist, "cmd_vel", 10)
pub_joint = node.create_publisher(JointState, "joint_command", 10)

TOPICS = {
    "/clock": Clock, "/scan": LaserScan, "/scan_raw": LaserScan, "/odom": Odometry, "/odom_raw": Odometry,
    "/imu": Imu, "/imu_corrected": Imu, "/ros_robot_controller/imu_raw": Imu,
    "/joint_states": JointState, "/controller_manager/joint_states": JointState, "/tf": TFMessage,
    "/depth_cam/rgb/image_raw": Image, "/depth_cam/rgb/camera_info": CameraInfo,
    "/depth_cam/depth0/image_raw": Image, "/depth_cam/depth0/camera_info": CameraInfo,
    "/depth_cam/depth0/points": PointCloud2, "/depth_cam/iro/image_raw": Image,
}
LAST, COUNT, STAMPS = {}, {t: 0 for t in TOPICS}, {t: [] for t in TOPICS}
TF_FRAMES = set()
TF_LAST = {}  # child -> (parent, xyz, quat xyzw)


def make_cb(topic):
    def cb(msg):
        LAST[topic] = msg
        COUNT[topic] += 1
        STAMPS[topic].append(time.time())
        if topic == "/tf":
            for t in msg.transforms:
                TF_FRAMES.add((t.header.frame_id, t.child_frame_id))
                tr, q = t.transform.translation, t.transform.rotation
                TF_LAST[t.child_frame_id] = (t.header.frame_id, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
    return cb


subs = [node.create_subscription(mt, t, make_cb(t), qos_profile_sensor_data) for t, mt in TOPICS.items()]

timeline = omni.timeline.get_timeline_interface()


def sim_time():
    return timeline.get_current_time()


from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node)


def step(n=1):
    for _ in range(n):
        app.update()
        for _ in range(60):  # drain all callbacks that are ready (one per spin_once)
            executor.spin_once(timeout_sec=0.0)


YAW = {"acc": 0.0, "last": None}
ODOM_LOG = []


def track_yaw():
    _, y, _ = pose()
    if YAW["last"] is not None:
        YAW["acc"] += math.atan2(math.sin(y - YAW["last"]), math.cos(y - YAW["last"]))
    YAW["last"] = y


def run_for(seconds, cmd=None):
    t0 = sim_time()
    while sim_time() - t0 < seconds:
        if cmd is not None:
            pub_cmd.publish(cmd)
        step()
        track_yaw()
        o = LAST.get("/odom")
        if o is not None:
            ODOM_LOG.append((o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z))


def twist(vx=0.0, vy=0.0, wz=0.0):
    m = Twist()
    m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
    return m


timeline.play()
step(5)

from isaacsim.core.experimental.prims import Articulation

robot = Articulation("/World/rosorin_pro/Geometry/base_footprint")
step(2)


def pose():
    p, q = robot.get_world_poses()
    p, q = p.numpy()[0], q.numpy()[0]  # q = (w, x, y, z)
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[1] ** 2 + q[2] ** 2)))))
    return p, yaw, tilt


def dofs(names):
    pos = robot.get_dof_positions().numpy()[0]
    allnames = robot.dof_names
    return {n: float(pos[allnames.index(n)]) for n in names}


RESULTS["articulation"] = {"num_dofs": robot.num_dofs, "num_links": robot.num_links}
check("articulation loaded", robot.num_dofs == 51 and robot.num_links == 52,
      "dofs=%d links=%d" % (robot.num_dofs, robot.num_links))

# --- 1. settle -------------------------------------------------------------------------------------------
run_for(2.0, twist())
p, yaw, tilt = pose()
RESULTS["phases"]["settle"] = {"pos": p.tolist(), "yaw": yaw, "tilt_deg": tilt}
check("robot rests on its rollers (base_footprint z ~ 0, upright)", abs(p[2]) < 0.01 and tilt < 2.0,
      "z=%.4f m tilt=%.2f deg" % (p[2], tilt))

init = {"joint1": 0.0, "joint2": 0.0, "joint3": 1.5708, "joint4": 0.7854, "joint5": 0.0, "r_joint": 0.0}
got0 = dofs(list(init))
check("arm starts in the init pose (camera looks ahead/down)", max(abs(got0[k] - v) for k, v in init.items()) < 0.05,
      str({k: round(v, 3) for k, v in got0.items()}))

dur = 2.0 if args.quick else 3.0


def motion_phase(name, cmd, expect):
    run_for(0.5, cmd)              # accelerate
    p1, y1, _ = pose()
    a1 = YAW["acc"]
    t1 = sim_time()
    n_odom = len(ODOM_LOG)
    run_for(dur, cmd)
    p2, y2, tilt2 = pose()
    dt = sim_time() - t1
    dyaw = YAW["acc"] - a1
    # velocity in the body frame at the start of the measured window
    d = p2[:2] - p1[:2]
    c, s = math.cos(y1), math.sin(y1)
    vb = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]]) / dt
    wz = dyaw / dt
    ov = tuple(np.mean(ODOM_LOG[n_odom:], axis=0)) if len(ODOM_LOG) > n_odom else None
    run_for(1.0, twist())          # stop
    RESULTS["phases"][name] = {"cmd": [cmd.linear.x, cmd.linear.y, cmd.angular.z], "measured_body_vel": [*vb, wz],
                               "odom_twist": ov, "tilt_deg": tilt2}
    err = [vb[0] - expect[0], vb[1] - expect[1], wz - expect[2]]
    tol = [max(0.03, 0.15 * abs(e)) for e in expect]   # mecanum rollers slip a little (real ones too)
    ok = all(abs(e) <= t for e, t in zip(err, tol)) and tilt2 < 3.0
    check("drive: %s" % name, ok, "cmd=(%.2f, %.2f, %.2f) measured=(%.3f, %.3f, %.3f) odom=%s" % (
        cmd.linear.x, cmd.linear.y, cmd.angular.z, vb[0], vb[1], wz,
        tuple(round(v, 3) for v in ov) if ov else None))


motion_phase("forward 0.2 m/s", twist(vx=0.2), (0.2, 0.0, 0.0))
motion_phase("strafe left 0.2 m/s", twist(vy=0.2), (0.0, 0.2, 0.0))
motion_phase("rotate 1.0 rad/s", twist(wz=1.0), (0.0, 0.0, 1.0))
motion_phase("diagonal 0.15/0.15", twist(vx=0.15, vy=-0.15), (0.15, -0.15, 0.0))

# --- TF correctness: compose /tf down to end_effector_link and compare with URDF forward kinematics -----------
import xml.etree.ElementTree as ET


def _R(q):  # quaternion xyzw -> 3x3
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _T(R, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, t
    return m


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axis(a, q):
    a = np.asarray(a) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


URDF_J = {}
for j in ET.parse(os.path.join(REPO, "description", "urdf", "rosorin_pro_sim.urdf")).getroot().findall("joint"):
    o, a = j.find("origin"), j.find("axis")
    URDF_J[j.find("child").get("link")] = (j.get("name"), j.find("parent").get("link"),
                                           [float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy").split()],
                                           [float(v) for v in a.get("xyz").split()] if a is not None else None)


def urdf_fk(link, q):
    m = np.eye(4)
    while link in URDF_J:
        name, parent, xyz, rpy, ax = URDF_J[link]
        A = _T(_rpy(*rpy), xyz)
        if ax is not None:
            A = A @ _T(_axis(ax, q.get(name, 0.0)), [0, 0, 0])
        m = A @ m
        link = parent
    return m


def tf_chain(link):
    m = np.eye(4)
    chain = []
    while link in TF_LAST and link != "base_footprint":
        parent, t, q = TF_LAST[link]
        m = _T(_R(q), t) @ m
        chain.append(link)
        link = parent
    return m, list(reversed(chain)), link


def check_tf(label):
    js_ = LAST.get("/joint_states")
    q = dict(zip(js_.name, js_.position))
    worst = 0.0
    for link in ("end_effector_link", "depth_cam_frame", "lidar_frame", "l_out_link"):
        m_tf, chain, root = tf_chain(link)
        m_fk = urdf_fk(link, q)
        worst = max(worst, float(np.linalg.norm(m_tf[:3, 3] - m_fk[:3, 3])))
    m_tf, chain, root = tf_chain("end_effector_link")
    check("/tf values match URDF forward kinematics (%s)" % label, worst < 0.005 and root == "base_footprint",
          "max position error %.4f m; chain %s -> %s" % (worst, root, " -> ".join(chain)))


# --- 2. arm + gripper through /joint_command -----------------------------------------------------------------
target = {"joint1": 0.5, "joint2": -0.6, "joint3": 0.8, "joint4": 0.6, "joint5": -0.4, "r_joint": 0.5}
js = JointState()
js.name = list(target)
js.position = [float(v) for v in target.values()]
for _ in range(5):
    pub_joint.publish(js)
    step()
run_for(2.5)
got = dofs(list(target) + ["l_joint", "l_in_joint", "l_out_joint", "r_in_joint", "r_out_joint"])
err = max(abs(got[k] - v) for k, v in target.items())
RESULTS["phases"]["arm"] = {"target": target, "reached": got}
check("arm + gripper follow /joint_command", err < 0.05, "max error %.4f rad" % err)
mimic_expect = {"l_joint": 1, "l_in_joint": -1, "l_out_joint": 1, "r_in_joint": 1, "r_out_joint": -1}
merr = max(abs(got[k] - m * got["r_joint"]) for k, m in mimic_expect.items())
check("gripper mimic joints follow r_joint (4-bar linkage)", merr < 0.03, "max error %.4f rad" % merr)
check_tf("arm in the test pose")
# back to the init pose
js.position = [float(init[k]) for k in target]
for _ in range(5):
    pub_joint.publish(js)
    step()
run_for(2.0)

# --- 3. sensors / topics -------------------------------------------------------------------------------------
t0w = time.time()
COUNT0 = dict(COUNT)
t0 = sim_time()
run_for(3.0, twist())
sim_dt = sim_time() - t0
for t in TOPICS:
    n = COUNT[t] - COUNT0[t]
    RESULTS["topics"][t] = {"messages": COUNT[t], "rate_hz_sim": n / sim_dt}
expected_rate = {"/scan": 10, "/depth_cam/rgb/image_raw": 15, "/depth_cam/depth0/image_raw": 15, "/odom": 60, "/imu": 60}
for t in TOPICS:
    rate = RESULTS["topics"][t]["rate_hz_sim"]
    exp = expected_rate.get(t)
    ok = COUNT[t] > 0 and (exp is None or abs(rate - exp) <= 0.25 * exp)
    check("topic %s" % t, ok, "%d msgs, %.1f Hz sim time%s" % (COUNT[t], rate, (" (expected ~%d)" % exp) if exp else ""))

scan = LAST.get("/scan")
if scan:
    r = np.array(scan.ranges)
    valid = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    RESULTS["scan"] = {"frame_id": scan.header.frame_id, "n": len(r), "angle_min": scan.angle_min,
                       "angle_max": scan.angle_max, "angle_increment": scan.angle_increment,
                       "range_min": scan.range_min, "range_max": scan.range_max,
                       "valid": int(valid.sum()), "min": float(r[valid].min()) if valid.any() else None,
                       "max": float(r[valid].max()) if valid.any() else None}
    check("lidar: COIN-D6 400 samples/scan, 0.05-12 m, frame lidar_frame",
          len(r) in (399, 400, 401) and scan.header.frame_id == "lidar_frame" and valid.sum() > 200,
          json.dumps(RESULTS["scan"]))
img = LAST.get("/depth_cam/rgb/image_raw")
if img:
    check("rgb camera 640x400 rgb8, frame depth_cam_frame", (img.width, img.height) == (640, 400)
          and img.header.frame_id == "depth_cam_frame", "%dx%d %s %s" % (img.width, img.height, img.encoding, img.header.frame_id))
dep = LAST.get("/depth_cam/depth0/image_raw")
if dep:
    d = np.frombuffer(bytes(dep.data), dtype=np.float32).reshape(dep.height, dep.width)
    fin = np.isfinite(d) & (d > 0)
    RESULTS["depth"] = {"encoding": dep.encoding, "valid_fraction": float(fin.mean()),
                        "min": float(d[fin].min()) if fin.any() else None, "max": float(d[fin].max()) if fin.any() else None}
    check("depth image 32FC1, sees the floor, within 0.15-3 m", dep.encoding == "32FC1" and fin.mean() > 0.3 and d[fin].max() <= 3.05,
          json.dumps(RESULTS["depth"]))
info = LAST.get("/depth_cam/rgb/camera_info")
if info:
    fx = info.k[0]
    hfov = math.degrees(2 * math.atan(info.width / (2 * fx)))
    RESULTS["camera_info"] = {"k": list(info.k), "hfov_deg": hfov, "frame_id": info.header.frame_id}
    check("camera_info: HFOV ~71 deg (Aurora 930 Pro)", abs(hfov - 71.0) < 1.0, "hfov=%.2f fx=%.1f" % (hfov, fx))
imu = LAST.get("/imu")
if imu:
    a = imu.linear_acceleration
    g = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
    RESULTS["imu"] = {"frame_id": imu.header.frame_id, "acc": [a.x, a.y, a.z], "|g|": g}
    check("imu at rest measures gravity ~9.81 along +z", abs(g - 9.81) < 0.3 and a.z > 9.0 and imu.header.frame_id == "imu_link",
          "acc=(%.2f, %.2f, %.2f)" % (a.x, a.y, a.z))
jsm = LAST.get("/joint_states")
if jsm:
    names = set(jsm.name)
    need = {"joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"}
    check("/joint_states has the arm + gripper joints", need <= names, "%d joints" % len(names))
need_tf = {("base_footprint", "base_link"), ("odom", "base_footprint")}
child_frames = {c for _, c in TF_FRAMES}
RESULTS["tf_pairs"] = sorted("%s -> %s" % p for p in TF_FRAMES)
need_pairs = {("odom", "base_footprint"), ("base_footprint", "base_link"), ("base_link", "lidar_frame"),
              ("base_link", "imu_link"), ("base_link", "link1"), ("link4", "link5"), ("link4", "camera_link0"),
              ("camera_link", "depth_cam_frame"), ("link5", "end_effector_link"), ("l_in_link", "l_out_link")}
check("/tf has the URDF tree (odom -> base_footprint -> base_link -> ... like robot_state_publisher)",
      need_pairs <= TF_FRAMES, "%d pairs, missing %s" % (len(TF_FRAMES), sorted(need_pairs - TF_FRAMES)))

# --- 4. snapshots for the docs ---------------------------------------------------------------------------
try:
    from PIL import Image as PILImage

    if img:
        a = np.frombuffer(bytes(img.data), dtype=np.uint8).reshape(img.height, img.width, -1)[:, :, :3]
        PILImage.fromarray(a).save(os.path.join(args.out, "camera_rgb.png"))
    if dep:
        dd = np.nan_to_num(d, nan=0.0, posinf=0.0)
        lo, hi = (float(dd[dd > 0].min()), float(dd[dd > 0].max())) if (dd > 0).any() else (0.15, 3.0)
        v = np.clip((dd - lo) / max(hi - lo, 1e-3), 0, 1)   # near = blue, far = red
        col = (np.stack([v, 1 - np.abs(2 * v - 1), 1 - v], -1) * 255 * (dd[..., None] > 0)).astype(np.uint8)
        PILImage.fromarray(col).save(os.path.join(args.out, "camera_depth.png"))
    ir = LAST.get("/depth_cam/iro/image_raw")
    if ir:
        a = np.frombuffer(bytes(ir.data), dtype=np.uint8).reshape(ir.height, ir.width, -1)[:, :, :3]
        PILImage.fromarray(a).save(os.path.join(args.out, "camera_ir.png"))
    if scan:
        size, res = 600, 600 / 5.0  # 5 m wide view
        canvas = np.full((size, size, 3), 255, np.uint8)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = valid
        # lidar_frame -> base frame (SCLIDAR yaw = pi)
        yaw_l = math.pi
        xs = r[ok] * np.cos(ang[ok] + yaw_l)
        ys = r[ok] * np.sin(ang[ok] + yaw_l)
        u = (size / 2 - ys * res).astype(int)
        v2 = (size / 2 - xs * res).astype(int)
        m = (u >= 0) & (u < size) & (v2 >= 0) & (v2 < size)
        for du in (-1, 0, 1):
            for dv in (-1, 0, 1):
                canvas[np.clip(v2[m] + dv, 0, size - 1), np.clip(u[m] + du, 0, size - 1)] = (220, 30, 30)
        canvas[size // 2 - 4:size // 2 + 4, size // 2 - 4:size // 2 + 4] = (0, 0, 0)
        canvas[size // 2 - 25:size // 2 - 4, size // 2 - 1:size // 2 + 1] = (0, 0, 0)  # robot heading (up)
        PILImage.fromarray(canvas).save(os.path.join(args.out, "lidar_scan_top.png"))
except Exception as e:  # snapshots are optional
    print("snapshot error", e)

timeline.stop()
step(2)
fails = [c for c in RESULTS["checks"] if not c["pass"]]
RESULTS["summary"] = {"passed": len(RESULTS["checks"]) - len(fails), "failed": len(fails)}
with open(os.path.join(args.out, "report.json"), "w", encoding="utf-8") as f:
    json.dump(RESULTS, f, indent=2, default=str)
print("TEST DONE passed=%d failed=%d" % (RESULTS["summary"]["passed"], len(fails)), flush=True)
node.destroy_node()
app.close()
