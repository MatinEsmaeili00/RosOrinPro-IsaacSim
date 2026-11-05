"""Send Nav2 goals to the ROSOrin Pro and report what happened.

    ros2 run rosorin_nav2 goal 1.5 0.5 90            # go to x=1.5 m, y=0.5 m, facing 90 deg (map frame)
    ros2 run rosorin_nav2 goal --world splat --place lab_door
    ros2 run rosorin_nav2 goal --world splat --tour  # every place of the world's tour, in order
    ros2 run rosorin_nav2 goal --world arena --tour --json result.json

For each goal it prints the Nav2 result, the time taken (simulation time when use_sim_time is on), the
distance driven (from /odom) and the final position error (from TF map -> base_footprint). In the simulator,
--ground-truth also reports the true position error and AMCL's localization error (the simulated /odom is
exact and starts at the world's spawn pose).
Exit code 0 when every goal succeeded.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED", GoalStatus.STATUS_UNKNOWN: "UNKNOWN"}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class GoalClient(Node):
    def __init__(self, use_sim_time):
        super().__init__("rosorin_goal", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.odom_dist = 0.0
        self._last_xy = None
        self.odom_pose = None  # (x, y, yaw) in the odom frame
        self.truth_spawn = None  # simulation only: world pose where /odom started (see --ground-truth)
        self.create_subscription(Odometry, "odom", self._odom, 20)

    def _odom(self, msg):
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self.odom_dist += math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)
        self.odom_pose = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))

    def true_pose(self):
        """World pose from the simulator's exact /odom and the spawn pose (None outside --ground-truth)."""
        if self.truth_spawn is None or self.odom_pose is None:
            return None
        sx, sy, syaw = self.truth_spawn
        ox, oy, oyaw = self.odom_pose
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * ox - s * oy, sy + s * ox + c * oy, wrap(syaw + oyaw)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:  # noqa: BLE001 - any TF error means "not localized yet"
            return None
        tr = t.transform.translation
        return tr.x, tr.y, quat_to_yaw(t.transform.rotation)

    def wait_ready(self, timeout):
        """Wait until the navigate_to_pose server exists and map -> base_footprint resolves."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self.client.server_is_ready() and self.robot_pose() is not None:
                return True
            self.spin_for(0.5)
        return False

    def go(self, x, y, yaw_deg, timeout):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(math.radians(yaw_deg))
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = qx, qy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw

        d0, t0, w0 = self.odom_dist, self.get_clock().now(), time.monotonic()
        send = self.client.send_goal_async(goal)
        while not send.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send.result()
        if not handle.accepted:
            return {"status": "REJECTED"}
        res = handle.get_result_async()
        last_print = 0.0
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - w0 > timeout:
                handle.cancel_goal_async()
                self.spin_for(1.0)
                status = "TIMEOUT"
                break
            if time.monotonic() - last_print > 5.0:
                p = self.robot_pose()
                if p:
                    print("   ... at (%.2f, %.2f), %.1f m to go" % (p[0], p[1], math.hypot(x - p[0], y - p[1])),
                          flush=True)
                last_print = time.monotonic()
        else:
            status = STATUS.get(res.result().status, str(res.result().status))
        self.spin_for(0.5)
        p = self.robot_pose()
        out = {
            "status": status,
            "sim_time_s": round((self.get_clock().now() - t0).nanoseconds * 1e-9, 2),
            "wall_time_s": round(time.monotonic() - w0, 2),
            "distance_m": round(self.odom_dist - d0, 3),
        }
        if p:
            out["final_pose"] = [round(p[0], 3), round(p[1], 3), round(math.degrees(p[2]), 1)]
            out["position_error_m"] = round(math.hypot(x - p[0], y - p[1]), 3)
            out["yaw_error_deg"] = round(math.degrees(abs(wrap(math.radians(yaw_deg) - p[2]))), 1)
        t = self.true_pose()
        if t:
            out["true_pose"] = [round(t[0], 3), round(t[1], 3), round(math.degrees(t[2]), 1)]
            out["true_position_error_m"] = round(math.hypot(x - t[0], y - t[1]), 3)
            if p:
                out["localization_error_m"] = round(math.hypot(p[0] - t[0], p[1] - t[1]), 3)
                out["localization_yaw_error_deg"] = round(math.degrees(abs(wrap(p[2] - t[2]))), 1)
        return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pose", nargs="*", type=float, help="x y [yaw_deg] in the map frame")
    ap.add_argument("--world", default="splat", help="world in config/worlds.yaml (for --place / --tour)")
    ap.add_argument("--place", action="append", default=[], help="named place of the world (repeatable)")
    ap.add_argument("--tour", action="store_true", help="visit the world's tour")
    ap.add_argument("--timeout", type=float, default=600.0, help="wall-clock seconds per goal")
    ap.add_argument("--ready-timeout", type=float, default=180.0, help="seconds to wait for Nav2 + localization")
    ap.add_argument("--use-sim-time", default="true", help="true (simulator) or false (real robot)")
    ap.add_argument("--json", default="", help="write the results to this file")
    ap.add_argument("--ground-truth", action="store_true",
                    help="simulation only: /odom is exact and starts at the world's spawn pose, so also report the "
                         "true position error and AMCL's localization error")
    ap.add_argument("--slam", action="store_true",
                    help="Nav2 runs with slam:=True: the map frame starts at the robot's start pose, so --place / "
                         "--tour positions (scene coordinates) are converted relative to the world's spawn pose")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    with open(os.path.join(get_package_share_directory("rosorin_nav2"), "config", "worlds.yaml"),
              encoding="utf-8") as f:
        worlds = yaml.safe_load(f)
    goals = []
    if len(args.pose) >= 2:
        goals.append(("goal", args.pose[0], args.pose[1], args.pose[2] if len(args.pose) > 2 else 0.0))
    names = list(args.place)
    if args.tour or names:
        if args.world not in worlds:
            ap.error("unknown world %r (have %s)" % (args.world, sorted(worlds)))
        places = worlds[args.world].get("places", {})
        if args.tour:
            names += worlds[args.world].get("tour", [])
        for n in names:
            if n not in places:
                ap.error("unknown place %r in world %r (have %s)" % (n, args.world, sorted(places)))
            px, py, pyaw = (float(v) for v in places[n])
            if args.slam:
                # SLAM's map frame starts at the robot's start pose, not at the scene's origin: express the
                # place (world coordinates) relative to the spawn pose
                sx, sy, syaw = (float(v) for v in worlds[args.world]["spawn"])
                c, s = math.cos(math.radians(syaw)), math.sin(math.radians(syaw))
                dx, dy = px - sx, py - sy
                px, py, pyaw = c * dx + s * dy, -s * dx + c * dy, pyaw - syaw
            goals.append((n, px, py, pyaw))
    if not goals:
        ap.error("give x y [yaw], --place NAME or --tour")

    rclpy.init(args=argv)
    node = GoalClient(args.use_sim_time.strip().lower() in ("1", "true", "yes"))
    if args.ground_truth:
        if args.world not in worlds:
            ap.error("--ground-truth needs --world (have %s)" % sorted(worlds))
        sx, sy, syaw = (float(v) for v in worlds[args.world]["spawn"])
        # with --slam the map frame is the start frame, i.e. /odom itself
        node.truth_spawn = (0.0, 0.0, 0.0) if args.slam else (sx, sy, math.radians(syaw))
    results = []
    try:
        print("[goal] waiting for Nav2 and localization (map -> base_footprint) ...", flush=True)
        if not node.wait_ready(args.ready_timeout):
            print("[goal] FAIL: Nav2 not ready after %.0f s (is the simulation playing and Nav2 launched?)"
                  % args.ready_timeout, flush=True)
            return 2
        p = node.robot_pose()
        print("[goal] robot at (%.2f, %.2f, %.0f deg)" % (p[0], p[1], math.degrees(p[2])), flush=True)
        for name, x, y, yaw in goals:
            print("[goal] -> %s (%.2f, %.2f, %.0f deg)" % (name, x, y, yaw), flush=True)
            r = node.go(x, y, yaw, args.timeout)
            r.update({"name": name, "target": [x, y, yaw]})
            results.append(r)
            print("[goal]    %s" % json.dumps(r), flush=True)
    finally:
        ok = bool(results) and all(r["status"] == "SUCCEEDED" for r in results)
        summary = {"ok": ok, "goals": results}
        print("[goal] %s: %d/%d goals succeeded" % ("PASS" if ok else "FAIL",
              sum(r["status"] == "SUCCEEDED" for r in results), len(goals)), flush=True)
        if args.json:
            with open(args.json, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
