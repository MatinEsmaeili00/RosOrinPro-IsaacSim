r"""End-to-end Nav2 test: headless simulation + Nav2 (AMCL or SLAM) + a tour of goals.

    launch\run_nav2_test.bat                       splat building, AMCL on the generated map, full tour
    launch\run_nav2_test.bat --world arena --slam  test arena, mapping with slam_toolbox while driving
    launch\run_nav2_test.bat --world splat --place conference_room

Steps: start launch\run_headless.bat on the world's scene, wait until the pixi ROS 2 sees /scan, start
`ros2 launch rosorin_nav2 bringup.launch.py`, run `ros2 run rosorin_nav2 goal --tour` and write
tests/results/report_nav2_<world>[_slam].json. Every process tree is stopped at the end, also on failure.
Needs only the Python standard library (run_nav2_test.bat uses Isaac Sim's bundled python.exe, which keeps
the simulator's and the pixi environment's ROS 2 libraries apart).
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISAAC = os.path.dirname(REPO)
NAV2 = os.path.join(REPO, "nav2")
RESULTS = os.path.join(REPO, "tests", "results")
SCENES = {"splat": os.path.join(REPO, "scenes", "splat_world.usd"),
          "arena": os.path.join(REPO, "scenes", "rosorin_world.usd")}


def find_pixi():
    exe = shutil.which("pixi") or os.path.join(os.environ.get("LOCALAPPDATA", ""), "pixi", "bin", "pixi.exe")
    if not os.path.isfile(exe):
        sys.exit("pixi not found: install it (winget install prefix-dev.pixi) and run `pixi install` in nav2\\")
    return exe


def start(cmd, log_path, cwd):
    log = open(log_path, "w", encoding="utf-8", errors="replace")
    flags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
    p = subprocess.Popen(cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, creationflags=flags,
                         env=dict(os.environ))
    p._log = log  # noqa: SLF001 - keep the handle with the process
    return p


def stop(p, name, grace=15.0):
    """Ctrl+Break the process group, then end the whole tree if it is still alive."""
    if p is None or p.poll() is not None:
        return
    try:
        if os.name == "nt":
            p.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            p.send_signal(signal.SIGINT)
        p.wait(timeout=grace)
    except Exception:  # noqa: BLE001
        pass
    if p.poll() is None and os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True)
    elif p.poll() is None:
        p.kill()
    p._log.close()  # noqa: SLF001
    print("[test_nav2] stopped", name, flush=True)


def wait_for_log(path, needles, proc, timeout):
    start_t = time.time()
    while time.time() - start_t < timeout:
        if proc.poll() is not None:
            return None
        try:
            with open(path, encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError:
            text = ""
        for n in needles:
            if n in text:
                return n
        time.sleep(2.0)
    return None


def topics(pixi):
    r = subprocess.run([pixi, "run", "--frozen", "ros2", "topic", "list", "--no-daemon"], cwd=NAV2,
                       capture_output=True, text=True, timeout=120)
    return set(r.stdout.split())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="splat", choices=sorted(SCENES))
    ap.add_argument("--slam", action="store_true", help="map with slam_toolbox instead of AMCL on the saved map")
    ap.add_argument("--place", action="append", default=[], help="named place(s) instead of the tour")
    ap.add_argument("--goal-timeout", type=float, default=900.0, help="wall seconds per goal")
    ap.add_argument("--cameras", action="store_true", help="keep the depth-camera graph on (slower)")
    ap.add_argument("--rviz", action="store_true", help="open RViz too")
    args = ap.parse_args()

    pixi = find_pixi()
    os.makedirs(RESULTS, exist_ok=True)
    tag = args.world + ("_slam" if args.slam else "")
    logs = {k: os.path.join(RESULTS, "nav2_%s_%s.log" % (tag, k)) for k in ("sim", "nav2", "goal")}
    report_path = os.path.join(RESULTS, "report_nav2_%s.json" % tag)
    goal_json = os.path.join(RESULTS, "nav2_%s_goals.json" % tag)
    scene = SCENES[args.world]
    if not os.path.exists(scene):
        sys.exit("scene missing: %s (splat: run launch\\build_splat_world.bat)" % scene)

    report = {"world": args.world, "slam": args.slam, "scene": os.path.relpath(scene, REPO).replace("\\", "/"),
              "date": time.strftime("%Y-%m-%d %H:%M"), "ok": False}
    sim = nav = None
    t0 = time.time()
    try:
        print("[test_nav2] building rosorin_nav2", flush=True)
        subprocess.run([pixi, "run", "--frozen", "build"], cwd=NAV2, check=True, capture_output=True)
        sim_cmd = ["cmd", "/c", os.path.join(REPO, "launch", "run_headless.bat"), "--scene", scene]
        if not args.cameras:
            sim_cmd.append("--no-cameras")
        print("[test_nav2] starting the simulation:", report["scene"], flush=True)
        sim = start(sim_cmd, logs["sim"], REPO)
        if not wait_for_log(logs["sim"], ["headless simulation running"], sim, 900):
            raise RuntimeError("simulation did not start (see %s)" % logs["sim"])
        for _ in range(30):
            if "/scan" in topics(pixi):
                break
            time.sleep(2.0)
        else:
            raise RuntimeError("/scan not visible from the pixi ROS 2 environment")
        report["sim_start_s"] = round(time.time() - t0, 1)

        nav_cmd = [pixi, "run", "--frozen", "ros2", "launch", "rosorin_nav2", "bringup.launch.py",
                   "world:=" + args.world, "slam:=" + str(args.slam), "rviz:=" + str(args.rviz)]
        print("[test_nav2] starting Nav2:", " ".join(nav_cmd[3:]), flush=True)
        nav = start(nav_cmd, logs["nav2"], NAV2)
        hit = wait_for_log(logs["nav2"], ["Managed nodes are active", "Aborting bringup"], nav, 300)
        if hit == "Aborting bringup":
            raise RuntimeError("Nav2 bringup aborted (see %s)" % logs["nav2"])
        if hit is None:
            print("[test_nav2] (no 'Managed nodes are active' line yet, the goal tool waits for Nav2 itself)",
                  flush=True)

        goal_cmd = [pixi, "run", "--frozen", "ros2", "run", "rosorin_nav2", "goal", "--world", args.world,
                    "--timeout", str(args.goal_timeout), "--json", goal_json, "--ground-truth"]
        goal_cmd += sum((["--place", p] for p in args.place), []) if args.place else ["--tour"]
        if args.slam:
            goal_cmd.append("--slam")  # SLAM's map frame starts at the spawn pose
        print("[test_nav2] sending goals ...", flush=True)
        with open(logs["goal"], "w", encoding="utf-8") as glog:
            g = subprocess.run(goal_cmd, cwd=NAV2, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            glog.write(g.stdout)
        for line in g.stdout.splitlines():
            if line.startswith("[goal]"):
                print(line, flush=True)
        if os.path.exists(goal_json):
            with open(goal_json, encoding="utf-8") as f:
                report.update(json.load(f))
        report["goal_exit_code"] = g.returncode
    except Exception as e:  # noqa: BLE001
        report["error"] = str(e)
        print("[test_nav2] ERROR:", e, flush=True)
    finally:
        stop(nav, "Nav2")
        stop(sim, "simulation", grace=30.0)
        report["wall_s"] = round(time.time() - t0, 1)
        goals = report.get("goals", [])
        report["summary"] = {
            "succeeded": sum(gl.get("status") == "SUCCEEDED" for gl in goals),
            "goals": len(goals),
            "distance_m": round(sum(gl.get("distance_m", 0.0) for gl in goals), 2),
            "sim_time_s": round(sum(gl.get("sim_time_s", 0.0) for gl in goals), 1),
            "max_true_position_error_m": max((gl.get("true_position_error_m", 0.0) for gl in goals), default=None),
            "max_localization_error_m": max((gl.get("localization_error_m", 0.0) for gl in goals), default=None),
        }
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        s = report["summary"]
        print("[test_nav2] %s: %d/%d goals, %.1f m driven, %.0f s sim time, %.0f s wall -> %s"
              % ("PASS" if report.get("ok") else "FAIL", s["succeeded"], s["goals"], s["distance_m"],
                 s["sim_time_s"], report["wall_s"], os.path.relpath(report_path, REPO)), flush=True)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
