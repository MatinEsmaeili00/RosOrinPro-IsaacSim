"""Step 2 - import the simulation URDF into a USD robot asset.

    C:\\isaac-sim-standalone-6.2.0\\python.bat scripts\\import_to_usd.py [--merge-fixed-joints/--no-merge-fixed-joints]

Output: usd/rosorin_pro/  (the Isaac Sim URDF importer writes an asset package with payload layers)
Post-processing (physics drives, colliders, sensors) is done by scripts/configure_robot.py.
"""

import argparse
import os
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--urdf", default=os.path.join(REPO, "description", "urdf", "rosorin_pro_sim.urdf"))
ap.add_argument("--out", default=os.path.join(REPO, "usd", "rosorin_pro"))
ap.add_argument("--merge-fixed-joints", action=argparse.BooleanOptionalAction, default=True)
ap.add_argument("--dump", default=None, help="write a text dump of the imported prim tree here")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app

em = omni.kit.app.get_app().get_extension_manager()
em.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
em.set_extension_enabled_immediate("isaacsim.robot.schema", True)

from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

def main():
    if os.path.isdir(args.out):
        shutil.rmtree(args.out)
    os.makedirs(args.out, exist_ok=True)

    cfg = URDFImporterConfig()
    cfg.urdf_path = args.urdf
    cfg.usd_path = args.out
    cfg.merge_fixed_joints = args.merge_fixed_joints
    cfg.fix_base = False
    cfg.collision_from_visuals = False
    cfg.collision_type = "Convex Hull"
    cfg.allow_self_collision = False
    cfg.robot_type = "Mobile Manipulators"
    cfg.ros_package_paths = [{"name": "rosorin_pro_sim_description", "path": os.path.join(REPO, "description")}]
    cfg.joint_drive_type = "force"
    cfg.joint_target_type = {
        r"joint[1-5]": "position",
        r"[lr]_.*joint": "position",
        r".*_wheel$": "velocity",
        r".*_roller_\d+_joint": "none",
    }
    out = URDFImporter(cfg).import_urdf()
    print("IMPORTED", out, flush=True)

    if args.dump:
        from pxr import Usd

        stage = Usd.Stage.Open(out)
        with open(args.dump, "w", encoding="utf-8") as f:
            f.write("root layer: %s\n" % stage.GetRootLayer().identifier)
            for layer in stage.GetLayerStack():
                f.write("layer: %s\n" % layer.identifier)
            f.write("defaultPrim: %s\n" % stage.GetDefaultPrim().GetPath())
            for p in stage.Traverse():
                f.write("%s  [%s]  %s\n" % (p.GetPath(), p.GetTypeName(), ",".join(p.GetAppliedSchemas())))
        print("DUMPED", args.dump, flush=True)



try:
    main()
except Exception:  # make failures visible to build_all.bat (Kit would otherwise exit with code 0)
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    os._exit(1)
app.close()
