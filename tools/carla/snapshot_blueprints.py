"""Collect CARLA blueprint snapshots.

- Connects to (or launches) CARLA, enumerates vehicle/walker/prop blueprints,
  and measures bounding-box dims.
- Writes snapshots/blueprints_<server_version>.json.

Requires: CARLA_ROOT, HOST/PORT reachable.
Usage: python tools/carla/make_snapshot.py
Output: tools/carla/snapshots/blueprints_<ver>.json
"""

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

SNAPSHOT_DIR = Path(__file__).resolve().parent / "snapshots"
HOST = "127.0.0.1"
PORT = 2000

SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)

# ---- server helpers ----------------------------------------------------------


def is_server_running(host=HOST, port=PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1)
        try:
            s.connect((host, port))
            return True
        except Exception:
            return False


def wait_until_ready(max_seconds=120):
    for _ in range(max_seconds):
        if is_server_running():
            return True
        time.sleep(1)
    return False


def check_carla_path():
    carla_root = os.environ.get("CARLA_ROOT")
    if not carla_root:
        print("ERROR: CARLA_ROOT is not set.", file=sys.stderr)
        sys.exit(2)
    return Path(carla_root)


def pick_launcher_from_root(root: Path):
    if (root / "CarlaUnreal.sh").exists():  # 0.10.x+
        return "CarlaUnreal.sh", "CarlaUnreal-Linux-Shipping"
    if (root / "CarlaUE4.sh").exists():  # 0.9.x
        return "CarlaUE4.sh", "CarlaUE4-Linux-Shipping"
    print(
        "ERROR: Could not find CarlaUnreal.sh or CarlaUE4.sh in CARLA_ROOT.",
        file=sys.stderr,
    )
    sys.exit(2)


def start_carla_if_needed():
    if is_server_running():
        return None
    root = check_carla_path()
    script, server_process_name = pick_launcher_from_root(root)
    print("[INFO] Starting CARLA")
    subprocess.Popen(
        f"bash {root}/{script} -RenderOffScreen",
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if not wait_until_ready():
        print("ERROR: Unable to connect to CARLA after starting.", file=sys.stderr)
        sys.exit(3)
    return server_process_name


# ---- categorization ----------------------------------------------------------

VEHICLE_BASETYPE_TO_CATEGORY = {
    "car": "carModels",
    "bicycle": "bicycleModels",
    "motorcycle": "motorcycleModels",
    "truck": "truckModels",
    "van": "vanModels",
    "bus": "busModels",
}

PROP_CATEGORY_RULES = [
    ("trashModels", ["trashcan", "bin"]),
    ("coneModels", ["cone"]),
    ("debrisModels", ["debris"]),
    ("vendingMachineModels", ["vendingmachine"]),
    ("chairModels", ["chair"]),
    ("busStopModels", ["busstop"]),
    ("advertisementModels", ["advertisement", "streetsign"]),
    ("garbageModels", ["colacan", "garbage", "plasticbag", "trashbag"]),
    ("containerModels", ["container"]),
    ("tableModels", ["table"]),
    ("barrierModels", ["barrier"]),
    ("plantpotModels", ["plantpot"]),
    ("mailboxModels", ["mailbox"]),
    ("gnomeModels", ["gnome"]),
    ("creasedboxModels", ["creasedbox"]),
    ("caseModels", ["case"]),
    ("boxModels", [".box"]),
    ("benchModels", ["bench"]),
    ("barrelModels", ["barrel"]),
    ("atmModels", ["atm"]),
    ("kioskModels", ["kiosk"]),
    ("ironplateModels", ["ironplank"]),
    ("trafficwarningModels", ["trafficwarning"]),
]

WALKER_CATEGORY = "walkerModels"

ALL_CATEGORY_KEYS = (
    list(VEHICLE_BASETYPE_TO_CATEGORY.values())
    + [WALKER_CATEGORY]
    + [category for category, _ in PROP_CATEGORY_RULES]
)

# ---- measuring ---------------------------------------------------------------


def get_dimensions_from_spawned(actor):
    bb = actor.bounding_box
    return {
        "width": 2.0 * abs(bb.extent.y),  # Y right
        "length": 2.0 * abs(bb.extent.x),  # X forward
        "height": 2.0 * abs(bb.extent.z),  # Z up
    }


def measure_dims(world, bp):
    import carla

    tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=500.0))
    actor = None
    try:
        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            print(
                f"[WARN] Could not spawn '{bp.id}' at x=0, y=0, z=500. No dimensions measured.",
                file=sys.stderr,
            )
            return None
        world.wait_for_tick()
        return get_dimensions_from_spawned(actor)
    except Exception as e:
        print(
            f"[WARN] Exception while measuring dims for '{bp.id}': {e}. No dimensions measured.",
            file=sys.stderr,
        )
        return None
    finally:
        if actor is not None:
            try:
                actor.destroy()
            except Exception:
                pass


def categorize_vehicle(bp):
    if not bp.has_attribute("base_type"):
        print(
            f"[WARN] vehicle blueprint '{bp.id}' has no 'base_type' attribute; skipping.",
            file=sys.stderr,
        )
        return None
    val = bp.get_attribute("base_type").as_str().lower()
    category = VEHICLE_BASETYPE_TO_CATEGORY.get(val)
    if not category:
        print(
            f"[WARN] vehicle blueprint '{bp.id}' has unsupported base_type='{val}'; skipping.",
            file=sys.stderr,
        )
    return category


def categorize_prop(bp):
    name = bp.id.lower()
    for category, keywords in PROP_CATEGORY_RULES:
        if any(kw in name for kw in keywords):
            return category
    return None


# ---- main --------------------------------------------------------------------


def main():
    try:
        import carla
    except Exception:
        print("ERROR: 'carla' Python package not installed.", file=sys.stderr)
        sys.exit(4)

    # start CARLA if needed; remember the process name so we can stop it later
    server_process_name = start_carla_if_needed()

    try:
        # connect
        client = carla.Client(HOST, PORT)
        client.set_timeout(10)
        world = client.get_world()
        server_version = client.get_server_version()
        print(f"[INFO] Connected to CARLA {server_version}")

        # gather blueprints
        lib = world.get_blueprint_library()
        vehicle_bps = lib.filter("vehicle.*")
        walker_bps = lib.filter("walker.pedestrian.*")
        prop_bps = lib.filter("static.prop.*")

        ids = {category: [] for category in ALL_CATEGORY_KEYS}
        dims = {}

        def add_group(bps, category_for_bp):
            for bp in bps:
                category = category_for_bp(bp)
                if not category:
                    continue
                ids[category].append(bp.id)
                md = measure_dims(world, bp)
                if md:
                    dims[bp.id] = md

        add_group(vehicle_bps, categorize_vehicle)
        add_group(walker_bps, lambda bp: WALKER_CATEGORY)
        add_group(prop_bps, categorize_prop)

        out_obj = {
            "server_version": server_version,
            "ids": ids,
            "dims": dims,
        }
        out_path = SNAPSHOT_DIR / f"blueprints_{server_version}.json"
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(out_obj, f, indent=2)
        print(f"[OK] Wrote {out_path}")

    finally:
        if server_process_name:
            subprocess.run(f"killall -9 {server_process_name}", shell=True)


if __name__ == "__main__":
    main()
