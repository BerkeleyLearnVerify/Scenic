"""Build scenic.simulators.carla._blueprintData from JSON snapshots.

- Reads tools/carla/snapshots/blueprints_*.json.gz
- Sorts versions semantically (newest → oldest)
- Normalizes IDs/dims (sorted for stable diffs)
- Writes src/scenic/simulators/carla/_blueprintData.py with _IDS/_DIMS.

Usage: python tools/carla/make_blueprints.py
Output: src/scenic/simulators/carla/_blueprintData.py
"""

import gzip
import json
from pathlib import Path
import sys

CARLA_TOOLS_DIR = Path(__file__).resolve().parent  # .../Scenic/tools/carla
SNAPSHOT_DIR = CARLA_TOOLS_DIR / "snapshots"  # .../Scenic/tools/carla/snapshots
PROJECT_ROOT = CARLA_TOOLS_DIR.parents[1]  # .../Scenic
SNAPSHOT_PATTERN = "blueprints_*.json.gz"
OUT_PATH = PROJECT_ROOT / "src" / "scenic" / "simulators" / "carla" / "_blueprintData.py"


# Template for src/scenic/simulators/carla/_blueprintData.py
TEMPLATE = """\
# AUTO-GENERATED. Do not edit by hand.
# Built from tools/carla/make_blueprints.py

_IDS = {IDS_JSON}

_DIMS = {DIMS_JSON}
"""


def _verkey(s: str):
    """Turn a version string like '0.9.15' into a sortable (major, minor, patch) tuple."""
    parts = [int(p) for p in s.split(".")]
    parts += [0] * (3 - len(parts))
    return tuple(parts[:3])


def load_versions():
    files = list(SNAPSHOT_DIR.glob(SNAPSHOT_PATTERN))
    if not files:
        sys.exit("No input JSONs found.")

    entries = []
    for jf in files:
        with gzip.open(jf, "rt", encoding="utf-8") as f:
            obj = json.load(f)
        ver = str(obj["server_version"])
        ids = obj["ids"]
        dims = obj["dims"]
        entries.append((ver, {"ids": ids, "dims": dims}))

    entries.sort(key=lambda x: _verkey(x[0]), reverse=True)  # newest → oldest
    return {ver: data for ver, data in entries}


def normalize(versions):
    """Return (ids_map, dims_map) with categories, blueprint IDs, and dims sorted."""
    ids_map, dims_map = {}, {}
    for ver, data in versions.items():
        ids = data["ids"]
        dims = data["dims"]
        ids_map[ver] = {cat: sorted(bp_ids) for cat, bp_ids in sorted(ids.items())}
        dims_map[ver] = dict(sorted(dims.items()))
    return ids_map, dims_map


def build_source(versions):
    ids_map, dims_map = normalize(versions)
    ids_json = json.dumps(ids_map, indent=2)
    dims_json = json.dumps(dims_map, indent=2)
    return TEMPLATE.format(IDS_JSON=ids_json, DIMS_JSON=dims_json)


def main():
    src = build_source(load_versions())
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(src, encoding="utf-8")
    print(f"[OK] Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
