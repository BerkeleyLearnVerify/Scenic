"""Build scenic.simulators.carla.blueprints from JSON snapshots.

- Reads tools/carla/snapshots/blueprints_*.json
- Sorts versions semantically (newest → oldest)
- Normalizes IDs/dims (sorted for stable diffs)
- Writes src/scenic/simulators/carla/blueprints.py with _IDS/_DIMS and helpers.

Usage: python tools/carla/make_blueprints.py
Output: src/scenic/simulators/carla/blueprints.py
"""

import json
from pathlib import Path

CARLA_TOOLS_DIR = Path(__file__).resolve().parent  # .../Scenic/tools/carla
SNAPSHOT_DIR = CARLA_TOOLS_DIR / "snapshots"  # .../Scenic/tools/carla/snapshots
PROJECT_ROOT = CARLA_TOOLS_DIR.parents[1]  # .../Scenic
SNAPSHOT_PATTERN = "blueprints_*.json"
OUT_PATH = PROJECT_ROOT / "src" / "scenic" / "simulators" / "carla" / "blueprints.py"


HEADER = '''\
# AUTO-GENERATED. Do not edit by hand.
# Built from tools/carla/make_blueprints.py

"""CARLA blueprints for cars, pedestrians, etc."""

from importlib.metadata import PackageNotFoundError, version as _pkg_version
import warnings

from scenic.core.distributions import distributionFunction
from scenic.core.errors import InvalidScenarioError

try:
    _CARLA_VER = _pkg_version("carla")
except PackageNotFoundError:
    _CARLA_VER = "0.0.0"  # no carla package; default to newest known blueprints

    
def _verkey(s: str):
    # Handle '0.9.15', '0.10.0', '1.2' -> (major, minor, patch)
    parts = [int(p) for p in s.split(".")]
    parts += [0] * (3 - len(parts))
    return tuple(parts[:3])

    
def _pick(vermap, ver):
    """Choose the blueprint map for CARLA version"""
    # 1) Exact match
    if ver in vermap:
        return vermap[ver]
    # 2) If same major.minor, use the newest patch.
    mm = ".".join(ver.split(".")[:2])
    cands = [v for v in vermap if v.startswith(mm + ".")]
    if cands:
        best = max(cands, key=_verkey)
        return vermap[best]
    # 3) Otherwise, use the newest version.
    best = max(vermap.keys(), key=_verkey)
    if ver != "0.0.0":
        warnings.warn(
            f"Unknown CARLA version {ver}; using blueprints for {best}."
            "Scenic may not have the correct set of blueprints."
        )
    return vermap[best]

    
oldBlueprintNames = {
    # Map current names to legacy names
    "vehicle.dodge.charger_police": ("vehicle.dodge_charger.police",),
    "vehicle.lincoln.mkz_2017": ("vehicle.lincoln.mkz2017",),
    "vehicle.mercedes.coupe": ("vehicle.mercedes-benz.coupe",),
    "vehicle.mini.cooper_s": ("vehicle.mini.cooperst",),
    "vehicle.ford.mustang": ("vehicle.mustang.mustang",),
}
'''


FOOTER = '''\
ids = _pick(_IDS, _CARLA_VER)
dims = _pick(_DIMS, _CARLA_VER)


def any_in(category):
    """Return all blueprint IDs for a category; raise if none recorded."""
    model = category + "Models"
    models = ids.get(model)
    if models:
        return models
    raise InvalidScenarioError(
        f"Scenic has no '{category}' blueprints recorded for CARLA {_CARLA_VER}."
    )


def _get_dim(bp_id, key, default):
    """Return recorded dimension or ``default`` if missing/0.

    Note: CARLA 0.9.14 bbox returns 0 for some blueprints (see CARLA issue #5841).
    """
    val = dims.get(bp_id, {}).get(key)
    return val if val else default


@distributionFunction
def width(bp_id, default):
    """Get width for ``bp_id``; return ``default`` if unknown."""
    return _get_dim(bp_id, "width", default)


@distributionFunction
def length(bp_id, default):
    """Get length for ``bp_id``; return ``default`` if unknown."""
    return _get_dim(bp_id, "length", default)


@distributionFunction
def height(bp_id, default):
    """Get height for ``bp_id``; return ``default`` if unknown."""
    return _get_dim(bp_id, "height", default)
'''

# Template for src/scenic/simulators/carla/blueprints.py
TEMPLATE = "{HEADER}\n" "_IDS = {IDS_JSON}\n\n" "_DIMS = {DIMS_JSON}\n\n" "{FOOTER}"


def _verkey(s: str):
    parts = [int(p) for p in s.split(".")]
    parts += [0] * (3 - len(parts))
    return tuple(parts[:3])


def load_versions():
    files = list(SNAPSHOT_DIR.glob(SNAPSHOT_PATTERN))
    if not files:
        raise SystemExit("No input JSONs found.")

    entries = []
    for jf in files:
        obj = json.loads(jf.read_text(encoding="utf-8"))
        ver = str(obj["server_version"])
        entries.append((ver, {"ids": obj.get("ids", {}), "dims": obj.get("dims", {})}))

    entries.sort(key=lambda x: _verkey(x[0]), reverse=True)  # newest → oldest
    return {ver: data for ver, data in entries}


def normalize(versions):
    """Return (ids_map, dims_map) with categories, blueprint IDs, and dims sorted."""
    ids_map, dims_map = {}, {}
    for ver, data in versions.items():
        ids = data.get("ids", {})
        dims = data.get("dims", {})
        ids_map[ver] = {cat: sorted(bp_ids) for cat, bp_ids in sorted(ids.items())}
        dims_map[ver] = dict(sorted(dims.items()))
    return ids_map, dims_map


def build_source(versions):
    ids_map, dims_map = normalize(versions)
    ids_json = json.dumps(ids_map, indent=2)
    dims_json = json.dumps(dims_map, indent=2)
    return TEMPLATE.format(
        HEADER=HEADER,
        IDS_JSON=ids_json,
        DIMS_JSON=dims_json,
        FOOTER=FOOTER,
    )


def main():
    src = build_source(load_versions())
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(src, encoding="utf-8")
    print(f"[OK] Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
