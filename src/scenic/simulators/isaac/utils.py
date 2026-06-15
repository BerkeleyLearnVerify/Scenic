import hashlib
import json
import os
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
import trimesh


def convert_sync(in_file, out_file, load_materials=False):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().convert_sync(in_file, out_file, load_materials=load_materials)


def get_simulation_app(headless=False):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().get_simulation_app(headless=headless)


def close_simulation_app(app):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().close_simulation_app(app)


def get_assets_root_path():
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().get_assets_root_path()


def asset_path(relative_path):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().asset_path(relative_path)


def resolvedPath(path):
    return Path(os.fspath(path)).expanduser().resolve()


def is_isaac_asset_reference(path):
    return os.fspath(path).startswith("Isaac/")


def has_url_scheme(path):
    return bool(urlparse(os.fspath(path)).scheme)


def _environment_cache_dir(source):
    source = os.fspath(source)
    digest = hashlib.sha1(source.encode("utf-8")).hexdigest()[:12]
    stem = Path(urlparse(source).path).stem or "environment"
    return (
        Path.home() / ".cache" / "scenic" / "isaac" / "environments" / f"{stem}_{digest}"
    )


def default_environment_mesh_paths(environment_usd_path):
    source = os.fspath(environment_usd_path)
    stem = Path(urlparse(source).path).stem

    if is_isaac_asset_reference(source) or has_url_scheme(source):
        output_dir = _environment_cache_dir(source)
    else:
        output_dir = resolvedPath(source).parent / "_converted"

    return (
        output_dir / f"{stem}_usd.gltf",
        output_dir / f"{stem}_info.json",
    )


def environment_outputs_current(environment_usd_path, mesh_path, info_path):
    mesh_path = Path(mesh_path)
    info_path = Path(info_path)
    if not mesh_path.is_file() or not info_path.is_file():
        return False

    source = os.fspath(environment_usd_path)
    if is_isaac_asset_reference(source) or has_url_scheme(source):
        return True

    usd_path = resolvedPath(source)
    if not usd_path.is_file():
        return False

    source_mtime = usd_path.stat().st_mtime
    return (
        mesh_path.stat().st_mtime >= source_mtime
        and info_path.stat().st_mtime >= source_mtime
    )


def ensure_environment_mesh_paths(
    environment_usd_path,
    environment_mesh_path=None,
    environment_info_path=None,
    *,
    headless=True,
    overwrite=False,
):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().ensure_environment_mesh_paths(
        environment_usd_path,
        environment_mesh_path,
        environment_info_path,
        headless=headless,
        overwrite=overwrite,
    )


class EnvironmentMeshCache:
    """Persistent cache for repaired Scenic meshes from a converted environment."""

    version = 2

    def __init__(self, environment_mesh_path, environment_info_path):
        self.environment_mesh_path = Path(environment_mesh_path)
        self.environment_info_path = Path(environment_info_path)
        self.cache_dir = self.environment_mesh_path.parent / (
            f"{self.environment_mesh_path.stem}_repaired"
        )
        self.manifest_path = self.cache_dir / "manifest.json"
        self.sources = self._source_signatures()
        self.manifest = self._load_manifest()
        self.changed = False

    def get(self, node_name, mesh):
        cache_file = self._cache_file(node_name)
        if self._manifest_current() and cache_file.is_file():
            try:
                cached = trimesh.load(cache_file, force="mesh", process=False)
                if cached.is_volume:
                    return cached
            except Exception:
                pass

        repaired = self._repair_mesh(mesh)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        repaired.export(cache_file)
        self.manifest["nodes"][node_name] = cache_file.name
        self.changed = True
        return repaired

    def save(self):
        if not self.changed:
            return
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.manifest.update(
            {
                "version": self.version,
                "sources": self.sources,
            }
        )
        with open(self.manifest_path, "w") as out_file:
            json.dump(self.manifest, out_file, indent=2)
        self.changed = False

    def _cache_file(self, node_name):
        digest = hashlib.sha1(node_name.encode("utf-8")).hexdigest()[:16]
        return self.cache_dir / f"{digest}.ply"

    def _load_manifest(self):
        default = {"version": self.version, "sources": self.sources, "nodes": {}}
        try:
            with open(self.manifest_path, "r") as in_file:
                manifest = json.load(in_file)
        except Exception:
            return default

        if not isinstance(manifest, dict) or "nodes" not in manifest:
            return default
        return manifest

    def _manifest_current(self):
        return (
            self.manifest.get("version") == self.version
            and self.manifest.get("sources") == self.sources
        )

    def _source_signatures(self):
        return {
            str(path): {"mtime_ns": path.stat().st_mtime_ns, "size": path.stat().st_size}
            for path in (self.environment_mesh_path, self.environment_info_path)
        }

    def _repair_mesh(self, mesh):
        from scenic.core.utils import repairMesh

        mesh = mesh.copy()
        if isPlanar(mesh):
            mesh = planeToMesh(mesh)
        mesh.apply_scale(0.01)

        if mesh.is_volume:
            print("Already watertight, skipping....")
            return mesh

        return repairMesh(mesh)


def vectorToArray(vector):
    return np.array((vector.x, vector.y, vector.z), dtype=float)


def colorToArray(color):
    return np.array(color, dtype=float) if color else None


def mesh_to_obj_frame(mesh):
    obj_mesh = mesh.copy()
    transform = trimesh.transformations.rotation_matrix(-np.pi / 2, (1, 0, 0))
    obj_mesh.apply_transform(transform)
    return obj_mesh


def planeToMesh(mesh):
    normal = mesh.face_normals[0]
    polygon = trimesh.path.polygons.projected(mesh, normal=normal)
    extruded = trimesh.creation.extrude_polygon(polygon, height=0.01)

    z_axis = np.array([0, 0, 1])
    rotation = trimesh.geometry.align_vectors(z_axis, normal)
    extruded.apply_transform(rotation)
    return extruded


def isPlanar(mesh, tolerance=1e-3):
    plane_origin, plane_normal = trimesh.points.plane_fit(mesh.vertices)
    distances = np.abs(np.dot(mesh.vertices - plane_origin, plane_normal))
    return np.all(distances <= tolerance)


def scenicToIsaacSimOrientation(orientation, initial_rotation=None):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().scenic_to_isaac_orientation(
        orientation, initial_rotation=initial_rotation
    )


def apply_visual_material(wrapper, obj):
    from scenic.simulators.isaac.backends import get_backend

    return get_backend().apply_visual_material(wrapper, obj)


_existingObj = {}


def _addExistingObj(obj):
    for key in (getattr(obj, "prim_path", None), getattr(obj, "name", None)):
        if key is not None:
            _existingObj[str(key)] = obj


def getExistingObj(objName):
    try:
        return _existingObj[objName]
    except KeyError as exc:
        available = ", ".join(sorted(_existingObj)) or "none"
        raise KeyError(
            f"no existing Isaac Sim object registered for {objName!r}; "
            f"available objects: {available}"
        ) from exc


def existingObjects():
    """Return each registered existing Isaac object once."""
    objs_by_prim_path = {}

    for obj in _existingObj.values():
        prim_path = getattr(obj, "prim_path", None)
        if prim_path is None:
            continue
        objs_by_prim_path[str(prim_path)] = obj

    return tuple(objs_by_prim_path.values())


def setCollidersExistingObj(verbose=False):
    from pxr import UsdPhysics

    from scenic.simulators.isaac.backends import get_backend

    approximation = UsdPhysics.Tokens.none

    changed = []
    failed = []

    for obj in existingObjects():
        prim_path = getattr(obj, "prim_path", None)
        if prim_path is None:
            continue

        prim_path = str(prim_path)

        try:
            get_backend().set_mesh_collision_approximation(prim_path, approximation)
            changed.append(prim_path)

            if verbose:
                print(
                    f"[setCollidersExistingObj] set {prim_path} "
                    f"collision approximation to {approximation}"
                )

        except Exception as exc:
            failed.append((prim_path, exc))

            if verbose:
                print(
                    f"[setCollidersExistingObj] failed for {prim_path}: "
                    f"{type(exc).__name__}: {exc}"
                )

    if failed:
        failed_text = "\n".join(
            f"  {prim_path}: {type(exc).__name__}: {exc}" for prim_path, exc in failed
        )
        raise RuntimeError(
            "failed to set mesh collision approximation for some existing "
            f"Isaac Sim objects:\n{failed_text}"
        )

    if verbose:
        print(
            f"[setCollidersExistingObj] updated {len(changed)} existing "
            "environment object(s)"
        )

    return changed
