"""Assorted utility functions."""

import bz2
import collections
from contextlib import contextmanager
import functools
import itertools
import math
import os
import signal
from subprocess import CalledProcessError
import sys
import typing
import warnings
import weakref

import numpy
import trimesh

sqrt2 = math.sqrt(2)

if sys.version_info >= (3, 12):
    from itertools import batched
else:

    def batched(iterable, n):
        if n < 1:
            raise ValueError("n must be at least one")
        it = iter(iterable)
        while batch := tuple(itertools.islice(it, n)):
            yield batch


def cached(oldMethod):
    """Decorator for making a method with no arguments cache its result"""
    storageName = f"_cached_{oldMethod.__name__}"

    @functools.wraps(oldMethod)
    def wrapper(self):
        try:
            # Use __getattribute__ for direct lookup in case self is a Distribution
            return self.__getattribute__(storageName)
        except AttributeError:
            value = oldMethod(self)
            setattr(self, storageName, value)
            return value

    def clearer(self):
        try:
            delattr(self, storageName)
        except AttributeError:
            pass

    wrapper._scenic_cache_clearer = clearer

    return wrapper


_methodCaches = weakref.WeakKeyDictionary()


def cached_method(oldMethod):
    """Decorator for making a method cache its result on a per-object basis.

    Like ``functools.lru_cache(maxsize=None)`` except using a separate cache
    for each object, with the cache automatically deallocated when the object
    is garbage collected.
    """
    name = oldMethod.__name__

    @functools.wraps(oldMethod)
    def wrapper(self, *args, **kwargs):
        caches = _methodCaches.get(self, collections.defaultdict(dict))
        cachedMethod = caches.get(name)
        if cachedMethod is None:
            cachedMethod = functools.lru_cache(maxsize=None)(oldMethod)
            caches[name] = cachedMethod
        return cachedMethod(self, *args, **kwargs)

    def clearer(self):
        caches = _methodCaches.get(self, collections.defaultdict(dict))
        cachedMethod = caches.get(name)
        if cachedMethod:
            cachedMethod.cache_clear()

    wrapper._scenic_cache_clearer = clearer

    return wrapper


def cached_property(oldMethod):
    return property(cached(oldMethod))


def argsToString(args, kwargs={}):
    args = ", ".join(repr(arg) for arg in args)
    kwargs = ", ".join(f"{name}={value!r}" for name, value in kwargs.items())
    parts = []
    if args:
        parts.append(args)
    if kwargs:
        parts.append(kwargs)
    return ", ".join(parts)


@contextmanager
def alarm(seconds, handler=None, noNesting=False):
    if seconds <= 0 or not hasattr(signal, "SIGALRM"):  # SIGALRM not supported on Windows
        yield
        return
    if handler is None:
        handler = signal.SIG_IGN
    signal.signal(signal.SIGALRM, handler)
    if noNesting:
        assert oldHandler is signal.SIG_DFL, "SIGALRM handler already installed"
    length = int(math.ceil(seconds))
    previous = signal.alarm(length)
    if noNesting:
        assert previous == 0, 'nested call to "alarm"'
    try:
        yield
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, signal.SIG_DFL)


def loadMesh(path, filetype, compressed, binary):
    working_path = path

    if binary:
        mode = "rb"
    else:
        mode = "r"

    # Check if file is compressed
    if compressed is None:
        root, ext = os.path.splitext(working_path)

        if ext == ".bz2":
            compressed = True
            working_path = root
        else:
            compressed = False

    # Check mesh filetype
    if filetype is None:
        root, ext = os.path.splitext(working_path)

        if ext == "":
            raise ValueError("Mesh filetype not provided, but could not be extracted")

        filetype = ext

    if compressed:
        open_function = bz2.open
    else:
        open_function = open

    with open_function(path, mode) as mesh_file:
        mesh = trimesh.load(mesh_file, file_type=filetype, force="mesh")

    return mesh


def unifyMesh(mesh, verbose=False):
    """Attempt to merge mesh bodies, aborting if something fails.

    Should only be used with meshes that are volumes. Returns the
    original mesh if something goes wrong.
    """
    assert mesh.is_volume

    # No need to unify a mesh with less than 2 bodies
    if mesh.body_count < 2:
        return mesh

    mesh_bodies = mesh.split()

    if not all(m.is_volume for m in mesh_bodies):
        if verbose:
            warnings.warn(
                "The mesh that you loaded was composed of multiple bodies,"
                " but Scenic was unable to unify it because some of those bodies"
                " are non-volumetric (e.g. hollow portions of a volume). This is probably"
                " not an issue, but note that if any of these bodies have"
                " intersecting faces, Scenic may give undefined resuls. To suppress"
                " this warning in the future, consider adding the 'unify=False' parameter"
                " to your fromFile call."
            )
        return mesh

    try:
        unified_mesh = trimesh.boolean.union(mesh_bodies, engine="scad")
    except CalledProcessError:
        # Something went wrong, return the original mesh
        if verbose:
            warnings.warn(
                "The mesh that you loaded was composed of multiple bodies,"
                " but Scenic was unable to unify it because OpenSCAD raised"
                " an error."
            )
        return mesh

    # Check that the output is still a valid mesh
    if unified_mesh.is_volume:
        if verbose:
            if unified_mesh.body_count == 1:
                warnings.warn(
                    "The mesh that you loaded was composed of multiple bodies,"
                    " but Scenic was able to unify it into one single body. To save on compile"
                    " time in the future, consider running unifyMesh on your mesh outside"
                    " of Scenic and using that output instead."
                )
            elif unified_mesh.body_count < mesh.body_count:
                warnings.warn(
                    "The mesh that you loaded was composed of multiple bodies,"
                    " but Scenic was able to unify it into fewer bodies. To save on compile"
                    " time in the future, consider running unifyMesh on your mesh outside"
                    " of Scenic and using that output instead. Note that if any of these"
                    " bodies have intersecting faces, Scenic may give undefined resuls."
                )

        return unified_mesh
    else:
        if verbose:
            warnings.warn(
                "The mesh that you loaded was composed of multiple bodies,"
                " and Scenic was unable to unify it into fewer bodies. To save on compile"
                " time in the future, consider adding the 'unify=False' parameter to your"
                " fromFile call. Note that if any of these bodies have intersecting faces,"
                " Scenic may give undefined resuls."
            )
        return mesh


def repairMesh(mesh, pitch=(1 / 2) ** 6, verbose=True):
    """Attempt to repair a mesh, returning a proper 2-manifold.

    Repair is attempted via several steps, each sacrificing more accuracy
    but with a higher chance of returning a proper volumetric mesh.

    Repair is first attempted with easy fixes, like merging vertices and fixing
    winding. These will usually not deteriorate the quality of the mesh.

    Repair is then attempted by voxelizing the mesh, filling it, and then running
    marching cubes on the mesh. This approach is somewhat accurate but always
    produces solid objects. (This is to be expected since non watertight hollow
    objects aren't well defined).

    Repair is finally attempted by using the convex hull, which is unlikely to
    be accurate but is guaranteed to result in a volume.

    NOTE: For planar meshes, this function will throw an error.

    Args:
        mesh: The input mesh to be repaired.
        pitch: The target pitch to be used when attempting to repair the mesh via
            voxelization. A lower pitch uses smaller voxels, and thus a closer
            approximation, but can require significant additional processsing time.
            The actual pitch used may be larger if needed to get a manifold mesh.
        verbose: Whether or not to print warnings describing attempts to repair the mesh.
    """
    # If mesh is already a volume, we're done.
    if mesh.is_volume:
        return mesh

    # If mesh is planar, we can't fix it.
    if numpy.any(mesh.extents == 0):
        raise ValueError("repairMesh is undefined for planar meshes.")

    # Pitch must be positive
    if pitch <= 0:
        raise ValueError("pitch parameter must be positive.")

    ## Trimesh Processing ##
    processed_mesh = mesh.process(validate=True, merge_tex=True, merge_norm=True).copy()

    if processed_mesh.is_volume:
        if verbose:
            warnings.warn("Mesh was repaired via Trimesh process function.")

        return processed_mesh

    if verbose:
        warnings.warn(
            "Mesh could not be repaired via Trimesh process function."
            " attempting voxelization + marching cubes."
        )

    ## Voxelization + Marching Cubes ##
    # Extract largest dimension and scale so that it is unit length
    dims = numpy.abs(processed_mesh.extents)
    position = processed_mesh.bounding_box.center_mass
    scale = numpy.max(dims)

    processed_mesh.vertices -= position

    scale_matrix = numpy.eye(4)
    scale_matrix[:3, :3] /= scale
    processed_mesh.apply_transform(scale_matrix)

    # Compute new mesh with largest possible pitch
    curr_pitch = pitch

    while curr_pitch < 1:
        new_mesh = processed_mesh.voxelized(pitch=curr_pitch).fill().marching_cubes

        if new_mesh.is_volume:
            if verbose:
                warnings.warn(
                    f"Mesh was repaired via voxelization + marching cubes"
                    f" with pitch {curr_pitch}. Check the output to ensure it"
                    f" looks reasonable."
                )

            # Center new mesh
            new_mesh.vertices -= new_mesh.bounding_box.center_mass

            # Rescale mesh to original size
            orig_scale = new_mesh.extents / dims
            scale_matrix = numpy.eye(4)
            scale_matrix[:3, :3] /= orig_scale
            new_mesh.apply_transform(scale_matrix)

            # # Return mesh center to original position
            new_mesh.vertices += position

            return new_mesh

        curr_pitch *= 2

    raise ValueError("Mesh could not be repaired.")


class DefaultIdentityDict:
    """Dictionary which is the identity map by default.

    The map works on all objects, even unhashable ones, but doesn't support all
    of the standard mapping operations.
    """

    def __init__(self):
        self.storage = {}

    def clear(self):
        self.storage.clear()

    def __getitem__(self, key):
        return self.storage.get(id(key), key)

    def __setitem__(self, key, value):
        self.storage[id(key)] = value

    def __contains__(self, key):
        return id(key) in self.storage

    def __repr__(self):
        pairs = (f"{hex(key)}: {value!r}" for key, value in self.storage.items())
        allPairs = ", ".join(pairs)
        return f"<DefaultIdentityDict {{{allPairs}}}>"


# Patched version of typing.get_type_hints fixing bpo-37838

if sys.version_info >= (3, 8, 1) or (
    sys.version_info < (3, 8) and sys.version_info >= (3, 7, 6)
):
    get_type_hints = typing.get_type_hints
else:

    def get_type_hints(obj, globalns=None, localns=None):
        if not isinstance(obj, (type, types.ModuleType)) and globalns is None:
            wrapped = obj
            while hasattr(wrapped, "__wrapped__"):
                wrapped = wrapped.__wrapped__
            globalns = getattr(wrapped, "__globals__", {})
        return typing.get_type_hints(obj, globalns, localns)
