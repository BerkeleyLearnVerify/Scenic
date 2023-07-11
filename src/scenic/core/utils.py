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
import weakref

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
        mesh = trimesh.load(mesh_file, file_type=filetype)

    return mesh


def unifyMesh(mesh):
    """Attempt to merge mesh bodies, aborting if something fails.

    Should only be used with meshes that are volumes.
    """
    assert mesh.is_volume

    # No need to unify a mesh with less than 2 bodies
    if mesh.body_count < 2:
        return mesh

    mesh_bodies = mesh.split()

    if not all(m.is_volume for m in mesh_bodies):
        return mesh

    try:
        unified_mesh = trimesh.boolean.union(mesh_bodies, engine="scad")
    except CalledProcessError:
        # Something went wrong, return the original mesh
        return mesh

    # Check that the output is still a valid mesh
    if unified_mesh.is_volume:
        return unified_mesh
    else:
        return mesh


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
