"""Assorted utility functions."""

import collections
from contextlib import contextmanager
import functools
import math
import signal
import sys
import typing
import os
import bz2

import trimesh

sqrt2 = math.sqrt(2)


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
    if seconds <= 0:
        yield
        return
    if handler is None:
        handler = signal.SIG_IGN
    try:
        signal.signal(signal.SIGALRM, handler)
        if noNesting:
            assert oldHandler is signal.SIG_DFL, "SIGALRM handler already installed"
    except ValueError:
        yield  # SIGALRM not supported on Windows
        return
    previous = signal.alarm(seconds)
    if noNesting:
        assert previous == 0, 'nested call to "alarm"'
    try:
        yield
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, signal.SIG_DFL)


def loadMesh(path, filetype=None, compressed=None):
    working_path = path

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

    with open_function(path, "r") as mesh_file:
        mesh = trimesh.load(mesh_file, file_type=filetype)

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


# Generic type introspection functions backported to Python 3.7
# (code taken from their Python 3.8 implementations)


def get_type_origin(tp):
    """Version of `typing.get_origin` supporting Python 3.7."""
    assert sys.version_info >= (3, 7)
    if sys.version_info >= (3, 8):
        return typing.get_origin(tp)
    if isinstance(tp, typing._GenericAlias):
        return tp.__origin__
    if tp is typing.Generic:
        return typing.Generic
    return None


def get_type_args(tp):
    """Version of `typing.get_args` supporting Python 3.7."""
    assert sys.version_info >= (3, 7)
    if sys.version_info >= (3, 8):
        return typing.get_args(tp)
    if isinstance(tp, typing._GenericAlias) and not tp._special:
        res = tp.__args__
        if get_type_origin(tp) is collections.abc.Callable and res[0] is not Ellipsis:
            res = (list(res[:-1]), res[-1])
        return res
    return ()
