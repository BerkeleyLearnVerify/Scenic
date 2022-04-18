"""Assorted utility functions."""

import collections
from contextlib import contextmanager
import math
import signal
import sys
import typing

import decorator

sqrt2 = math.sqrt(2)

def cached(oldMethod):
    """Decorator for making a method with no arguments cache its result"""
    storageName = f'_cached_{oldMethod.__name__}'
    @decorator.decorator
    def wrapper(wrapped, *args, **kwargs):
        self = args[0]
        try:
            # Use __getattribute__ for direct lookup in case self is a Distribution
            return self.__getattribute__(storageName)
        except AttributeError:
            value = wrapped(self)
            setattr(self, storageName, value)
            return value
    return wrapper(oldMethod)

def cached_property(oldMethod):
    return property(cached(oldMethod))

def argsToString(args):
    names = (f'{a[0]}={a[1]}' if isinstance(a, tuple) else str(a) for a in args)
    joinedArgs = ', '.join(names)
    return f'({joinedArgs})'

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
            assert oldHandler is signal.SIG_DFL, 'SIGALRM handler already installed'
    except ValueError:
        yield      # SIGALRM not supported on Windows
        return
    previous = signal.alarm(seconds)
    if noNesting:
        assert previous == 0, 'nested call to "alarm"'
    try:
        yield
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, signal.SIG_DFL)

def areEquivalent(a, b):
    """Whether two objects are equivalent, i.e. have the same properties.

    This is only used for debugging, e.g. to check that a Distribution is the
    same before and after pickling. We don't want to define __eq__ for such
    objects since for example two values sampled with the same distribution are
    equivalent but not semantically identical: the code::

        X = (0, 1)
        Y = (0, 1)

    does not make X and Y always have equal values!
    """
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            return False
        for x, y in zip(a, b):
            if not areEquivalent(x, y):
                return False
        return True
    elif isinstance(a, (set, frozenset)) and isinstance(b, (set, frozenset)):
        if len(a) != len(b):
            return False
        mb = set(b)
        for x in a:
            found = False
            for y in mb:
                if areEquivalent(x, y):
                    mb.remove(y)
                    found = True
                    break
            if not found:
                return False
        return True
    elif isinstance(a, dict) and isinstance(b, dict):
        if len(a) != len(b):
            return False
        for x, v in a.items():
            found = False
            for y, w in b.items():
                if areEquivalent(x, y) and areEquivalent(v, w):
                    del b[y]
                    found = True
                    break
            if not found:
                return False
        return True
    elif hasattr(a, 'isEquivalentTo'):
        return a.isEquivalentTo(b)
    elif hasattr(b, 'isEquivalentTo'):
        return b.isEquivalentTo(a)
    else:
        return a == b

class DefaultIdentityDict:
    """Dictionary which is the identity map by default.

    The map works on all objects, even unhashable ones, but doesn't support all
    of the standard mapping operations.
    """
    def __init__(self):
        self.storage = {}

    def __getitem__(self, key):
        return self.storage.get(id(key), key)

    def __setitem__(self, key, value):
        self.storage[id(key)] = value

    def __contains__(self, key):
        return id(key) in self.storage

    def __repr__(self):
        pairs = (f'{hex(key)}: {value!r}' for key, value in self.storage.items())
        allPairs = ', '.join(pairs)
        return f'<DefaultIdentityDict {{{allPairs}}}>'

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
