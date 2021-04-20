"""Assorted utility functions."""

import math

import wrapt

sqrt2 = math.sqrt(2)

def cached(oldMethod):
    """Decorator for making a method with no arguments cache its result"""
    storageName = f'_cached_{oldMethod.__name__}'
    @wrapt.decorator
    def wrapper(wrapped, instance, args, kwargs):
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
