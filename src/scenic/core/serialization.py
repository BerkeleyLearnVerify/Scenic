"""Utilities to help serialize Scenic objects."""

from scenic.core.vectors import Vector

def scenicToJSON(obj):
    """Utility function to help serialize Scenic objects to JSON.

    Suitable for passing as the ``default`` argument to `json.dump`.
    """
    if isinstance(obj, Vector):
        return list(obj)
    raise TypeError(f'Object of type {obj.__class__.__name__} is not JSON serializable')
