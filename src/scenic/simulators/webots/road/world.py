"""Stub to allow changing the Webots world without changing the model."""

import os.path

#: Path to the WBT file to load the Webots world from
worldPath = None

def setLocalWorld(module, relpath):
    """Select a WBT file relative to the given module.

    This function is intended to be used with ``__file__`` as the *module*.
    """
    global worldPath
    base = os.path.dirname(module)
    worldPath = os.path.join(base, relpath)
