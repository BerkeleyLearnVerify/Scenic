"""Isaac Sim API backends.

Each backend wraps one flavor of the Isaac Sim Python API (the 5.1 Core API,
the 5.1/6.0 Core Experimental APIs, or Isaac Lab) behind the common
`IsaacBackend` interface. Backends are singletons: `getBackend` creates each
one on first use and returns the same instance afterwards, since a backend
owns the running ``SimulationApp``.
"""

from importlib import import_module

_BACKENDS = {
    "core_51": "scenic.simulators.isaac.backends.core_51:Core51Backend",
    "experimental_51": "scenic.simulators.isaac.backends.experimental_51:Experimental51Backend",
    "experimental_60": "scenic.simulators.isaac.backends.experimental_60:Experimental60Backend",
    "lab": "scenic.simulators.isaac.backends.lab:LabBackend",
}
_INSTANCES = {}

#: Backend chosen for each installed Isaac Sim major version when the
#: ``isaacBackend`` parameter is ``"auto"``.
_VERSION_BACKENDS = {"5": "core_51", "6": "experimental_60"}
#: Backend used when the Isaac Sim version cannot be read (e.g. it is not
#: pip-installed, or Scenic is running outside the Isaac environment).
FALLBACK_BACKEND_NAME = "experimental_60"
DEFAULT_BACKEND_NAME = "auto"


def detectBackend():
    """Pick a backend from the installed Isaac Sim version (pip metadata)."""
    try:
        from importlib.metadata import version

        major = version("isaacsim").split(".", 1)[0]
    except Exception:
        return FALLBACK_BACKEND_NAME
    return _VERSION_BACKENDS.get(major, FALLBACK_BACKEND_NAME)


def getBackend(name=DEFAULT_BACKEND_NAME):
    """Return the (singleton) backend with the given name.

    ``"auto"`` or `None` selects the backend matching the installed Isaac Sim
    version (see `detectBackend`).
    """
    name = DEFAULT_BACKEND_NAME if name is None else str(name)
    if name == DEFAULT_BACKEND_NAME:
        name = detectBackend()
    if name not in _BACKENDS:
        available = ", ".join(sorted(_BACKENDS))
        raise ValueError(
            f"unknown Isaac backend {name!r}; available backends: {available}"
        )
    if name not in _INSTANCES:
        module_name, class_name = _BACKENDS[name].split(":")
        module = import_module(module_name)
        _INSTANCES[name] = getattr(module, class_name)()
    return _INSTANCES[name]


def articulationAction(**kwargs):
    """Build a backend-independent articulation action.

    An action is a dict which may contain ``joint_positions``,
    ``joint_velocities`` and/or ``joint_efforts``, each paired with the DOF
    indices it applies to (``joint_position_indices`` etc., or a shared
    ``joint_indices``). Backends translate it to their native action type.
    """
    return dict(kwargs)
