from importlib import import_module

_BACKENDS = {
    "core_51": "scenic.simulators.isaac.backends.core_51:Core51Backend",
    "experimental_51": "scenic.simulators.isaac.backends.experimental_51:Experimental51Backend",
    "experimental_60": "scenic.simulators.isaac.backends.experimental_60:Experimental60Backend",
    "lab": "scenic.simulators.isaac.backends.lab:LabBackend",
}
_INSTANCES = {}

# Backend auto-selected from the installed Isaac Sim major version. The default
# isaacBackend param is "auto" (resolved by detectBackend); an explicit
# isaacBackend param or --param flag overrides it. FALLBACK is used when the
# version can't be read (e.g. Isaac not pip-installed).
_VERSION_BACKENDS = {"5": "core_51", "6": "experimental_60"}
FALLBACK_BACKEND_NAME = "experimental_60"
DEFAULT_BACKEND_NAME = "auto"
_DEFAULT_BACKEND = FALLBACK_BACKEND_NAME


def detectBackend():
    """Pick a backend from the installed Isaac Sim version (pip metadata)."""
    try:
        from importlib.metadata import version

        major = version("isaacsim").split(".", 1)[0]
    except Exception:
        return FALLBACK_BACKEND_NAME
    return _VERSION_BACKENDS.get(major, FALLBACK_BACKEND_NAME)


def _resolve(name):
    if name is None:
        return _DEFAULT_BACKEND
    name = str(name)
    return detectBackend() if name == "auto" else name


def setDefaultBackend(name):
    global _DEFAULT_BACKEND
    name = detectBackend() if name in (None, "auto") else str(name)
    getBackend(name)
    _DEFAULT_BACKEND = name


def getBackend(name=None):
    name = _resolve(name)
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


def getBackendVersion():
    return _DEFAULT_BACKEND


def articulationAction(backend=None, **kwargs):
    return getBackend(backend).articulationAction(**kwargs)
