from importlib import import_module


_BACKENDS = {
    "core_51": "scenic.simulators.isaac.backends.core_51:Core51Backend",
    "experimental_51": "scenic.simulators.isaac.backends.experimental_51:Experimental51Backend",
    "experimental_60": "scenic.simulators.isaac.backends.experimental_60:Experimental60Backend",
    "lab": "scenic.simulators.isaac.backends.lab:LabBackend",
}
_INSTANCES = {}
DEFAULT_BACKEND_NAME = "experimental_60"
_DEFAULT_BACKEND = DEFAULT_BACKEND_NAME


def set_default_backend(name):
    global _DEFAULT_BACKEND
    name = DEFAULT_BACKEND_NAME if name is None else str(name)
    get_backend(name)
    _DEFAULT_BACKEND = name


def get_backend(name=None):
    name = _DEFAULT_BACKEND if name is None else str(name)
    if name not in _BACKENDS:
        available = ", ".join(sorted(_BACKENDS))
        raise ValueError(f"unknown Isaac backend {name!r}; available backends: {available}")
    if name not in _INSTANCES:
        module_name, class_name = _BACKENDS[name].split(":")
        module = import_module(module_name)
        _INSTANCES[name] = getattr(module, class_name)()
    return _INSTANCES[name]

def get_backend_version():
    return _DEFAULT_BACKEND

def articulation_action(backend=None, **kwargs):
    return get_backend(backend).articulation_action(**kwargs)
