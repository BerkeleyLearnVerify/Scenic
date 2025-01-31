metadrive = None
try:
    import metadrive
except ImportError:
    raise ImportError("Metadrive is required. Please install the 'metadrive' package.")
if metadrive:
    from .simulator import MetaDriveSimulator
del metadrive
