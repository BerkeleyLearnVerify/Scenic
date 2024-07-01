metadrive = None
try:
    import metadrive
except ImportError:
    pass
if metadrive:
    from .simulator import MetaDriveSimulator
del metadrive