try:
    import metadrive
except ImportError:
    raise ImportError(
        "Metadrive is required. Please install the 'metadrive' package (and sumolib) or use scenic[metadrive]."
    )

from .simulator import MetaDriveSimulator
