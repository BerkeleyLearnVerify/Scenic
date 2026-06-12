import os.path
from pathlib import Path


def getAssetPath(relpath):
    base = Path(__file__).parent.parent.parent.parent / "assets"
    return os.path.join(base, relpath)
