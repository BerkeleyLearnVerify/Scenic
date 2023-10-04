import glob
import os
from pathlib import Path
import shutil

import pytest

mapFolder = Path("assets") / "maps"
maps = glob.glob(str(mapFolder / "**" / "*.xodr"))
