import glob
import os
from pathlib import Path
import shutil

import pytest

mapFolder = Path("assets") / "maps"
maps = glob.glob(str(mapFolder / "**" / "*.xodr"))

# TODO fix handling of this problematic map
badmap = str(mapFolder / "opendrive.org" / "sample1.1.xodr")
map_params = []
for path in maps:
    if path == badmap:
        param = pytest.param(
            badmap,
            marks=pytest.mark.xfail(
                reason="unsolved bug in geometry calculations", strict=True
            ),
        )
    else:
        param = path
    map_params.append(param)
