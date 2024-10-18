import argparse
import json
import os
import pprint
import re
import sys
import tempfile
import time
from warnings import warn

import airsim
import cv2
import numpy as np
import trimesh

from scenic.core.shapes import MeshShape
from scenic.core.utils import repairMesh
from scenic.simulators.airsim.utils import (
    airsimToScenicLocationTuple,
    airsimToScenicOrientationTuple,
)

# get output directory
parser = argparse.ArgumentParser()
parser.add_argument(
    "-p",
    "--path",
    type=str,
    help="the directory where the obj to be reparied is. This file should already exist.",
    required=True,
)
args = parser.parse_args()


objPath = args.path 

assert objPath.lower().endswith(".obj")

def repairMeshShape(path):
    tmesh = trimesh.load(path)

    tmesh = repairMesh(tmesh)

    with open(
        path,
        "w",
    ) as outfile:
        outfile.write(trimesh.exchange.obj.export_obj(tmesh))

repairMeshShape(objPath)