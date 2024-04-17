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

assert objPath.lower().endswith(".binvox")

def getBinvox(path):
    tmesh = trimesh.load(path)
    print(type(tmesh))
    tmesh = tmesh.marching_cubes
    print(type(tmesh))

    tmesh = repairMesh(tmesh)

    with open(
        "examples/airsim/worldInfo/nh2.obj",
        "w",
    ) as outfile:
        outfile.write(trimesh.exchange.obj.export_obj(tmesh))

getBinvox(objPath)