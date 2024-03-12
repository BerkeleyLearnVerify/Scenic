import math
from pathlib import Path

import pytest
import shapely.geometry
import trimesh.voxel

from scenic.core.object_types import Object, OrientedPoint
from scenic.core.regions import *
from scenic.core.vectors import VectorField
from tests.utils import sampleSceneFrom

plane_region = MeshVolumeRegion.fromFile("assets/meshes/classic_plane.obj.bz2")
vr = plane_region.voxelized(max(plane_region.mesh.extents) / 100)
mr = vr.mesh()
