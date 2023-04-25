import trimesh

import numpy as np
import random

air_region = BoxRegion(dimensions=(1.25, 1.25, 1.25)).difference(BoxRegion())

# Create wall region
center_block = new Object

# Load chair mesh from file and create chair shape from it
with open(localPath("chair.obj"), "r") as mesh_file:
    mesh = trimesh.load(mesh_file, file_type="obj")

chair_shape = MeshShape(mesh, dimensions=(0.5,0.5,0.5), initial_rotation=(0,90 deg,0))

# Create chair object
ego = new Object in air_region, on center_block.surface,
    with onDirection Uniform((1,0,0),(0,1,0),(0,0,1)), with shape chair_shape
