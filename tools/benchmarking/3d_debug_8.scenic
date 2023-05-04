import trimesh

import numpy as np
import random
np.random.seed(random.getrandbits(32))

# Pick a workspace
workspace_region = RectangularRegion(0 @ 0, 0, 40.1, 40.1)
workspace = Workspace(workspace_region)

air_vf = VectorField("TestVF", lambda pos: (42 deg, 45 deg, 52 deg))
air_region = BoxRegion(dimensions=(30,30,30), position=(0,0,15), orientation=air_vf)

# Create wall region
wall = new Object at (0,0,20),
    with shape BoxShape(dimensions=(40,0.5,40))

# Load chair mesh from file and create chair shape from it
with open(localPath("chair.obj"), "r") as mesh_file:
    mesh = trimesh.load(mesh_file, file_type="obj")

chair_shape = MeshShape(mesh, dimensions=(5,5,5), initial_rotation=(0,90 deg,0))

# Create chair object
ego = new Object in air_region, on wall.surface,
    with shape chair_shape, with onDirection (0,1,0)