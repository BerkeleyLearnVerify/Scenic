import trimesh

import numpy as np
import random
np.random.seed(random.getrandbits(32))

# Pick a workspace
workspace_region = RectangularRegion(0 @ 0, 0, 40.1, 40.1)
workspace = Workspace(workspace_region)

air_vf = VectorField("TestVF", lambda pos: (42 deg, 45 deg, 52 deg))
air_region = BoxRegion(dimensions=(30,30,30), position=(0,0,15), orientation=air_vf)

# Place a large cube in the workspace, which should inherit the
# workspace's parentOrientation. 
air_cube = new Object in air_region,
    with width 5,
    with length 5,
    with height 5,
    with viewAngles (90 deg, 45 deg),
    with visibleDistance 10

# Place a small cone on the air_cube, which should automatically
# have it's parent orientation set to make its bounding box
# flush with the face.
small_air_cone = new Object on air_cube.topSurface,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1)),
    with viewAngles (60 deg, 30 deg),
    with visibleDistance 5

# Create floor region
floor = new Object at (0,0,0),
    with shape BoxShape(dimensions=(40,40,0.1))

# Place a small cone below the air_cube, and another on the floor below the air_cube.
small_below_cone = new Object below air_cube,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1))

small_floor_cone = new Object below air_cube, on floor.topSurface,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1))

# Load chair mesh from file and create chair shape from it
with open(localPath("chair.obj"), "r") as mesh_file:
    mesh = trimesh.load(mesh_file, file_type="obj")

chair_shape = MeshShape(mesh, dimensions=(5,5,5), initial_rotation=(0,90 deg,0))

# Create large chair object
chair = new Object on floor.topSurface,
    with shape chair_shape

ego = chair

# Place a small cube on the large chair
new Object on chair.topSurface

# Place a small cube in the empty space of the large chair
new Object in chair.boundingBox, on floor.topSurface

# Create large chair object fallen on the floor on a random side
fall_orientation = Uniform((0, -90 deg, 0), (0, 90 deg, 0), (0, 0, -90 deg), (0, 0, 90 deg))

fallen_chair = new Object on floor,
    with shape chair_shape,
    facing fall_orientation

# Place a small cube on the large chair
new Object on fallen_chair.topSurface

# Place a small cube in the empty space of the large chair
new Object in fallen_chair.boundingBox, on floor.topSurface
