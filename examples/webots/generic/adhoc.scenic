"""
Generate a complicated scene with ad-hoc Webots objects
"""

model scenic.simulators.webots.model

import numpy as np
import random

import trimesh

# Pick a workspace
workspace_region = RectangularRegion(0 @ 0, 0, 40.1, 40.1)
workspace = Workspace(workspace_region)

air_vf = VectorField("TestVF", lambda pos: (42 deg, 45 deg, 52 deg))
air_region = BoxRegion(dimensions=(30,30,30), position=(0,0,15), orientation=air_vf)

# Create a webots class that automatically creates adhoc objects.
class AdhocObject(WebotsObject):
  webotsAdhoc: {}

# Place a large cube in the workspace, which should inherit the
# workspace's parentOrientation. 
air_cube = new AdhocObject in air_region,
    with width 5,
    with length 5,
    with height 5,
    with viewAngles (90 deg, 45 deg),
    with visibleDistance 10

# Place a small cone on the air_cube, which should automatically
# have it's parent orientation set to make its bounding box
# flush with the face.
small_air_cone = new AdhocObject on air_cube.topSurface,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1)),
    with viewAngles (60 deg, 30 deg),
    with visibleDistance 5

# Create floor region
floor = new AdhocObject at (0,0,0),
    with shape BoxShape(dimensions=(40,40,0.1))

# Place a small cone below the air_cube, and another on the floor below the air_cube.
small_below_cone = new AdhocObject below air_cube,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1))

small_floor_cone = new AdhocObject below air_cube, on floor.topSurface,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1))

# Create large chair object
chair = new AdhocObject on floor.topSurface,
    with shape MeshShape.fromFile(localPath("../../../assets/meshes/chair.obj.bz2"))

ego = chair

# Place a small cube on the large chair
new AdhocObject on chair.topSurface
