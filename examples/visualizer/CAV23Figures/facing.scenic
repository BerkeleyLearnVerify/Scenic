import trimesh
from pathlib import Path

# Load plane mesh from file and create plane shape from it
plane_shape = MeshShape.fromFile(localPath("../../../assets/meshes/classic_plane.obj.bz2"), initial_rotation=(-90 deg, 0, -10 deg))

class Plane:
    shape: plane_shape
    length: 2
    width: 2
    height: 1

class Ball:
    shape: SpheroidShape()

ego = new Ball at (0,0, 1.25)
new Plane at (2,0,0), facing toward ego
new Plane at (-2,0,0), facing directly toward ego
