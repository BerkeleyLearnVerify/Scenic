import trimesh
from zipfile import ZipFile

# Unzip mesh files
with ZipFile(localPath('meshes.zip'), 'r') as zf:
    zf.extractall(localPath("."))

# Load plane mesh from file and create plane shape from it
plane_shape = MeshShape.fromFile(path=localPath("meshes/plane.obj"), type="obj", initial_rotation=(-90 deg, 0, -10 deg))

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
