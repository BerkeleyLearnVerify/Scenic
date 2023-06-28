import trimesh
from pathlib import Path

# Load chair mesh from file and create chair shape from it
chair_shape = MeshShape.fromFile(localPath("../../../assets/meshes/chair.obj.bz2"), initial_rotation=(0,90 deg,0))

class Chair:
    shape: chair_shape
    length: 1
    width: 1
    height: 1

floor = new Object with width 5, with length 5, with height 0.1
air_cube = new Object at (Range(-5,5), Range(-5,5), 3), 
    facing (Range(0,360 deg), Range(0,30 deg), 0)
new Chair below air_cube, with color (0,0,0.784)
ego = new Chair below air_cube, on floor
