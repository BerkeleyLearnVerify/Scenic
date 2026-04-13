from utils import getAssetPath

workspace = Workspace(RectangularRegion(0@0, 0, 100, 100))

plane_shape = MeshShape.fromFile(
    getAssetPath("meshes/classic_plane.obj.bz2"),
    initial_rotation=(-90 deg, 0, -10 deg)
)

for i in range(4):
    new Object at Range(-40, 40) @ Range(-40, 40), with shape plane_shape