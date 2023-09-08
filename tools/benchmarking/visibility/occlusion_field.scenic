""" Tests visibility calculation time on a complex mesh, completely occluded by another complex mesh"""

from pathlib import Path

import visibility_benchmarking

ego = new Object with viewRayDensity globalParameters.viewRayDensity

chair_shape = MeshShape.fromFile(path=Path(localPath(".")).parent.parent / "meshes" / "chair.obj.bz2", initial_rotation=(0,90 deg,0))

# Create field of occluding spheres
class OccludingSphere:
	shape: SpheroidShape()
	length: Range(1,1.5)
	width: Range(1,1.5)
	height: Range(1,1.5)
	allowCollisions: True

occlusion_zone = BoxRegion(position=(0,5,0), dimensions=(5,1,5))
for _ in range(50):
	new OccludingSphere in occlusion_zone

target_zone = BoxRegion(position=(0,10,0), dimensions=(4,1,4))
target_chair = new Object with shape chair_shape, at (0,10,0),
	with width 3, with length 3, with height 3

param visibility = ego can see target_chair
