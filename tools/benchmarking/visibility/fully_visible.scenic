""" Tests visibility calculation time on a complex mesh that doesn't contain its center """

from pathlib import Path

import visibility_benchmarking

ego = new Object with viewRayDensity globalParameters.viewRayDensity

chair_shape = MeshShape.fromFile(path=Path(localPath(".")).parent.parent / "meshes" / "chair.obj.bz2", initial_rotation=(0,90 deg,0))

target_chair = new Object with shape chair_shape, at (0,10,0),
	with width 3, with length 3, with height 3, visible
