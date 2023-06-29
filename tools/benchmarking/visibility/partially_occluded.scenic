""" Tests visibility calculation time on two walls, one almost entirely obscuring the other. """

from pathlib import Path

from scenic.core.distributions import distributionFunction

import visibility_benchmarking


ego = new Object with viewRayDensity globalParameters.viewRayDensity

@distributionFunction
def make_obscuring_wall_shape(hole_loc):
	base_wall = BoxRegion(dimensions=(5,0.1,5))
	hole = BoxRegion(dimensions=(0.1, 1, 0.1), position=hole_loc)
	hole_wall = base_wall.difference(hole)
	hole_shape = MeshShape(hole_wall.mesh)
	return hole_shape

class ObscuringWall:
	hole: (Range(-2.44,2.44), 0, Range(-2.44,2.44))
	shape: make_obscuring_wall_shape(self.hole)

obscuring_wall = new ObscuringWall at (0, 2.54999, 0), with length 0.1, 
	with width 5, with height 5, with color (200,0,0)

target_wall = new Object at (0, 2.65, 0), with length 0.1,
	with width 5, with height 5, with color (0,0,200), visible
