"""Scenic model for Mars rover scenarios in Webots."""

# Set up workspace
width = 5
height = 5
workspace = Workspace(RectangularRegion(0 @ 0, 0, width, height))

# types of objects

class Goal:
	"""Flag indicating the goal location."""
	width: 0.3
	height: 0.3
	webotsType: 'GOAL'

class Rover:
	"""Mars rover."""
	width: 0.5
	height: 0.7
	webotsType: 'ROVER'

class Debris:
	"""Abstract class for debris scattered randomly in the workspace."""
	position: Point in workspace
	heading: Range(0, 360) deg

class BigRock(Debris):
	"""Large rock."""
	width: 0.17
	height: 0.17
	webotsType: 'ROCK_BIG'

class Rock(Debris):
	"""Small rock."""
	width: 0.10
	height: 0.10
	webotsType: 'ROCK_SMALL'

class Pipe(Debris):
	"""Pipe with variable length."""
	width: 0.2
	height: Range(0.5, 1.5)
	webotsType: 'PIPE'
