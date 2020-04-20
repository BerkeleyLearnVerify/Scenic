
import math

# map
width = 5
height = 5
workspace = Workspace(RectangularRegion(0 @ 0, 0, width, height))

# types of objects

class Goal:
	width: 0.3
	height: 0.3
	webotsType: 'GOAL'

class Robot:
	width: 0.5
	height: 0.7
	webotsType: 'ROVER'

class Debris:
	position: Point in workspace
	heading: (-math.pi, math.pi)

class BigRock(Debris):
	width: 0.17
	height: 0.17
	webotsType: 'ROCK_BIG'

class Rock(Debris):
	width: 0.10
	height: 0.10
	webotsType: 'ROCK_SMALL'

class Pipe(Debris):
	width: 0.2
	height: (0.5, 1.5)
	webotsType: 'PIPE'
