import math

from scenic.core.vectors import Vector
from scenic.core.geometry import normalizeAngle


def vectorFromHeading(heading):
	angle = -heading - math.pi/2
	return Vector(
		math.cos(angle),
		math.sin(angle)
	)

def angularSpeedFromSteer(steer, speed, length):
	return speed * math.tan(steer) / length

def headingFromVector(vec):
	x, y = vec.x, vec.y
	if x == 0 and y == 0:
		return None
	return math.atan2(y, x) - math.pi / 2