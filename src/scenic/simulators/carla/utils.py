import carla
import math
from scenic.core.vectors import Vector
from scenic.core.geometry import normalizeAngle  # TODO: understand what normalizeAngle does

def scenicToCarlaLocation(pos, z=0):
	return carla.Location(pos.x, pos.y, z)

def scenicToCarlaRotation(heading):
	# NOTE: Scenic in degrees counterclockwise from forward vector
	yaw = 180 - math.degrees(heading)  # TODO: make sure this is correct
	return carla.Rotation(yaw=yaw)

def carlaToScenicPosition(loc):
	return Vector(loc.x, loc.y)  # TODO: make sure loc.y is correct

def carlaToScenicElevation(loc):
	return loc.z  # TODO: make sure this is correct

def carlaToScenicHeading(rot, tolerance2D=0):
	# NOTE: Scenic only defines yaw
	if abs(rot.pitch) > tolerance2D or abs(rot.roll) > tolerance2D:
		return None
	return normalizeAngle(math.radians(180 - rot.yaw))  # TODO: make sure this is correct
