import carla
import math
from scenic.core.vectors import Vector
from scenic.core.geometry import normalizeAngle  # TODO: understand what normalizeAngle does


def scenicToCarlaLocation(pos, z=0.0):
	z = 0.0 if z is None else z
	return carla.Location(pos.x, pos.y, z)

def scenicToCarlaRotation(heading):
	# NOTE: Scenic in degrees counterclockwise from forward vector
	yaw = 180 - math.degrees(heading)  # TODO: make sure this is correct
	return carla.Rotation(yaw=yaw)

def scenicToCarlaVector3D(measure, z=0.0):
	# NOTE: carla.Vector3D used for velocity, acceleration; superclass of carla.Location
	z = 0.0 if z is None else z
	return carla.Vector3D(measure.x, measure.y, z)

def carlaToScenicPosition(loc):
	return Vector(loc.x, loc.y)  # TODO: make sure loc.y is correct

def carlaToScenicElevation(loc):
	return loc.z  # TODO: make sure this is correct

def carlaToScenicHeading(rot, tolerance2D=0):
	# NOTE: Scenic only defines yaw
	if abs(rot.pitch) > tolerance2D or abs(rot.roll) > tolerance2D:
		return None
	return normalizeAngle(math.radians(180 - rot.yaw))  # TODO: make sure this is correct
