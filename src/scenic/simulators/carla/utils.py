import carla
import math
from scenic.core.vectors import Vector
from scenic.core.geometry import normalizeAngle

def snapToGround(world, location):
	"""Mutates @location to have the same z-coordinate as the nearest waypoint in @world."""
	waypoint = world.get_map().get_waypoint(location)
	location.z = waypoint.transform.location.z + 1
	return location

def scenicToCarlaLocation(pos, z=0.0, world=None):
	if world is not None:
		return snapToGround(world, carla.Location(pos.x, -pos.y, 0.0))
	elif z is not None:
		return carla.Location(pos.x, -pos.y, z)
	else:
		return carla.Location(pos.x, -pos.y, 0.0)

def scenicToCarlaRotation(heading):
	yaw = math.degrees(-heading) - 90
	return carla.Rotation(yaw=yaw)

def scalarToCarlaVector3D(x, y, z=0.0):
	# NOTE: Used for velocity, acceleration; superclass of carla.Location
	z = 0.0 if z is None else z
	return carla.Vector3D(x, y, z)

def carlaToScenicPosition(loc):
	return Vector(loc.x, -loc.y)

def carlaToScenicElevation(loc):
	return loc.z

def carlaToScenicHeading(rot, tolerance2D=5.0):
	# NOTE: Scenic only defines yaw
	if abs(rot.pitch) > tolerance2D or abs(rot.roll) > tolerance2D:
		return None
	return normalizeAngle(-math.radians(rot.yaw + 90))
