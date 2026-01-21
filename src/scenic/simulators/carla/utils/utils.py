import math

import carla
import scipy

from scenic.core.geometry import normalizeAngle
from scenic.core.vectors import Orientation, Vector


def _snapToGround(world, location):
    """Mutates @location to have the same z-coordinate as the nearest waypoint in @world."""
    waypoint = world.get_map().get_waypoint(location)
    location.z = waypoint.transform.location.z
    return location


def scenicToCarlaVector3D(x, y, z=0.0):
    # NOTE: Used for velocity, acceleration; superclass of carla.Location
    z = 0.0 if z is None else z
    return carla.Vector3D(x, -y, z)


def scenicToCarlaLocation(pos, world=None, blueprint=None, snapToGround=False):
    if snapToGround:
        assert world is not None
        loc = _snapToGround(world, carla.Location(pos.x, -pos.y, 0.0))
    else:
        loc = carla.Location(pos.x, -pos.y, pos.z)

    # Spawn patch for vehicles/walkers (needed in both 2D and 3D)
    if blueprint is not None and ("vehicle" in blueprint or "walker" in blueprint):
        loc.z += 0.5

    return loc


def scenicToCarlaRotation(orientation):
    # CARLA uses intrinsic yaw, pitch, roll rotations (in that order), like Scenic,
    # but with yaw being left-handed and with zero yaw being East.
    yaw, pitch, roll = orientation.r.as_euler("ZXY", degrees=True)
    yaw = -yaw - 90
    return carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)


def carlaToScenicPosition(loc):
    return Vector(loc.x, -loc.y, loc.z)


def carlaToScenicElevation(loc):
    return loc.z


def carlaToScenicOrientation(rot):
    angles = (-rot.yaw - 90, rot.pitch, rot.roll)
    r = scipy.spatial.transform.Rotation.from_euler(
        seq="ZXY", angles=angles, degrees=True
    )
    return Orientation(r)


def carlaToScenicHeading(rot):
    return normalizeAngle(-math.radians(rot.yaw + 90))


def carlaToScenicAngularSpeed(vel):
    return math.hypot(math.radians(vel.x), -math.radians(vel.y), math.radians(vel.y))


def carlaToScenicAngularVel(vel):
    return Vector(math.radians(vel.x), -math.radians(vel.y), math.radians(vel.y))


_scenicToCarlaMap = {
    "red": carla.TrafficLightState.Red,
    "green": carla.TrafficLightState.Green,
    "yellow": carla.TrafficLightState.Yellow,
    "off": carla.TrafficLightState.Off,
    "unknown": carla.TrafficLightState.Unknown,
}


def scenicToCarlaTrafficLightStatus(status):
    return _scenicToCarlaMap.get(status, None)


def carlaToScenicTrafficLightStatus(status):
    return str(status).lower()
