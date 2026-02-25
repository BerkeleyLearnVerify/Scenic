import math
import warnings

import carla
import scipy

from scenic.core.geometry import normalizeAngle
from scenic.core.vectors import Orientation, Vector


def _snapToGround(world, location, blueprint):
    """Mutates @location to have the same z-coordinate as the nearest waypoint in @world."""
    waypoint = world.get_map().get_waypoint(location)
    # patch to avoid the spawn error issue with vehicles and walkers.
    z_offset = 0
    if blueprint is not None and ("vehicle" in blueprint or "walker" in blueprint):
        z_offset = 0.5

    location.z = waypoint.transform.location.z + z_offset
    return location


def scenicToCarlaLocation(pos, world=None, blueprint=None, snapToGround=False):
    warnings.warn(
        "scenicToCarlaLocation is deprecated; use scenicToCarlaTransform(...).location",
        DeprecationWarning,
    )
    if snapToGround:
        assert world is not None
        return _snapToGround(world, carla.Location(pos.x, -pos.y, 0.0), blueprint)
    return carla.Location(pos.x, -pos.y, pos.z)


def scenicToCarlaVector3D(x, y, z=0.0):
    # NOTE: Used for velocity, acceleration; superclass of carla.Location
    z = 0.0 if z is None else z
    return carla.Vector3D(x, -y, z)


def scenicToCarlaRotation(orientation):
    # CARLA uses intrinsic yaw, pitch, roll rotations (in that order), like Scenic,
    # but with yaw being left-handed and with zero yaw being East.
    yaw, pitch, roll = orientation.r.as_euler("ZXY", degrees=True)
    yaw = -yaw - 90
    return carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)


def scenicToCarlaTransform(obj, *, world=None, snapToGround=False, pos=None):
    if pos is None:
        pos = obj.position
    desired_rot = scenicToCarlaRotation(obj.orientation)

    if not snapToGround:
        # 3D maps: Scenic already provides the true 3D pose (including z), so use it directly.
        loc = carla.Location(pos.x, -pos.y, pos.z)
        return carla.Transform(loc, desired_rot)

    assert world is not None
    # Snap (x, y) to the nearest map waypoint to get surface z + lane pitch/roll.
    # Note: CARLA "waypoints" are lane-centered; LaneType.Any may include sidewalk/shoulder/etc.
    waypoint = world.get_map().get_waypoint(
        carla.Location(pos.x, -pos.y, 0.0),
        project_to_road=True,
        lane_type=carla.LaneType.Any,
    )
    ground_loc = waypoint.transform.location
    ground_rot = waypoint.transform.rotation

    # Keep Scenic yaw, but follow the local surface for pitch/roll when snapping.
    rot = carla.Rotation(
        pitch=ground_rot.pitch, yaw=desired_rot.yaw, roll=ground_rot.roll
    )
    loc = carla.Location(pos.x, -pos.y, ground_loc.z)

    if obj.blueprint is not None and (
        "vehicle" in obj.blueprint or "walker" in obj.blueprint
    ):
        # Offset from contact point to object center, rotated to follow the surface normal.
        base_offset = Vector(0, 0, -obj.height / 2)
        contact_tolerance = 1e-4
        offset = (Vector(0, 0, contact_tolerance / 2) - base_offset).rotatedBy(
            carlaToScenicOrientation(ground_rot)
        )

        loc.x += offset.x
        loc.y += -offset.y
        loc.z += offset.z

    return carla.Transform(loc, rot)


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
