import airsim
import scipy

from scenic.core.type_support import toVector
from scenic.core.vectors import Orientation, Vector


def tupleToVector3r(tuple):
    return airsim.Vector3r(tuple[0], tuple[1], tuple[2])


def scenicToAirsimOrientation(orientation):
    pitch, yaw, roll = orientation.r.as_euler("XZY", degrees=False)
    return airsim.to_quaternion(pitch, roll, yaw)


def airsimToScenicOrientation(orientation):

    # convert to unreal

    pitch, roll, yaw = airsim.to_eularian_angles(orientation)
    angles = (pitch, roll, yaw)

    conversion = 180 / 3.14
    conversioned = (
        angles[0] * conversion,
        angles[1] * conversion,
        angles[2] * conversion,
    )
    # print("angles = ,", conversioned)

    r = scipy.spatial.transform.Rotation.from_euler(
        seq="XYZ", angles=angles, degrees=False
    )
    return Orientation(r)


def airsimToScenicOrientationTuple(orientation):
    # ! DONT TOUCH NOW
    # intrinsic angles
    pitch, roll, yaw = airsim.to_eularian_angles(orientation)
    # angles = (pitch, yaw, roll)
    angles = (-yaw, pitch, roll)
    return angles


def scenicToAirsimLocation(position, centerOffset):

    # turn position to a vector
    position = Vector(position.x, position.y, position.z)

    # undo airsim.Vector3r's future adjustments
    position = (position - centerOffset) / 100  # divide by 100
    position = Vector(position.x, position.y, -position.z)  # negate z axis

    airsimPos = airsim.Vector3r(
        position.x, position.y, position.z
    )  # multiplies by 100 and negates z axis

    return airsimPos


def airsimToScenicLocation(position):
    loc = Vector(
        position.x_val,
        position.y_val,
        position.z_val,
    )

    # convert to scenic
    loc = Vector(loc.x, -loc.y, loc.z)  # left hand coords
    loc = loc / 100  # account for mesh scaling by .01

    return loc


def scenicToAirsimScale(size, dims):
    # movment function in meters
    # drone size in blender is 98.1694 m
    # coords scaled by 100? https://microsoft.github.io/AirSim/apis/#:~:text=All%20AirSim%20API%20uses%20NED,in%20centimeters%20instead%20of%20meters.
    scaleFactor = 1
    # return airsim.Vector3r(size.x / dims.x, size.y / dims.y, size.z / dims.z)
    return airsim.Vector3r(1, 1, 1)


_prexistingObjs = {}


def _addPrexistingObj(obj):
    _prexistingObjs[obj.name] = obj


def getPrexistingObj(objName):
    return _prexistingObjs[objName]
