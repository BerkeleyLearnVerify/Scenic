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


def scenicToAirsimLocation(position):

    position = toVector(position) * 10
    print(position)
    return airsim.Vector3r(position.x, position.y, position.z)
    # return airsim.Vector3r(0, 0, 0)


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


def scenicToAirsimScale(width, length, height):
    # movment function in meters
    # drone size in blender is 98.1694 m
    # coords scaled by 100? https://microsoft.github.io/AirSim/apis/#:~:text=All%20AirSim%20API%20uses%20NED,in%20centimeters%20instead%20of%20meters.
    scaleFactor = 1
    return airsim.Vector3r(
        width * scaleFactor, length * scaleFactor, height * scaleFactor
    )


_prexistingObjs = {}


def _addPrexistingObj(obj):
    _prexistingObjs[obj.name] = obj


def getPrexistingObj(objName):
    return _prexistingObjs[objName]
