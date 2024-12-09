import airsim
import scipy

from scenic.core.type_support import toVector
from scenic.core.vectors import Orientation, Vector


def tupleToVector3r(tuple):
    return airsim.Vector3r(tuple[0], tuple[1], tuple[2])


def scenicToAirsimOrientation(orientation):
    # pitch, yaw, roll = orientation.r.as_euler("XZY", degrees=False)
    rad90 = 1.5708
    yaw, pitch, roll = orientation.r.as_euler("ZXY", degrees=False)
    yaw = -yaw - rad90
    return airsim.to_quaternion(pitch=pitch, yaw=yaw, roll=roll)


def airsimToScenicOrientation(orientation):

    pitch, roll, yaw = airsim.to_eularian_angles(orientation)
    angles = (pitch, -yaw - 90, roll)
    r = scipy.spatial.transform.Rotation.from_euler(
        seq="ZXY", angles=angles, degrees=True
    )
    orientation = Orientation(r)
    return Orientation(r)


def airsimToScenicOrientationTuple(orientation):
    # ! DONT TOUCH NOW
    # intrinsic angles
    pitch, roll, yaw = airsim.to_eularian_angles(orientation)
    # angles = (pitch, yaw, roll)
    angles = (-yaw, pitch, roll)
    return angles


# forPose - if the output will be fed into airsim's airsim.Pose() function
def scenicToAirsimLocation(position, centerOffset, forPose=True):

    print("pos=", position, "center offset = ", centerOffset)

    # turn position to a vector
    position = Vector(position.x, position.y, position.z)

    # convert to unreal
    position *= 100
    position = Vector(position.x, -position.y, position.z)  # left hand coords

    position -= centerOffset

    if forPose:
        # undo airsim's adjustments
        position /= 100  # divide by 100
        position = Vector(position.x, position.y, -position.z)  # negate z axis

    position = airsim.Vector3r(
        position.x, position.y, position.z
    )  # multiplies by 100 and negates z axis

    return position


# fromPose - if position was gained from simGetObjectPose
def airsimToScenicLocation(position, centerOffset, fromPose=True):

    position = Vector(
        position.x_val,
        position.y_val,
        position.z_val,
    )

    if fromPose:
        # undo airsim's adjustments
        position *= 100  # mult by 100
        position = Vector(position.x, position.y, -position.z)  # negate z axis

    position += centerOffset

    # convert to scenic
    position /= 100  # account for mesh scaling by .01
    position = Vector(position.x, -position.y, position.z)  # left hand coords

    return position


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
