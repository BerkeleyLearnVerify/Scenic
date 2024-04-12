# standard libs
import asyncio
from cmath import atan, pi, tan
import math
from math import copysign, degrees, radians, sin
import subprocess
import threading
import time

# third party libs
import airsim
import numpy as np
import scipy

# scenic libs
from scenic.core.simulators import (
    Simulation,
    SimulationCreationError,
    Simulator,
    SimulatorInterfaceWarning,
)
from scenic.core.type_support import toVector
from scenic.core.vectors import Orientation, Vector
# import scenic.simulators.airsim.MavsdkUtils as mavutils
from scenic.syntax.veneer import verbosePrint

from .utils import (
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimOrientation,
    scenicToAirsimScale,
    scenicToAirsimVector,
)

# Constants
PX4_DRONE = "PX4Drone"


class AirSimSimulator(Simulator):
    def __init__(self, timestep, idleStoragePos=(0, 0, 0)):
        # start airsim client
        try:
            client = airsim.MultirotorClient()
            client.confirmConnection()
            client.simPause(True)
        except Exception:
            raise RuntimeError("Airsim must be running on before executing scenic")

        # init properties
        self.client = client
        self.idleStoragePos = idleStoragePos
        self.timestep = timestep

        verbosePrint(
            "\n\nAll Asset Names:\n",
            [name for name in self.client.simListAssets()],
            level=2,
        )
        # call super
        super().__init__()

    def createSimulation(self, scene, **kwargs):
        return AirSimSimulation(self, scene, self.client, **kwargs)

    def destroy(self):
        super().destroy()


class AirSimSimulation(Simulation):
    # ------------------- Required Methods -------------------

    def __init__(self, simulator, scene, client, **kwargs):
        # init properties
        self.simulator = simulator
        self.client = client
        self.joinables = []  # used in waitforjoinables
        self.objTrove = []  # objs to delete on simulation complete
        self.objs = {}  # obj name to objrealname dict
        self.drones = {}  # obj name to objrealname dict only for drones
        self.PX4Drone = None
        self.startDrones = None
        self.nextDroneIndex = 0

        super().__init__(scene, **kwargs)

    def setup(self):
        # set up startDrones
        self.startDrones = self.client.listVehicles()

        # remove px4 drone from start drones bc it is handled differently
        if PX4_DRONE in self.startDrones:
            self.startDrones.remove(PX4_DRONE)

        # move all drones (except px4 drone) to offscreen position
        self.client.simPause(False)
        for i, drone in enumerate(self.startDrones):
            newPose = airsim.Pose(
                position_val=scenicToAirsimVector(self.simulator.idleStoragePos)
                + airsim.Vector3r(i, 0, 0)
            )
            self.client.enableApiControl(True, drone)
            self.client.simSetVehiclePose(
                vehicle_name=drone, pose=newPose, ignore_collision=False
            )
            self.client.moveByVelocityAsync(
                0, 0, 0, 1, vehicle_name=drone
            )  # make drone hover

        self.client.simPause(True)

        # create objs
        super().setup()

        # ensure that drones are in the correct places
        self.client.simPause(False)
        time.sleep(1)
        self.client.simPause(True)

    def createObjectInSimulator(self, obj):
        # create AirSimPrexisting
        if obj.blueprint == "AirSimPrexisting":
            self.objs[obj.name] = obj.name
            return

        # set obj name if no name specified
        if not obj.name:
            obj.name = str(hash(obj))

        # ensure obj isn't already in world
        if obj.name in self.objs:
            raise RuntimeError(
                "there is already an object of the name " + obj.name + " in the simulator"
            )

        # set default realObjName
        realObjName = obj.name + str(hash(obj))

        # set object airsim pose
        pose = airsim.Pose(
            position_val=scenicToAirsimVector(obj.position),
            orientation_val=scenicToAirsimOrientation(obj.orientation),
        )

        # create obj in airsim
        if obj.blueprint == "Drone":
            realObjName = "Drone" + str(self.nextDroneIndex)
            obj._startPos = obj.position

            # if there is an avalible drone, take it, else create one
            if self.nextDroneIndex < len(self.startDrones):
                realObjName = self.startDrones[self.nextDroneIndex]
                obj.realObjName = realObjName
            else:
                self.client.simAddVehicle(
                    vehicle_name=realObjName, vehicle_type="simpleflight", pose=pose
                )

            self.nextDroneIndex += 1

            # save the drone name
            obj.realObjName = realObjName
            self.objs[obj.name] = realObjName
            self.drones[obj.name] = realObjName

            # start the drone and place it in the world
            self.client.enableApiControl(True, realObjName)
            self.client.armDisarm(True, realObjName)
            self.client.simSetVehiclePose(
                vehicle_name=realObjName, pose=pose, ignore_collision=True
            )

            # set propellers on or off
            if obj.startHovering:
                self.client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=realObjName)
            else:
                # shut off drone propellers
                self.client.moveByVelocityAsync(0, 0, 0, -1, vehicle_name=realObjName)

        elif obj.blueprint == "PX4Drone":
            realObjName = PX4_DRONE

            if self.PX4Drone:
                raise RuntimeError("more than 1 px4 drone is not currently supported")

            self.PX4Drone = PX4_DRONE
            self.client.simSetVehiclePose(
                vehicle_name=realObjName, pose=pose, ignore_collision=True
            )

            obj.realObjName = realObjName
            self.objs[obj.name] = realObjName

        elif obj.blueprint == "StaticObj":
            # ensure user is creating an object that uses an existing asset
            if not (obj.assetName in self.client.simListAssets()):
                raise RuntimeError(
                    "no asset of name found: "
                    + obj.assetName
                    + "\n use one of these assets:\n"
                    + self.client.simListAssets()
                )

            # create obj in airsim
            realObjName = self.client.simSpawnObject(
                object_name=realObjName,
                asset_name=obj.assetName,
                pose=pose,
                scale=scenicToAirsimScale(obj),
                physics_enabled=obj.physEnabled,
                is_blueprint=False,
            )

            # add obj to sim lists
            obj.realObjName = realObjName
            self.objs[obj.name] = realObjName
            self.objTrove.append(realObjName)

        else:
            raise RuntimeError("object blueprint does not exist", obj.blueprint)

    def step(self):
        self.client.simContinueForTime(self.simulator.timestep)

    # ------------------- Other Simulator methods -------------------

    def destroy(self):
        # reinstantiate client
        client = self.client
        client.client._loop.stop()  # stop any running tasks to prevent errors

        client.simPause(True)

        # destroy all objs
        for obj_name in self.objTrove:
            client.simDestroyObject(obj_name)

        for droneName, realDroneName in self.drones.items():
            client.cancelLastTask(vehicle_name=realDroneName)
            client.moveByVelocityAsync(0, 0, 0, -1, vehicle_name=realDroneName)

        # reset the client
        client.reset()

        super().destroy()
        print("canceled simulation")

    def getProperties(self, obj, properties):
        if obj.blueprint == "AirSimPrexisting":
            return dict(
                position=obj.position,
                velocity=Vector(0, 0, 0),
                speed=0,
                angularSpeed=0,
                angularVelocity=Vector(0, 0, 0),
                yaw=0,
                pitch=0,
                roll=0,
            )
        objName = self.objs[obj.name]

        pose = None
        velocity, speed, angularSpeed, angularVelocity = None, None, None, None

        # get obj data
        if obj.blueprint == "Drone" or obj.blueprint == "PX4Drone":
            pose = self.client.simGetVehiclePose(objName)
            kinematics = self.client.simGetGroundTruthKinematics(objName)
            velocity = airsimToScenicLocation(kinematics.linear_velocity)

            angularVelocity = airsimToScenicLocation(kinematics.angular_velocity)

        elif obj.blueprint == "StaticObj" or obj.blueprint == "AirSimPrexisting":
            pose = self.client.simGetObjectPose(objName)

            # static objs don't have velocity
            velocity = Vector(0, 0, 0)
            angularVelocity = Vector(0, 0, 0)
        else:
            raise RuntimeError("object blueprint does not exist", obj.blueprint)

        # convert values
        globalOrientation = airsimToScenicOrientation(pose.orientation)
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(globalOrientation)

        location = airsimToScenicLocation(pose.position)

        speed = math.hypot(velocity.x, velocity.y, velocity.z)
        angularSpeed = math.hypot(angularVelocity.x, angularVelocity.y, angularVelocity.z)

        values = dict(
            position=location,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )

        return values
