"""Interface to Webots for dynamic simulations."""

from collections import defaultdict
import math
import pickle

import scenic.simulators.webots.utils as utils
from scenic.core.simulators import Simulator, Simulation, Action
from scenic.core.vectors import Vector

class WebotsSimulator(Simulator):
    """`Simulator` object for Webots.

    Args:
        supervisor: Supervisor node handle from the Webots Python API.
    """
    def __init__(self, supervisor):
        super().__init__()
        self.supervisor = supervisor

    def createSimulation(self, scene, verbosity=0):
        return WebotsSimulation(scene, self.supervisor)

class WebotsSimulation(Simulation):
    def __init__(self, scene, supervisor, verbosity=0):
        timestep = supervisor.getBasicTimeStep() / 1000
        super().__init__(scene, timestep=timestep, verbosity=verbosity)
        self.supervisor = supervisor
        self.objects = scene.objects

        # Find Webots objects corresponding to Scenic objects
        self.webotsObjects = {}
        usedNames = defaultdict(lambda: 0)
        for obj in self.objects:
            if not hasattr(obj, 'webotsName'):
                raise RuntimeError(f'object {obj} does not have a webotsName property')
            if obj.webotsName:
                name = obj.webotsName
            else:
                ty = obj.webotsType
                if not ty:
                    raise RuntimeError(f'object {obj} has no webotsName or webotsType')
                nextID = usedNames[ty]
                if nextID == 0 and supervisor.getFromDef(ty):
                    name = ty
                else:
                    name = f'{ty}_{nextID}'
            webotsObj = supervisor.getFromDef(name)
            if webotsObj is None:
                raise RuntimeError(f'Webots object {name} does not exist in world')
            self.webotsObjects[obj] = webotsObj
            obj.webotsObject = webotsObj
            obj.webotsName = name

            # get starting elevation
            if obj.elevation is None:
                pos = webotsObj.getField('translation').getSFVec3f()
                obj.elevation = pos[1]

        # Reset Webots simulation
        supervisor.simulationResetPhysics()

        # Set initial properties of Webots objects
        self.writePropertiesToWebots()

    def writePropertiesToWebots(self):
        for obj in self.objects:
            webotsObj = self.webotsObjects[obj]
            # position
            pos = utils.scenicToWebotsPosition(obj.position + obj.positionOffset, y=obj.elevation)
            webotsObj.getField('translation').setSFVec3f(pos)
            # heading
            rot = utils.scenicToWebotsRotation(obj.heading)
            webotsObj.getField('rotation').setSFRotation(rot)
            # battery
            battery = getattr(obj, 'battery', None)
            if battery:
                if not isinstance(battery, (tuple, list)) or len(battery) != 3:
                    raise RuntimeError(f'"battery" of {obj.webotsName} does not'
                                       ' have 3 components')
                field = webotsObj.getField('battery')
                field.setMFFloat(0, battery[0])
                field.setMFFloat(1, battery[1])
                field.setMFFloat(2, battery[2])
            # controller
            if obj.controller:
                webotsObj.getField('controller').setSFString(obj.controller)
                if obj.resetController:
                    webotsObj.restartController()

    def createObjectInSimulator(self, obj):
        raise RuntimeError('the Webots interface does not support dynamic object creation')

    def step(self):
        ms = round(1000 * self.timestep)
        self.supervisor.step(ms)

    def getProperties(self, obj, properties):
        """Read the values of the given properties of the object from the simulation."""
        webotsObj = self.webotsObjects[obj]

        pos = webotsObj.getField('translation').getSFVec3f()
        x, y = utils.webotsToScenicPosition(pos)
        elevation = pos[1]
        rot = webotsObj.getField('rotation').getSFRotation()
        heading = utils.webotsToScenicRotation(rot, tolerance2D=None)
        lx, ly, lz, ax, ay, az = webotsObj.getVelocity()
        velocity = utils.webotsToScenicPosition((lx, ly, lz))
        speed = math.hypot(*velocity)

        values = dict(
            position=Vector(x, y),
            elevation=elevation,
            heading=heading,
            velocity=velocity,
            speed=speed,
            angularSpeed=ay,
        )

        if hasattr(obj, 'battery'):
            field = webotsObj.getField('battery')
            val = (field.getMFFloat(0), obj.battery[1], obj.battery[2])
            values['battery'] = val

        return values

class MoveAction(Action):
    def __init__(self, offset):
        self.offset = offset

    def applyTo(self, obj, webotsObj):
        pos = obj.position.offsetRotated(obj.heading, self.offset)
        pos = utils.scenicToWebotsPosition(pos, y=obj.elevation)
        webotsObj.getField('translation').setSFVec3f(pos)

class WriteFileAction(Action):
    def __init__(self, path, data):
        self.path = path
        self.data = data

    def applyTo(self, obj, webotsObj):
        with open(self.path, 'wb') as outFile:
            pickle.dump(self.data, outFile)
