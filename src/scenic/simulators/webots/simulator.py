
import pickle

import scenic.simulators as simulators
import scenic.simulators.webots.utils as utils
from scenic.core.vectors import Vector

class WebotsSimulator(simulators.Simulator):
    def __init__(self, supervisor):
        super().__init__()
        self.supervisor = supervisor

    def createSimulation(self, scene):
        return WebotsSimulation(scene, self.supervisor)

class WebotsSimulation(simulators.Simulation):
    def __init__(self, scene, supervisor):
        super().__init__(scene)
        self.supervisor = supervisor
        self.timeStep = int(supervisor.getBasicTimeStep())
        self.objects = scene.objects

        # Find Webots objects corresponding to Scenic objects
        self.webotsObjects = {}
        for obj in self.objects:
            if not hasattr(obj, 'webotsName'):
                raise RuntimeError(f'object {obj} does not have a webotsName property')
            name = obj.webotsName
            webotsObj = supervisor.getFromDef(name)
            if webotsObj is None:
                raise RuntimeError(f'Webots object {name} does not exist in world')
            self.webotsObjects[obj] = webotsObj
            obj.webotsObject = webotsObj

            # get starting elevation
            pos = webotsObj.getField('translation').getSFVec3f()
            obj.elevation = pos[1]

        # Reset Webots and object controllers
        supervisor.simulationResetPhysics()
        for webotsObj in self.webotsObjects.values():
            webotsObj.restartController()

        # Set initial positions and orientations of Webots objects
        self.writePropertiesToWebots()

    def writePropertiesToWebots(self):
        for obj in self.objects:
            webotsObj = self.webotsObjects[obj]
            # position
            pos = utils.scenicToWebotsPosition(obj.position, y=obj.elevation)
            webotsObj.getField('translation').setSFVec3f(pos)
            # heading
            rot = utils.scenicToWebotsRotation(obj.heading)
            webotsObj.getField('rotation').setSFRotation(rot)

    def readPropertiesFromWebots(self):
        for obj in self.objects:
            webotsObj = self.webotsObjects[obj]
            # webotsObject
            obj.webotsObject = webotsObj
            # position
            pos = webotsObj.getField('translation').getSFVec3f()
            x, y = utils.webotsToScenicPosition(pos)
            obj.position = Vector(x, y)
            # heading
            rot = webotsObj.getField('rotation').getSFRotation()
            heading = utils.webotsToScenicRotation(rot, tolerance2D=100)
            if heading is None:
                raise RuntimeError(f'{webotsObj} has non-planar orientation!')
            obj.heading = heading

    def currentState(self):
        return tuple(obj.position for obj in self.objects)

    def initialState(self):
        return self.currentState()

    def step(self, actions):
        # execute actions
        for agent, action in actions.items():
            if action is not None:
                action.applyTo(agent, self.webotsObjects[agent])
        # run simulation for one time step
        self.supervisor.step(self.timeStep)
        # read back the results of the simulation
        self.readPropertiesFromWebots()
        return self.currentState()

class MoveAction(simulators.Action):
    def __init__(self, offset):
        self.offset = offset

    def applyTo(self, obj, webotsObj):
        pos = obj.position.offsetRotated(obj.heading, self.offset)
        pos = utils.scenicToWebotsPosition(pos, y=obj.elevation)
        webotsObj.getField('translation').setSFVec3f(pos)

class WriteFileAction(simulators.Action):
    def __init__(self, path, data):
        self.path = path
        self.data = data

    def applyTo(self, obj, webotsObj):
        with open(self.path, 'wb') as outFile:
            pickle.dump(self.data, outFile)
