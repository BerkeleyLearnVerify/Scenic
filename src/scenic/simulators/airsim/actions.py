from scenic.core.simulators import Action
from scenic.core.type_support import toVector

from .utils import (
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimOrientation,
    scenicToAirsimScale,
    scenicToAirsimVector,
)

# A Generic Template to Define an Action using AirSim's API
'''
class NameOfAction(Action): # Make sure to always inherit the "Action" Class
    # The Action class must have 2 functions as below:

    def __init__(self, input1, input2, ..., inputN): # define the input arguments for API
        # log the inputs here
        self.input1 = input1
        self.input2 = input2
        ...
        self.inputN = inputN

    def applyTo(self, obj, sim): # The inputs for the applyTo() function is FIXED. Always: self, obj, sim
        # obj := Scenic's definition of the object that is executing this action
        # the object's definition is defined in the model.scenic
        
        # This is how the airSim's API is accessed
        client = sim.client # sim.client is defined in simulator.py's line 44 (client = airsim.MultirotorClient())
        
        # In the following, invoke airSim's APIs through the client using the input arg
        # stop the previous action taken
        client.cancelLastTask(vehicle_name=obj.realObjName)  # obj.realObjName is defined in simulator.py's line 137
        
        # execute the actual action using airSim API. for example,...
        client.moveByVelocityAsync(
            self.newVelocity.x_val,
            self.newVelocity.y_val,
            self.newVelocity.z_val,
            duration=5,
            vehicle_name=obj.realObjName,
        )
'''

class SetVelocity(Action):
    def __init__(self, velocity):
        self.newVelocity = scenicToAirsimVector(toVector(velocity))

    def applyTo(self, obj, sim):
        client = sim.client 
        client.cancelLastTask(vehicle_name=obj.realObjName)
        client.moveByVelocityAsync(
            self.newVelocity.x_val,
            self.newVelocity.y_val,
            self.newVelocity.z_val,
            duration=5,
            vehicle_name=obj.realObjName,
        )

class MoveToPositionAsync(Action):
    def __init__(self, position, speed):
        self.position = scenicToAirsimVector(toVector(position))
        self.speed = speed

    def applyTo(self, obj, sim):
        client = sim.client 
        client.cancelLastTask(vehicle_name=obj.realObjName)
        client.moveToPositionAsync(
                self.position.x_val,
                self.position.y_val,
                self.position.z_val,
                velocity=self.speed,
                vehicle_name=obj.realObjName,
            )
