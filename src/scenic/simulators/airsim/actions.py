from scenic.core.simulators import Action
from scenic.core.type_support import toVector

from .utils import (
    scenicToAirsimVector,
    scenicToAirsimOrientation,
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimScale,
)

class SetVelocity(Action):
    def __init__(self, velocity):
        self.newVelocity = scenicToAirsimVector(toVector(velocity))
        
    def applyTo(self,obj,sim):
        client = sim.client
        client.cancelLastTask(vehicle_name=obj.realObjName)
        client.moveByVelocityAsync(
            self.newVelocity.x_val,
            self.newVelocity.y_val,
            self.newVelocity.z_val,
            duration=5,
            vehicle_name=obj.realObjName,
        )
    