from scenic.core.simulators import Action

import airsim
import time
import threading
from promise import Promise
from scenic.core.type_support import toVector
from .utils import (
    scenicToAirsimVector,
    scenicToAirsimOrientation,
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimScale,
)

# creates a promise from an msgpackrpc future (the futures that are used in airsim)
def createPromise(future):
    def promFunc(resolve, reject):
        
        def joinAsync():
            future.join()
            resolve(True)
        
        def waitAsync():
            while not future._set_flag:
                time.sleep(.01)
            resolve(True)

        if not future._loop:
            threading.Thread(target=joinAsync).start()
        else:
            threading.Thread(target=waitAsync).start()
       
            

    prom = Promise(promFunc)

    return prom

# waits for a promise to be fulfilled
behavior waitForPromise(promise):
    while not promise.is_fulfilled:
        wait

# Flies the drone to a position
behavior FlyToPosition(newPos, speed = 5,tolerance = 1):
    client = simulation().client

    newPos = scenicToAirsimVector(toVector(newPos))

    do waitForPromise(createPromise(
        client.moveToPositionAsync(
            newPos.x_val,
            newPos.y_val,
            newPos.z_val,
            velocity=speed,
            vehicle_name=self.realObjName,
        )
    ))

    return




behavior Patrol(positions, loop=True, smooth = False):
    
    verbosePrint("Drone",self.name,"patrolling",positions)
    
    while True:
        for pos in positions:
            if smooth:
            #     distance = self.position - pos
            #    take MoveByVelocityUntilStopped(distance)
                pass
            else:
                do FlyToPosition(pos)
           
        if not loop:
            return




behavior MoveByVelocity(velocity,seconds):
    client = simulation().client

    newVelocity = scenicToAirsimVector(toVector(velocity))

    do waitForPromise(createPromise(
        client.moveByVelocityAsync(
            newVelocity.x_val,
            newVelocity.y_val,
            newVelocity.z_val,
            duration=seconds,
            vehicle_name=self.realObjName,
        )
    ))


class MoveByVelocityUntilStopped(Action):
    def __init__(self, velocity):
        self.newVelocity = scenicToAirsimVector(toVector(velocity))
        
    def applyTo(self,obj,sim):
        client = sim.client
        client.cancelLastTask(vehicle_name=obj.realObjName)
        client.moveByVelocityAsync(
            self.newVelocity.x_val,
            self.newVelocity.y_val,
            self.newVelocity.z_val,
            duration=1000,
            vehicle_name=obj.realObjName,
        )

# behavior MoveByVelocityUntilStopped(velocity):
#     client = simulation().client

#     client.cancelLastTask(vehicle_name=self.realObjName)

#     newVelocity = scenicToAirsimVector(toVector(velocity))
#     client.moveByVelocityAsync(
#         newVelocity.x_val,
#         newVelocity.y_val,
#         newVelocity.z_val,
#         duration=1000,
#         vehicle_name=self.realObjName,
#     )
#     wait
    

behavior FlyToStart():
    do FlyToPosition(self._startPos)

    
