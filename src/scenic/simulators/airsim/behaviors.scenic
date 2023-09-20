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

def createPromise(future):
    def promFunc(resolve, reject):
        
        def joinAsync():
            print("joinAsync")
            future.join()
            resolve(True)
        
        def waitAsync():
            # print("waitAsync")
            while not future._set_flag:
                time.sleep(.01)
            resolve(True)

        if not future._loop:
            threading.Thread(target=joinAsync).start()
        else:
            threading.Thread(target=waitAsync).start()
       
            

    prom = Promise(promFunc)

    return prom

behavior waitForPromise(promise):
    while not promise.is_fulfilled:
        # print("waiting")
        wait


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




behavior Patrol(positions, loop=True):
    
    while True:
        for pos in positions:
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


behavior FlyToStart():
    print(self._startPos)
    do FlyToPosition(self._startPos)

    
