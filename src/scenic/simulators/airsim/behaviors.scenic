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

    distance = 0
    while True:
        print("started loop")
        curPos = client.simGetVehiclePose(self.realObjName).position + scenicToAirsimVector(toVector(self._startPos))
        print(curPos.y_val)
        curToNew = newPos - curPos
        distance = curToNew.get_length()
        print("dis = ",distance)
        if distance <= tolerance:
            print("less")
            break

        curToNew = (curToNew / distance) * min(speed * distance, speed)
        
        do waitForPromise(createPromise(
                client.moveByVelocityAsync(
                    curToNew.x_val,
                    curToNew.y_val,
                    curToNew.z_val,
                    duration=simulation().simulator.timestep,
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


# ? how to run
behavior FlyToStart():
    print(self._startPos)
    do FlyToPosition(self._startPos)

    


behavior Follow(obj,speed = 5, interval = 5):
    client = simulation().client
    while True:
        position = scenicToAirsimLocationVector(obj.position)
        print(obj.position)
        # client.moveToPositionAsync(
        #     position.x,position.y,position.z,
        #     speed,
        #     vehicle_name=self.realObjName,
        # )
        do FlyToPosition(obj.position)
        # for i in range(interval):
        #     wait
