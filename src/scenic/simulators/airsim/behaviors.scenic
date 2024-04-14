

import airsim
import time
import threading
import sys
import math
from promise import Promise
from scenic.core.type_support import toVector
from .utils import (
    scenicToAirsimVector,
    scenicToAirsimOrientation,
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimScale,
)
from scenic.simulators.airsim.actions import *

def magnitude(v):
    return math.hypot(v.x, v.y, v.z)

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
behavior FlyToPosition(newPos, speed = 5,tolerance = 1,pidMode = True):
    # pidMode is true if we want the drone to slow down as it reaches its destination
    
    client = simulation().client

    if pidMode:
        newPos = scenicToAirsimVector(toVector(newPos))
        # NOTE: in a typical behavior, we would take an action for moveToPositionAsync, BUT with the way the airsim API is set up, we need to use promises
        # I do not know why this one needs a promise and the SetVelocity (which calls moveByVelocityAsync in actions.py) does not
        do waitForPromise(createPromise(
            client.moveToPositionAsync(
                newPos.x_val,
                newPos.y_val,
                newPos.z_val,
                velocity=speed,
                vehicle_name=self.realObjName,
            )
        ))
    else:
        while True:
            direction = newPos -self.position 
            distance = magnitude(direction)
            
            if distance < tolerance:
                break
            direction= (direction/distance)*speed
            take SetVelocity(direction)
            wait
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
    do FlyToPosition(self._startPos)

    
behavior Patrol(positions, loop=True, smooth = False, speed = 5,tolerance = 2):
    while True:
        for pos in positions:
            do FlyToPosition(pos,speed=speed,pidMode= not smooth,tolerance=tolerance)
           
        if not loop:
            return

behavior Follow(target, speed = 5,tolerance = 2, offset = (0,0,1)):
    client = simulation().client

    while True:
        targetPosition = target.position + offset
        
        velocity = targetPosition-self.position
        distance = magnitude(velocity)
        velocity = (velocity / distance) * speed
        if distance > tolerance:
            take SetVelocity(velocity)
        wait