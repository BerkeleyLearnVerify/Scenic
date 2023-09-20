from scenic.core.type_support import toVector
from scenic.simulators.airsim.demoBehaviors import *

from .conversionUtils import (
    scenicToAirsimVector,
    scenicToAirsimOrientation,
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimScale,
)

def myAbs(num):
    if num < 0:
        return -num
    return num

def airsimtoVector(airsimVec):
    return toVector((airsimVec.x_val,airsimVec.y_val,airsimVec.z_val))

def getRealDirection(client, self, direction, collisionDistanceMin,levels = 0):
        # direction = desired direction
        
        if levels >=3:
            # tried all turning but doesn't work so don't move at all
            return airsim.Vector3r(0,0,0)
        
        frontDistance = client.getDistanceSensorData(vehicle_name=self.realObjName,distance_sensor_name="frontDistance").distance
        rightDistance = client.getDistanceSensorData(vehicle_name=self.realObjName,distance_sensor_name="rightDistance").distance
        leftDistance = client.getDistanceSensorData(vehicle_name=self.realObjName,distance_sensor_name="leftDistance").distance

        # print(frontDistance,rightDistance,leftDistance)
        distance = None
        if myAbs(direction.x_val) > myAbs(direction.y_val):
            print("front")
            direction = airsim.Vector3r(direction.x_val,0,0)
            distance = frontDistance
        elif direction.y_val > 0:
            print("right")
            distance = rightDistance
            direction = airsim.Vector3r(0,direction.y_val,0)
        else:
            print("left")
            distance = leftDistance
            direction = airsim.Vector3r(0,direction.y_val,0)
        
        if distance < collisionDistanceMin:
            # rotate direction by 90 deg
            return getRealDirection(client,self,airsim.Vector3r(direction.y_val,-direction.x_val,direction.z_val), collisionDistanceMin,levels+1)
        print("distance")
        return direction


behavior navigateTo(pos, posTolerance = 1, collisionDistanceMin = 5,speed=5):
    client = simulation().client

    newPos = scenicToAirsimVector(toVector(pos))

    
    while True:
        
        curPos = client.simGetVehiclePose(self.realObjName).position + scenicToAirsimVector(toVector(self._startPos))
        curToNew = newPos - curPos
        distanceToNew = curToNew.get_length()
        
        if distanceToNew <= posTolerance:
            break

        # print(airsimtoVector(curToNew/))
        velocity = (curToNew / distanceToNew) * min(speed * distanceToNew, speed)
        print("next")
        velocity = getRealDirection(client,self, velocity,collisionDistanceMin)
        # print(velocity)
        
        do waitForPromise(createPromise(
            client.moveByVelocityAsync(
                velocity.x_val,
                velocity.y_val,
                velocity.z_val,
                duration=simulation().simulator.timestep,
                vehicle_name=self.realObjName,
            )
        ))
       
        
        
    return
    

