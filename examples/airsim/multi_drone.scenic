# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/worldInfo/droneBlocks"

# Spawns 3 drones that fly to random positions


model scenic.simulators.airsim.model

# Flies the drone to a position
behavior FlyToPosition(newPos, speed = 5,tolerance = 1,pidMode = True):
    # pidMode is true if we want the drone to slow down as it reaches its destination
    
    client = simulation().client

    

    if pidMode:
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

for i in range(3):
    new Drone at (Range(-10,10),Range(-10,10),Range(0,10)),
        with behavior FlyToPosition((Range(-30,10),Range(-30,10),Range(0,30)))

terminate after 10 seconds