import math

# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

# param timestep = .1

# drone1 patrols to a single location after spawning. ego spawns somehwere in the center area and tries to catch it before time ends
# run the src/scenic/simulators/airsim/verifai_sampler.py file to test with verifai. 

model scenic.simulators.airsim.model


behavior Catch(target, speed = 5,tolerance = 1, offset = (0,0,0)):
    try:
        while True:
            targetPosition = target.position + offset
            
            velocity = targetPosition-self.position
            distance = magnitude(velocity)
            velocity = (velocity / distance) * speed
            if distance > tolerance:
                take SetVelocity(velocity)
            wait
    interrupt when (distance from self to drone1) < 3:
        print("Caught drone1!")
        terminate
    

centerArea = RectangularRegion(Vector(0,0,0), 0, 30, 30)

points = [(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)]

pos = VerifaiOptions(points)


drone1 = new Drone at (0,0,0),
    with behavior Patrol([pos],True)

ego = new Drone in centerArea,
    with behavior Catch(drone1)

# NOTE: verifai errors: 'float' object is not iterable when -
# ego uses range and drone1 uses relative position operator or range
# AND when ego uses absolute position and drone1 uses range
# AND when using custom meshes?

terminate after 5 seconds