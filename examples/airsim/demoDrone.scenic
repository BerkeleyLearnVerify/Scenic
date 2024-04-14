import math

# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/worldInfo/droneBlocks"

model scenic.simulators.airsim.model

# This demo includes 1 adversarial drone, and 1 drone looking for this adversary.
# Also contains notes on relevant scenic features you might want to use.

# Can define a workspace, by default workspace contains all space
"""
workspace_region = RectangularRegion(Vector(0,0,0), 30, 30, 30)
workspace = Workspace(workspace_region)
"""

def magnitude(v):
    return math.hypot(v.x, v.y, v.z)

# Can extend scenic models to add more parameters to them. Generic models are in model.scenic.
# Scenic models are like templates for the objects to spawn.
class AdversaryDrone(Drone):
    patrolPoints: []
    patrolPointsProb: []

# Find the adversary. drone1 is the adversary target
behavior FindAdversary(positions, speed = 5):
    # typical syntax for behaviors in scenarios are try-interrupt statements. It's essentially a nicer while loop
    # https://scenic-lang.readthedocs.io/en/latest/reference/statements.html#try-interrupt-statement
    try:
        print("POSSIBLE POSITIONS:")
        print(positions)
        while ((distance from self to drone1) >= 1):
            selectedPoint = Discrete({positions[0]:drone1.patrolPointsProb[0], 
            positions[1]:drone1.patrolPointsProb[1], 
            positions[2]:drone1.patrolPointsProb[2], 
            positions[3]:drone1.patrolPointsProb[3]})
            
            print("EGO CHECKING POSITION:")
            print(selectedPoint)

            do FlyToPosition(selectedPoint, speed=speed, tolerance=1,pidMode=True)
            
            # resample point since didn't find adversary at that position
    
    interrupt when (distance from self to drone1) < 20:
        # when I see that I am within 20 meters of adversary, follow it
        print("FOLLOW")
        do Follow(drone1, speed=10, tolerance=1, offset=(0,0,0))
    interrupt when distance from self to drone1 < 5:
        # when I get within 5 meters of adversary, terminate scenario
        print("TERMINATING")
        terminate

# Adversary behavior. Move to randomly chosen position out of points.
behavior AdversaryBehavior(points, speed):
    do Patrol(points, loop=True, speed=speed)

ground = getPrexistingObj("ground")
centerArea = RectangularRegion(Vector(0,200,30), 0, 70, 70)
platforms = []

blockCount = 4
for i in range(blockCount):
    platforms.append(new StaticObj on ground, 
        contained in centerArea,
        with assetName "Cone", # use * to pick a random asset in assets
        with width Range(3,10),
        with length Range(3,10),
        with height 10)

points = []
for plat in platforms:
    point = new Point on plat
    points.append(point.position)

pt = Uniform(*points)

# Adversary drone spawning at random point
drone1 = new AdversaryDrone at pt + (0,0,2),
    with behavior AdversaryBehavior(points, speed=5)

drone1.patrolPointsProb = [0.4, 0.2, 0.1, 0.3] # Probability distribution on the patrolPoints

ego = new Drone at (0,200,12),
    with behavior FindAdversary(points, speed=5)


# took too long to locate so terminate after x seconds

terminate after 30 seconds 


# can require initial scenario conditions here 
""" 
require (distance from ego to drone) > 20
"""
