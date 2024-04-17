import math

# NOTE: add your world info path here
param worldInfoPath = "examples/airsim/worldInfo/nh"

model scenic.simulators.airsim.model

# This demo includes 1 adversarial drone, and 1 drone (ego) looking for this adversary.

class AdversaryDrone(Drone): # Drone class defined in model.scenic
    patrolPoints: []
    patrolPointsProb: []

# Find the adversary. drone1 is the adversary target
behavior FindAdversary(positions, speed = 5):
    # try-interrupt statements: https://scenic-lang.readthedocs.io/en/latest/reference/statements.html#try-interrupt-statement
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

            do FlyToPosition(selectedPoint, speed=speed, tolerance=1,pidMode=True) # FlyToPosition behavior defined in behaviors.scenic
    
    interrupt when (distance from self to drone1) < 15:
        # when I see that I am within 15 meters of adversary, follow it
        print("FOLLOWING ADVERSARY")
        do Follow(drone1, speed=10, tolerance=1, offset=(0,0,0)) # Follow behavior defined in behaviors.scenic

    interrupt when distance from self to drone1 < 7:
        # when I get within 7 meters of adversary, terminate scenario
        print("ADVERSARY FOUND")
        terminate

# Adversary behavior. Patrol through given points.
behavior AdversaryBehavior(points, speed):
    do Patrol(points, loop=True, speed=speed) # Patrol behavior defined in behaviors.scenic

ground = getPrexistingObj("nh")
area1 = RectangularRegion(Vector(0,0,0), 0, 10, 70)
area2 = RectangularRegion(Vector(0,0,0), 0, 70, 10)

workspaceArea = area1.union(area2)

platforms = []
blockCount = 4
for i in range(blockCount):
    platforms.append(new StaticObj on ground, 
        contained in workspaceArea,
        with assetName "Cone", # use * to pick a random asset in assets
        with parentOrientation 0,
        with width 5,
        with length 5,
        with height 10)

points = []
for plat in platforms:
    point = new Point on plat
    points.append(point.position)

adversarySpawn = Options(points)

# Adversary drone spawning at random point
drone1 = new AdversaryDrone at adversarySpawn + (0,0,2),
    with behavior AdversaryBehavior(points, speed=2)
# drone1.patrolPoints = possiblePoints
drone1.patrolPointsProb = [0.4, 0.2, 0.1, 0.3] # Probability distribution on the patrolPoints

# ego drone spawning somwhere in the middle of the workspace area
ego = new Drone at (Range(-5, 5),Range(-5, 5),10),
    with behavior FindAdversary(points, speed=5)



# took too long to locate so terminate after x seconds
terminate after 15 seconds 

record final (distance to drone1) as dist