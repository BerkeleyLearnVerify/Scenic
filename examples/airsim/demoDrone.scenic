import math

# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/worldInfo/droneBlocks"

model scenic.simulators.airsim.model

# This demo includes 1 adversarial drone, and 1 drone looking for this adversary.
# Currently assuming offline path finding.
# Also contains notes on relevant scenic features you might want
# Currently it's just set up with points of interests rather than regions of interests, 
# but you can easily add regions with Regions and sample points
# (https://scenic-lang.readthedocs.io/en/latest/modules/scenic.core.regions.html)

# box_region = BoxRegion(dimensions=(30,30,30), position=(0,0,0))
# points = []
# for i in range(3):
#     points.append(new Point in box_region)

# Can define a workspace, by default workspace contains all space
# workspace_region = RectangularRegion(0 @ 0, 30, 30, 30)
# workspace = Workspace(workspace_region)

def magnitude(v):
    return math.hypot(v.x, v.y, v.z)

# Can add arbritrary static objects in the scene,
# as long as there is a mesh in the worldInfo folder and a corresponding one in unreal.
# I have provided the default primitives and some Blocks assets in examples/airsim/worldInfo/carBlocks/assets

# obstacle = new StaticObj on ground,
#     with assetName "Cone",
#     with width Range(3,10),
#     with length Range(3,10),
#     with height 10

# Can extend scenic models to add more parameters to them. Generic models for are in model.scenic.
# Scenic models are like templates for the objects to spawn.
class AdversaryDrone(Drone):
    patrolPoints: []
    patrolPointsProb: []

# Find the adversary. drone1 is the adversary target
# https://scenic-lang.readthedocs.io/en/latest/reference/visibility.html
behavior FindAdversary(speed = 5):
    # Gets the airsim client to call airsim's apis
    client = simulation().client

    # Randomly select point/region to look at given the weights
    # https://scenic-lang.readthedocs.io/en/latest/reference/distributions.html
    # Discrete({value: weight, … })
    selectedPoint = Discrete({drone1.patrolPoints[0]:drone1.patrolPointsProb[0], 
        drone1.patrolPoints[1]:drone1.patrolPointsProb[1], 
        drone1.patrolPoints[2]:drone1.patrolPointsProb[2], 
        drone1.patrolPoints[3]:drone1.patrolPointsProb[3]})

    # typical syntax for behaviors are try-interrupt statements. It's essentially a nicer while loop
    # https://scenic-lang.readthedocs.io/en/latest/reference/statements.html#try-interrupt-statement
    try:
        # if doing online path solution then do api call for that here, and remove the patrolPoints/patrolPointsProb
        do FlyToPosition(selectedPoint, speed) for 10 seconds
        # resample point since didn't find adversary at that position
        selectedPoint = Discrete({drone1.patrolPoints[0]:drone1.patrolPointsProb[0], 
            drone1.patrolPoints[1]:drone1.patrolPointsProb[1], 
            drone1.patrolPoints[2]:drone1.patrolPointsProb[2], 
            drone1.patrolPoints[3]:drone1.patrolPointsProb[3]})
    interrupt when self can see drone1:
        # when I can see adversary, follow it
        do Follow(drone1)
    interrupt when distance from self to drone1 < 2:
        # when I get within 2 meters of adversary, terminate scenario
        terminate

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
# Note: SetVelocity is an action in actions.py. Actions are 1 time events that a scenic object takes
# Behaviors are continuous events that a scenic object takes, that change depending on conditions met
# Generic behaviors can be found in behaviors.scenic

# Adversary behavior. Continuously patrol to each of these positions.
behavior Adversary(speed):
    client = simulation().client

    do Patrol(self.patrolPoints, loop=True, speed=speed)




ego = new Drone at (Range(-10,10),Range(-10,10),Range(0,10)),
    with behavior FindAdversary(),
    with viewAngles (180 deg, 180 deg), # https://scenic-lang.readthedocs.io/en/latest/reference/visibility.html
    with visibleDistance 10

# Adversary drone moving around various points
drone1 = new AdversaryDrone at (Range(-10,10),Range(-10,10),Range(0,10)),
    with behavior Adversary(speed=2)
drone1.patrolPoints = [(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4)]
drone1.patrolPointsProb = [0.4, 0.2, 0.1, 0.3]
# Using your API, do some sort of probability distribution on the patrolPoints to create patrolPointsProb

# took too long to locate so terminate after x seconds
terminate after 30 seconds 

# can require initial scenario conditions here 
# require (distance from ego to drone) > 20