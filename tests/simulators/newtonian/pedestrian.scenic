param render = False
param map = localPath('../../../assets/maps/CARLA/Town01.xodr')
model scenic.simulators.newtonian.driving_model

behavior Walk():
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(1)

ped = new Pedestrian with regionContainedIn None,
    with behavior Walk()

record initial ped.position as InitialPos
record final ped.position as FinalPos
terminate after 8 steps
