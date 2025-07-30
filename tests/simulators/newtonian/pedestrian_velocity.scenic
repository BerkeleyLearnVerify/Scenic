param render = False
param map = localPath('../../../assets/maps/CARLA/Town01.xodr')
model scenic.simulators.newtonian.driving_model

ped = new Pedestrian on sidewalk, with velocity (1, 1)

record initial ped.position as InitialPos
record final ped.position as FinalPos
terminate after 8 steps
