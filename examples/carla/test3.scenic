param map = localPath('../../assets/maps/CARLA/Town10HD_Opt.xodr')
model scenic.simulators.carla.model
param carla_map = "Town10HD_Opt"
param time_step = 1.0/10

behavior CrossStreet():
    while True:
        take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(1)

ego = new Car at (-42, -137)

ped = new Pedestrian at (-35, -145.30), with regionContainedIn None,
    with heading ego.heading + 90 deg,
    with behavior CrossStreet()

