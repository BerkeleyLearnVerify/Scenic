param map = localPath('../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.domains.driving.model

ego = new Car

rightCurb = ego.laneGroup.curb
spot = new OrientedPoint on visible rightCurb
badAngle = Uniform(1.0, -1.0) * Range(10, 20) deg
parkedCar = new Car left of spot by 0.5,
                facing badAngle relative to roadDirection