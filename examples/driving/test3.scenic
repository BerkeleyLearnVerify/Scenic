param map = localPath('../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.simulators.metadrive.model


lg = Uniform(*network.laneGroups)
curb = lg.curb

ego = new Car on curb


terminate after 50 steps
