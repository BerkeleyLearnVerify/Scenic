param map = localPath('../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.simulators.carla.model

ego = new Car
atm = new ATM visible
terminate after 1 seconds