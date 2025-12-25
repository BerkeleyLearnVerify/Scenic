param map = localPath('../../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../../assets/maps/CARLA/Town01.net.xml')

model scenic.simulators.metadrive.model

ego = new Car in intersection

ego = new Car on ego.lane.predecessor

new Pedestrian on visible sidewalk

third = new Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
