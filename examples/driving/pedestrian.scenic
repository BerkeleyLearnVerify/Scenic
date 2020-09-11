
param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
model scenic.domains.driving.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)
ego = Car on select_lane.centerline

right_sidewalk = network.laneGroupAt(ego)._sidewalk

Pedestrian on visible right_sidewalk
