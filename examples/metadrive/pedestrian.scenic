param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)
ego = new Car on select_lane.centerline, with behavior FollowLaneBehavior

right_sidewalk = network.laneGroupAt(ego)._sidewalk

new Pedestrian on visible right_sidewalk
