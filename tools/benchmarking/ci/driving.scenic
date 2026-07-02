
from utils import getAssetPath

param map = getAssetPath("maps/CARLA/Town01.xodr")
param use2DMap = True
param map_options = dict(useCache=False, writeCache=False)
model scenic.domains.driving.model

selected_road = Uniform(*network.roads)
selected_lane = Uniform(*selected_road.lanes)
ego = new Car on selected_lane.centerline

new Pedestrian on visible sidewalk

rightCurb = ego.laneGroup.curb
spot = new OrientedPoint on visible rightCurb
badAngle = Uniform(1.0, -1.0) * Range(10, 20) deg
parkedCar = new Car left of spot by 0.5,
                facing badAngle relative to roadDirection
