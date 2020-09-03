
param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town01.xodr')

model scenic.domains.driving.model

ego = Car

Pedestrian on visible ego.oppositeLaneGroup.sidewalk
