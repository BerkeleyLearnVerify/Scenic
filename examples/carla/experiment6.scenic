param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town05.xodr') 
param carla_map = 'Town05'
model scenic.domains.driving.model

ego = Car on road

spot = OrientedPoint on visible intersection


require (distance from ego to intersection) < 5