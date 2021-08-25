param map = localPath('../../tests/formats/opendrive/maps/LGSVL/cubetown.xodr')
param lgsvl_map = 'CubeTown'
param time_step = 1.0/10

model scenic.domains.driving.model
from scenic.simulators.lgsvl.model import Car

ego = Car
Car