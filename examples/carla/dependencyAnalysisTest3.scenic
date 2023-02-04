param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

ego = Car on Uniform(*network.lanes)
spot = OrientedPoint on visible curb
Car left of spot by Range(2,5)
Car at (spot offset by (Range(3,5), Range(5,10)))