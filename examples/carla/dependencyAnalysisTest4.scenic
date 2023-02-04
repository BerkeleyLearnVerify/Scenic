param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

spot = OrientedPoint on road
car1 = Car right of spot by 1
car2 = Car left of spot by 1
ego = Truck behind car1 by 2

# this is an example where two objs can have both dependent and jointly dependent relationships
# in such case, within jointly relation, there can be ordering if dependent relation exists