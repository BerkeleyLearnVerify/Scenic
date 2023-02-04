param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

car0 = Car on road
car1 = Car behind car0 by Range(2,5)
car2 = Car ahead of car0 by Range(2,5)
ego = Car left of car0 by Range(2,5)