param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
from .action import NNAction

# Sample a lane at random
lane = Uniform(*network.lanes)

spot = OrientedPoint on lane.centerline

attrs = {"image_size_x": 640,
         "image_size_y": 480}

car_model = "vehicle.tesla.model3"

behavior NNBehavior():
    action = NNAction()
    while True:
        take action


# Spawn car on that spot with logging autopilot behavior and
# - an RGB Camera pointing forward with specific attributes
# - a semantic segmentation sensor
ego = Car at spot,
    with blueprint car_model,
    with behavior NNBehavior(),
    with sensors {"front_rgb": CarlaRGBSensor(offset=(1.6, 0, 1.7), attributes=attrs)}

other = Car offset by 0 @ Range(10, 30),
    with behavior AutopilotBehavior()
