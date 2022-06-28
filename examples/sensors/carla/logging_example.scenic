param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

# Sample a lane at random
lane = Uniform(*network.lanes)

spot = OrientedPoint on lane.centerline

attrs = {"image_size_x": 1024,
         "image_size_y": 576}

path = localPath("data/logging_example")

# Spawn car on that spot with logging autopilot behavior and
# - an RGB Camera pointing forward with specific attributes
# - an RGB Camera pointing backward
# - a semantic segmentation sensor with CityScapesPalette Conversion
ego = Car at spot,
    with behavior AutopilotBehavior(),
    with sensors {"front_camera": CarlaRGBSensor(offset=(1.6, 0, 1.7), attributes=attrs),
                  "back_camera": CarlaRGBSensor(offset=(-1.6, 0, 1.7), rotation=(0, 180, 0), attributes=attrs),
                  "semantic_segmentation": CarlaSSSensor(offset=(1.6, 0, 1.7), attributes=attrs, convert="CityScapesPalette")},
    with record_sensors path

other = Car offset by 0 @ Range(10, 30),
    with behavior AutopilotBehavior()