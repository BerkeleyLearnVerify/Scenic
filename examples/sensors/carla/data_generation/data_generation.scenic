param map = localPath('../../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

# Sample a lane at random
lane = Uniform(*network.lanes)

spot = new OrientedPoint on lane.centerline

attrs = {"image_size_x": 1056,
         "image_size_y": 704}

car_model = "vehicle.tesla.model3"

# Spawn car on that spot with logging autopilot behavior and
# - an RGB Camera pointing forward with specific attributes
# - a semantic segmentation sensor
ego = new Car at spot,
    with blueprint car_model,
    with behavior AutopilotBehavior(),
    with sensors {"front_ss": SSSensor(offset=(1.6, 0, 1.7), convert='CityScapesPalette', attributes=attrs),
                  "front_rgb": RGBSensor(offset=(1.6, 0, 1.7), attributes=attrs)
                  }


other = new Car offset by 0 @ Range(10, 30),
    with behavior AutopilotBehavior()

param recordFolder = "out/{simulation}"
record ego.observations["front_ss"] every 0.5 seconds after 5 seconds to "frontss_{time}.jpg"
record ego.observations["front_rgb"] after 5 seconds to "frontrgb.mp4"

terminate after 15 seconds
