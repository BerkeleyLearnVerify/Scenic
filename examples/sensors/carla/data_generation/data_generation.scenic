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

record ego.observations["front_rgb"] as "front_rgb"
#record ego.observations["front_rgb"] as "front_rgb" after 5s every 1s # TODO Implement
#record ego.observations["front_rgb"] after 5s every 1s to localPath("data/generation") # TODO Implement

require monitor RecordingMonitor(ego, path=localPath("data"), recording_start=5, subsample=5)
