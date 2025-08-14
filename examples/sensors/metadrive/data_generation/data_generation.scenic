param map = localPath('../../../../assets/maps/CARLA/Town05.xodr')
model scenic.simulators.metadrive.model

# Sample a lane at random
lane = Uniform(*network.lanes)

spot = new OrientedPoint on lane.centerline

# Spawn car on that spot with follow lane behavior and
# - an RGB Camera pointing forward
# - a semantic segmentation sensor
ego = new Car at spot,
    with behavior FollowLaneBehavior(),
    with sensors {"front_ss": SSSensor(offset=(1.6, 0, 1.7), rotation=(0.0, -5.0, 0.0), width=1056, height=704),
                  "front_rgb": RGBSensor(offset=(1.6, 0, 1.7), rotation=(0.0, -5.0, 0.0), width=1056, height=704)
                  }

other = new Car offset by 0 @ Range(10, 30),
    with behavior FollowLaneBehavior()

param recordFolder = "out/{simulation}"
record ego.observations["front_ss"] every 0.5 seconds after 5 seconds to "frontss_{time}.jpg"
record ego.observations["front_rgb"] after 5 seconds to "frontrgb.mp4"

terminate after 15 seconds
