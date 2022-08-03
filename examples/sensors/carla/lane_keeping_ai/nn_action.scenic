param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

from behaviors import LaneKeepingAI

param weather = "ClearNoon"
param timestep = 0.05

# Sample a lane at random
lane = Uniform(*network.lanes)

spot = OrientedPoint on lane.centerline

camera_attrs = { "offset": (0.5, 0, 1.3),
                 "rotation": (-5, 0, 0),
                 "image_size_x": 1024,
                 "image_size_y": 512,
                 "fov": 45
                 }

controller_attrs = {"k_p": 2, "k_i": 0, "k_d": 0, "k_dd": 0.4}

car_model = "vehicle.audi.tt"

ego = Car at spot,
    with blueprint car_model,
    with behavior LaneKeepingAI(camera_attrs, controller_attrs),
    with sensors {"lane_camera": CarlaRGBSensor(offset=camera_attrs["offset"],
                                                rotation=camera_attrs["rotation"],
                                                attributes=camera_attrs,
                                                record_npy=False)}

record ego.observations["lane_camera"] as "lane_camera"
record ego.status as "status"

#monitor RecordingMonitor:
#    RECORDING_START = 5
#    SUBSAMPLE = 5
#    # Exclude the first 5 steps because cars are spawned in the air and need to drop down first
#    for _ in range(1, RECORDING_START):
#        wait
#    while True:
#        ego.save_observations(localPath("data/generation"))
#        for _ in range(SUBSAMPLE):
#            wait
#
