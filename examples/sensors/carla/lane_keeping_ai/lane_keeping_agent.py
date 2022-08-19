import copy

import numpy as np

from lka.lane_detector import LaneDetector
from lka.camera_geometry import CameraGeometry
from lka.pure_pursuit import PurePursuitPlusPID
from scenic.core.simulators import Action
import torch


def carla_to_np_rgb(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))  # RGBA format
    array = array[:, :, :3]  # Take only RGB
    array = array[:, :, ::-1]  # Revert order
    return array


class LaneKeepingAgent(Action):
    def __init__(self, model_path, camera_attrs, controller_attrs, device="cpu", camera="lane_camera"):
        self.model = torch.load(model_path, map_location=device)
        self.model.eval()
        self.camera_attributes = camera_attrs
        camera_geometry = CameraGeometry(offset=camera_attrs["offset"],
                                         rotation=camera_attrs["rotation"],
                                         image_width=camera_attrs["image_size_x"],
                                         image_height=camera_attrs["image_size_y"],
                                         field_of_view_deg=camera_attrs["fov"]
                                         )
        self.lane_detector = LaneDetector(self.model, cam_geometry=camera_geometry, device=device)
        self.controller = PurePursuitPlusPID(controller_attrs)
        self.lane_camera = camera

    def applyTo(self, agent, simulation):
        image = carla_to_np_rgb(agent.observations[self.lane_camera])

        traj = self.lane_detector.get_center_lane_trajectory(image)
        agent.status.update(self.lane_detector.status)
        throttle, steer = self.controller.get_control(traj, agent.speed, 20, simulation.timestep)

        agent.setThrottle(throttle)
        agent.setSteering(steer)
