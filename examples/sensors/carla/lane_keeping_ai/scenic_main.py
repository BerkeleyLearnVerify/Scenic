import copy
import time
from lane_keeping_agent import carla_to_np_rgb
from scenic.syntax.translator import scenarioFromFile

import cv2


def main():
    scenario = scenarioFromFile("nn_action.scenic", params={"render": 0})
    simulator = scenario.getSimulator()

    scene, _ = scenario.generate()
    simulation = simulator.createSimulation(scene)
    now = time.time()
    simulation.run(250)
    print(time.time() - now)
    result = simulation.result

    lane_camera = result.records['lane_camera']
    status = result.records['status']

    video = cv2.VideoWriter('lane_keeping.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20, (1024, 512))
    for camera, status in zip(lane_camera, status):
        image = copy.deepcopy(carla_to_np_rgb(camera[1]))
        left_mask = status[1]["left_lane_seg"] > 0.5
        right_mask = status[1]["right_lane_seg"] > 0.5
        image[left_mask] = [0, 0, 255]
        image[right_mask] = [255, 0, 0]
        image = image[:, :, ::-1]
        video.write(image)


if __name__ == "__main__":
    main()
