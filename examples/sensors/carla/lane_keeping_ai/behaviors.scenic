
from lane_keeping_agent import LaneKeepingAgent


behavior LaneKeepingAI(camera_attrs, controller_attrs, camera, device="cpu"):
    agent = LaneKeepingAgent(localPath("models/ClearNoon_model.pth"),
                             camera_attrs, controller_attrs, camera=camera, device=device)
    while True:
        image = carla_to_np_rgb(self.observations[camera])
        traj = agent.lane_detector.get_center_lane_trajectory(image)
        agent.status.update(agent.lane_detector.status)
        throttle, steer = agent.controller.get_control(traj, agent.speed, 20, simulation.timestep)
        take SetSteerAction(steer), SetThrottleAction(throttle)
