
from lane_keeping_agent import LaneKeepingAgent


behavior LaneKeepingAI(camera_attrs, controller_attrs, camera, device="cpu"):
    agent = LaneKeepingAgent(localPath("models/ClearNoon_model.pth"),
                             camera_attrs, controller_attrs, camera=camera, device=device)
    while True:
        take agent
