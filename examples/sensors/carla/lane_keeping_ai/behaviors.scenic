
from lane_keeping_agent import LaneKeepingAgent


behavior LaneKeepingAI(camera_attrs, controller_attrs):
    agent = LaneKeepingAgent(localPath("models/ClearNoon_model.pth"), camera_attrs, controller_attrs)
    while True:
        take agent
