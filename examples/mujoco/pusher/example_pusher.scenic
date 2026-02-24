from pusher import Pusher
from scenic.simulators.mujoco.simulator import MujocoSimulator

simulator MujocoSimulator(base_xml_file="pusher_base.xml", use_viewer=True)

# Load pusher XML content
# pusher_xml = open("pusher.xml").read()

# Create pusher using the XML's default positioning
# The pusher.xml already has pos="0 -0.6 0" for the shoulder
# ego = new Pusher at (0, 0, 0),  # Let XML handle positioning
#     with xml_content pusher_xml,
#     with sb3_model "PPO_pusher.zip"

ego = new Pusher at (0, 0, 0)