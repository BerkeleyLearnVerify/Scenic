# param startTime = 0
param map = localPath('../../assets/maps/CARLA/Town05.xodr') # OpenDrive file
param xml_map = localPath("../../assets/maps/CARLA/Town05.net.xml") # Sumo file
# param address = "172.21.116.114"
param address = "10.0.0.122"
# param verbose = True
model scenic.simulators.cosim.model

ego = new EgoCar with name "ego", with behavior DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12)

for i in range(40):
    title = f"npccar_{i}" # allow me to debug more easily
    vehicle = new NPCCar with name title

terminate after 500 steps