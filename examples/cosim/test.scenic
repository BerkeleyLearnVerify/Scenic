# param startTime = 0
param map = localPath('../../assets/maps/CARLA/Town05.xodr') # OpenDrive file
param xml_map = localPath("../../assets/maps/CARLA/Town05.net.xml") # Sumo file
param address = "10.45.20.114"
# param address = "10.0.0.122"
# param verbose = True
model scenic.simulators.cosim.model

ego = new EgoCar with behavior DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12), with name "ego", with position Vector(88.80499658384105, 1.5729413763422928, 0)


for i in range(10):
    title = f"npccar_{i}" # allow me to debug more easily
    new NPCCar with name title

terminate after 500 steps