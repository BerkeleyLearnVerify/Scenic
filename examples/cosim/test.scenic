# param startTime = 0
param map = localPath('../../assets/maps/CARLA/Town05.xodr') # OpenDrive file
param xml_map = localPath("../../assets/maps/CARLA/Town05.net.xml") # Sumo file
param address = "10.29.10.114"
# param address = "10.0.0.122"
# param verbose = True
model scenic.simulators.cosim.model

# with behavior DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12),
ego = new EgoCar with name "ego", with behavior DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12)
#, with position Vector(88.80499658384105, 1.5729413763422928, 0)


for i in range(20):
    title = f"npccar_{i}" # allow me to debug more easily
    new NPCCar with name title
    #, with position Vector(86.43, 1.5500000000000114, 0)

terminate after 500 steps