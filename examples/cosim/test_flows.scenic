# param startTime = 0
param map = localPath('../../assets/maps/CARLA/Town05.xodr') # OpenDrive file
param xml_map = localPath("../../assets/maps/CARLA/Town05.net.xml") # Sumo file
param address = "10.139.168.114"
# param address = "10.0.0.122"
# param verbose = True
model scenic.simulators.cosim.model

scenario CustomCommuterTrafficStream(origin, destination):
    setup:
        num_commuters = Range(100, 200)
        morning_peak_time = 1*60*60 # Normal(9*60*60, 30*60)
        evening_peak_time = 2*60*60 # Normal(17*60*60, 30*60)
        traffic_stddev = 15*60 # Normal(1*60*60, 10*60)

    compose:
        do CommuterTrafficStream(origin, destination, num_commuters,
                    morning_peak_time, evening_peak_time, traffic_stddev)

scenario Main():
    setup:
        ego = new EgoCar with name "ego", with behavior DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12)
    compose:
        ts_2_21  = CustomCommuterTrafficStream(2, 21)
        ts_3_21  = CustomCommuterTrafficStream(3, 21)
        ts_4_21  = CustomCommuterTrafficStream(4, 21)
        ts_7_21  = CustomCommuterTrafficStream(7, 21)
        ts_11_21 = CustomCommuterTrafficStream(11, 21)

        do ts_2_21, ts_3_21, ts_4_21, ts_7_21, ts_11_21 for 3*60*60 seconds # 16*60*60 seconds
