param startTime = 0
param map = "CARLA_TOWN5"
model scenic.simulators.metsr.model

# TODO: Test distribution as parameter default
# TODO: Ask Zengxiang what benchmark performance is

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
    compose:
        ts_2_21  = CustomCommuterTrafficStream(2, 21)
        ts_3_21  = CustomCommuterTrafficStream(3, 21)
        ts_4_21  = CustomCommuterTrafficStream(4, 21)
        ts_7_21  = CustomCommuterTrafficStream(7, 21)
        ts_11_21 = CustomCommuterTrafficStream(11, 21)

        do ts_2_21, ts_3_21, ts_4_21, ts_7_21, ts_11_21 for 3*60*60 seconds # 16*60*60 seconds
