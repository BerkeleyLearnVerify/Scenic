param startTime = 0
param map = "CARLA_TOWN5"
param map = localPath('../../../CARLA/Town05.xodr')
param carla_map = 'Town01'
param address = "10.0.0.122"
# param verbose = True
model scenic.simulators.cosim.model

scenario Test():
    compose:
        while True:
            do GeneratePrivateTrip(-1, -1)

scenario Main():
    compose:
        foo = Test()
        do foo for 500 seconds

