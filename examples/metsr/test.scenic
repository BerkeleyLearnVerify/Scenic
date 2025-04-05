param startTime = 0
param map = "CARLA_TOWN5"
# param verbose = True
model scenic.simulators.metsr.model

scenario Test():
    compose:
        while True:
            do GeneratePrivateTrip(-1, -1)

scenario Main():
    compose:
        foo = Test()
        do foo for 500 seconds
