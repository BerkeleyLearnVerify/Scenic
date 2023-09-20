model scenic.simulators.airsim.model

ground = getPrexistingObj("ground")


for i in range(3):
    new Drone at (Range(-10,10),Range(-10,10),Range(0,10)),
        with behavior FlyToPosition((Range(-10,10),Range(-10,10),Range(0,10)))
