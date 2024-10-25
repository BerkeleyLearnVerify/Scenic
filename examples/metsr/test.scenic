model scenic.simulators.metsr.model

zone_2_center = (-0.0024190, -0.0000165, 0)
zone_9_center = (0.0013876, -0.0000135, 0)


scenario Main():
    compose:
        do ConstantTrafficStream(2,9,60)
