param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town03.xodr')
model scenic.simulators.carla.model

# Cars on adjacent lanes of the first section of Road 69
roadSec = network.elements['road69'].sections[0]
ego = Car in roadSec.forwardLanes[0],    # rightmost lane
    with color Color(1, 0, 0)
c1 = Car in roadSec.forwardLanes[1],     # next lane to left
    with color Color(0, 1, 0)
c2 = Car in roadSec.backwardLanes[0],    # rightmost lane on other side of road
    with color Color(0, 0, 1)
c3 = Car in roadSec.backwardLanes[1],    # next lane to left
    with color Color(0, 0, 0)
