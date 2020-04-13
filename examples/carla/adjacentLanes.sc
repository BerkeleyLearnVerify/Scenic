from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, 'OpenDrive/Town03.xodr')
from scenic.simulators.carla.model import *

# Cars on lanes [-2..3] of the first LaneSection of Road 70.
ego = Car in laneSectionDict[70][0][1]
c1 = Car in laneSectionDict[70][0][2]
c2 = Car in laneSectionDict[70][0][3]
c3 = Car in laneSectionDict[70][0][-1]
c4 = Car in laneSectionDict[70][0][-2]
