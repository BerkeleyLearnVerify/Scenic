from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, 'OpenDrive/Town01.xodr')
from scenic.simulators.carla.road_model import *

ego = Car 
c2 = Car at ego offset by (-5, 5) @ (0, 10)

require abs(relative heading of c2 from ego) >= 80 deg
require abs(relative heading of c2 from ego) <= 110 deg

