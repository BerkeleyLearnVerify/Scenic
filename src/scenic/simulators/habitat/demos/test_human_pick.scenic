import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
from scenic.core.vectors import Vector
import math
import time

spot = new SpotRobot at (-1.5, -6.5, 0)
# can = new MasterChef at (-1.5, -4.5, 0.3)
box = new GelatinBox at (-1.5, -4.5, 0.3)
human = new Female_0 at (-1.5, -5.5, 0), with behavior GoToLookAt(box)
# human = new Female_0 at (-1.5, -5.5, 0)

