
from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import loadLocalNetwork
loadLocalNetwork(__file__, 'maps/Straight2LaneSame.xodr')
from scenic.simulators.lgsvl.model import *

simulator LGSVLSimulator('Straight2LaneSame')
param apolloHDMap = 'Straight2LaneSame'
param time_step = 1.0/2

egoStartPos = OrientedPoint on road
egoDestination = follow roadDirection from egoStartPos for 100
require egoDestination in road

ego = EgoCar at egoStartPos,
             with behavior DriveTo(egoDestination)

npcStartPos = egoStartPos offset by -3.5 @ 0

npcWP0 = Waypoint at npcStartPos, with speed (6, 10)
npcWP1 = Waypoint following roadDirection from npcStartPos for 50, with speed (6, 10)
npcWP2 = Waypoint following roadDirection from egoStartPos for 75, with speed (6, 10)
npcWP3 = Waypoint following roadDirection from egoStartPos for 100, with speed 0
waypoints = [npcWP0, npcWP1, npcWP2, npcWP3]

#for waypoint in waypoints:
#    require waypoint in road

npc = Car at npcStartPos,
          with behavior FollowWaypoints(waypoints),
          with lgsvlName Uniform('Sedan', 'SUV', 'Hatchback', 'Jeep')

# Require NPC to be headed approximately the same way as the ego
require abs(relative heading of npc) <= 20 deg
