
from scenic.simulators.lgsvl.map import loadLocalNetwork
loadLocalNetwork(__file__, 'maps/borregasave.xodr')
from scenic.simulators.lgsvl.model import *
from scenic.simulators.lgsvl.behaviors import DriveTo, FollowWaypoints

simulator = LGSVLSimulator('BorregasAve')
param time_step = 1.0/10.0

ego = EgoCar in intersection

maneuver = Uniform(*ego.intersection.maneuversAt(ego))

Car on maneuver.connectingLane.centerline
