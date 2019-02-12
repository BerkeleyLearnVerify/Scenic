from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.gta_model import *

wiggle = (-10 deg, 10 deg)

ego = EgoCar with roadDeviation wiggle
Car offset by (-5, 5) @ (7, 12), with roadDeviation resample(wiggle)