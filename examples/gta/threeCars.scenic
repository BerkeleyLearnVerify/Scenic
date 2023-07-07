from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

wiggle = Range(-10 deg, 10 deg)

ego = new EgoCar with roadDeviation wiggle
new Car visible, with roadDeviation resample(wiggle)
new Car visible, with roadDeviation resample(wiggle)
new Car visible, with roadDeviation resample(wiggle)