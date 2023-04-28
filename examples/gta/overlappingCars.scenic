from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

wiggle = Range(-10 deg, 10 deg)

ego = new EgoCar with roadDeviation wiggle

c = new Car visible, with roadDeviation resample(wiggle)

leftRight = Options([1.0, -1.0]) * Range(1.25, 2.75)

new Car beyond c by leftRight @ Range(4, 10),
    with roadDeviation resample(wiggle)