from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, localPath('../../../../examples/gta/map.npz'))

from scenic.simulators.gta.model import *

ego = new Car
new Car at ego offset by Range(-5, 5) @ Range(20, 40)