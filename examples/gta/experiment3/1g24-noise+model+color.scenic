from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param time = 12 * 60
param weather = 'EXTRASUNNY'

ego = new EgoCar at -628.78787878787944 @ -540.60676779463461,
    facing -359.16913666080427 deg

c = new Car at -625.4444493298472 @ -530.76549003839568,
    facing 8.287256822061408 deg

mutate