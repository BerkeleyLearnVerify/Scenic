
import os

from scenic.simulators.gta.interface import Map

mapFilename = 'map.npz'

if not os.path.exists(mapFilename):
    print('Saved map not found; generating from image... (may take several minutes)')
    m = Map('map.png',
        Ax=1.515151515151500 / 2, Ay=-1.516919486581100 / 2,
        Bx=-700, By=500)
    m.dumpToFile(mapFilename)
    print('map saved to "{}"'.format(mapFilename))
