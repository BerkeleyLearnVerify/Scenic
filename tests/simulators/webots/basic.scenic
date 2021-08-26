
model scenic.simulators.webots.model

workspace = Workspace(RectangularRegion((0, 0), 0, 100, 100))

class Rover(WebotsObject):
    webotsName: 'Rover'
    battery[dynamic]: (1000, 1000, 0)

class Rock(WebotsObject):
    webotsType: 'Rock'

ego = Rover

Rock in workspace
Rock in workspace
