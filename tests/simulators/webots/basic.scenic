
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

hill1 = Hill in workspace, with width 50, with length 50, with height 1
hill2 = Hill in workspace, with width 25, with length 25, with height 0.5

Ground with width 100, with length 100,
    with terrain [hill1, hill2]
