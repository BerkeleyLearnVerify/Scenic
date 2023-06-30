
model scenic.simulators.webots.model

workspace = Workspace(RectangularRegion((0, 0), 0, 100.01, 100.01))

class Rover(WebotsObject):
    webotsName: 'Rover'
    battery[dynamic]: (1000, 1000, 0)

class Rock(WebotsObject):
    webotsType: 'Rock'

ego = new Rover

new Rock in workspace
new Rock in workspace

hill1 = new Hill in workspace, with width 50, with length 50, with height 1
hill2 = new Hill in workspace, with width 25, with length 25, with height 0.5

new Ground with width 100, with length 100,
    with terrain [hill1, hill2]
