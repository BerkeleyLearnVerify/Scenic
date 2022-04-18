
model scenic.simulators.webots.model

workspace = Workspace(RectangularRegion((0, 0), 0, 1, 1))

class Box(WebotsObject):
    webotsType: 'box'
    heading: Range(0, 360) deg
    width: 0.1
    length: 0.1

behavior Push():
    while True:
        take ApplyForceAction((0, 2), relative=True)

ego = Box with behavior Push

Box in workspace
Box in workspace

terminate after 5
