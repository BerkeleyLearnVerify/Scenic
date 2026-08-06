"""
Create a Webots cube in air and have it drop
"""

model scenic.simulators.webots.model

class Floor(Object):
    width: 5
    length: 5
    height: 0.01
    position: (0,0,0)
    color: [0.785, 0.785, 0.785]

class Block(WebotsObject):
    webotsAdhoc: {'physics': True}
    shape: BoxShape()
    width: 0.2
    length: 0.2
    height: 0.2
    density: 100
    color: [1, 0.502, 0]

floor = new Floor
ego = new Block at (0, 0, 0.5) #above floor by 0.5

terminate when ego.z < 0.1
record (ego.z) as BlockPosition