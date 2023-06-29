"""
Generate a simple compositional with ad-hoc Webots objects
"""

model scenic.simulators.webots.model

air_region = RectangularRegion((0,0,1), 0, 8, 8)

class Floor(WebotsObject):
    width: 10
    length: 10
    height: 0.5
    baseOffset: (0,0,self.height/2)
    webotsAdhoc: {'physics': False}

class Ball(WebotsObject):
    shape: SpheroidShape()
    webotsAdhoc: {'physics': True}

scenario ItemRain():
    setup:
        new Ball in air_region

scenario Main():
    setup:
        floor = new Floor

    compose:
        while True:
            do ItemRain() for 10 seconds
