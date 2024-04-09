"""
Create a Webots cube in air and have it drop
"""

model scenic.simulators.webots.model

class AdhocBox(WebotsObject):
  webotsAdhoc: {}

ego = new AdhocBox at (0, 0, 10)
workspace = Workspace(RectangularRegion((0, 0, 0), 0, 10, 10))