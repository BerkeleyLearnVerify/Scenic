"""
Generate a simple scene with ad-hoc Webots objects
"""

model scenic.simulators.webots.model

import trimesh

workspace = Workspace(RectangularRegion((0, 0, 0), 0, 10, 10))

class AdhocBox(WebotsObject):
  webotsAdhoc: {}

class AdhocCone(WebotsObject):
  webotsAdhoc: {}
  shape: MeshShape(trimesh.creation.cone(radius=0.5, height=1))
  density: 500

class AdhocSphere(WebotsObject):
  webotsAdhoc: {
    "physics": False
  }
  shape: MeshShape(trimesh.creation.icosphere(radius=0.5))

ego = new AdhocBox at (0, 0, 0)
cone = new AdhocCone at (0, 2, 0)
sphere = new AdhocSphere at (0, 4, 0)
