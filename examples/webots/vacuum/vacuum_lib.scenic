"""
Library for generating robot vacuum scenarios.
"""

model scenic.simulators.webots.model

import numpy as np
import trimesh
import random
from pathlib import Path

param numToys = 0
param duration = 10

## Class Definitions ##

class Vacuum(WebotsObject):
    webotsName: "IROBOT_CREATE"
    shape: CylinderShape()
    width: 0.335
    length: 0.335
    height: 0.07
    customData: str(DiscreteRange(0, 2**32 - 1)) # Random seed for robot controller

# Floor uses builtin Webots floor to keep Vacuum Sensors from breaking
# Not actually linked to WebotsObject because Webots floor is 2D
class Floor(Object):
    width: 5
    length: 5
    height: 0.01
    position: (0,0,-0.005)
    color: [200, 200, 200]

class Wall(WebotsObject):
    webotsAdhoc: {'physics': False}
    width: 5
    length: 0.04
    height: 0.5
    color: [160, 160, 160]

class DiningTable(WebotsObject):
    webotsAdhoc: {'physics': True}
    shape: MeshShape.fromFile(Path(localPath(".")).parent.parent.parent / "tools" / "meshes" / "dining_table.obj.bz2")
    width: Range(0.7, 1.5)
    length: Range(0.7, 1.5)
    height: 0.75
    density: 670 # Density of solid birch
    color: [103, 71, 54]

class DiningChair(WebotsObject):
    webotsAdhoc: {'physics': True}
    shape: MeshShape.fromFile(Path(localPath(".")).parent.parent.parent / "tools" / "meshes" / "dining_chair.obj.bz2", initial_rotation=(180 deg, 0, 0))
    width: 0.4
    length: 0.4
    height: 1
    density: 670 # Density of solid birch
    positionStdDev: (0.05, 0.05 ,0)
    orientationStdDev: (10 deg, 0, 0)
    color: [103, 71, 54]

class Couch(WebotsObject):
    webotsAdhoc: {'physics': False}
    shape: MeshShape.fromFile(Path(localPath(".")).parent.parent.parent / "tools" / "meshes" / "couch.obj.bz2", initial_rotation=(-90 deg, 0, 0))
    width: 2
    length: 0.75
    height: 0.75
    positionStdDev: (0.05, 0.5 ,0)
    orientationStdDev: (5 deg, 0, 0)
    color: [51, 51, 255]

class CoffeeTable(WebotsObject):
    webotsAdhoc: {'physics': False}
    shape: MeshShape.fromFile(Path(localPath(".")).parent.parent.parent / "tools" / "meshes" / "coffee_table.obj.bz2")
    width: 1.5
    length: 0.5
    height: 0.4
    positionStdDev: (0.05, 0.05 ,0)
    orientationStdDev: (5 deg, 0, 0)
    color: [103, 71, 54]

class Toy(WebotsObject):
    webotsAdhoc: {'physics': True}
    shape: Uniform(BoxShape(), CylinderShape(), ConeShape(), SpheroidShape())
    width: 0.1
    length: 0.1
    height: 0.1
    density: 100
    color: [255, 128, 0]

class BlockToy(Toy):
    shape: BoxShape()