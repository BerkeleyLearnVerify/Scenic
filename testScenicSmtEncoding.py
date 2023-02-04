import scenic
import os
from scenic.core.distributions import *
from scenic.core.vectors import Vector
from scenic.core.regions import SectorRegion
import math
import subprocess

ego_visibleDistance = 100
ego_viewAngle = 135 #deg
ego_labelled_position = Vector(0, 0)
ego_labelled_heading = 0 #deg
egoVisibleRegion = SectorRegion(ego_labelled_position, ego_visibleDistance, \
                                math.radians(ego_labelled_heading), math.radians(ego_viewAngle))

def test_pointInOptions_laneSection():
	scenic_script = "./examples/carla/test1.scenic"
	scenario = scenic.scenarioFromFile(scenic_script)
	ego = scenario.egoObject

def test_pointInOptions_roads():
	scenic_script = "./examples/carla/test2.scenic"
	scenario = scenic.scenarioFromFile(scenic_script)
	ego = scenario.egoObject
	
def test_pointInOptions_lanes():
	scenic_script = "./examples/carla/test3.scenic"
	scenario = scenic.scenarioFromFile(scenic_script)
	ego = scenario.egoObject

def test_pointInOptions_roadSections():
	scenic_script = "./examples/carla/test4.scenic"
	scenario = scenic.scenarioFromFile(scenic_script)
	ego = scenario.egoObject

