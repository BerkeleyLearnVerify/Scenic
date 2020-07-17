
import os
import glob
import pytest

from tests.utils import compileScenic, sampleScene, sampleEgo

preamble = """
        from scenic.simulators.domains.driving.network import loadNetwork
        loadNetwork('{map}')
        from scenic.simulators.domains.driving.model import *
        {firstLine}
"""

def makePreamble(path='tests/simulators/formats/opendrive/maps/CARLA/Town01.xodr',
                 firstLine=''):
    return preamble.format(map=path, firstLine=firstLine)

def compileDrivingScenario(code):
    return compileScenic(makePreamble() + code)

maps = glob.glob('tests/simulators/formats/opendrive/maps/**/*.xodr')
@pytest.mark.parametrize("path", maps)
def test_opendrive(path):
    scenario = compileScenic(makePreamble(path, firstLine='ego = Car'))
    sampleScene(scenario, maxIterations=1000)

def test_elements_at():
    scenario = compileDrivingScenario("""
        ego = Car
        param element = network.elementAt(ego)
        param road = network.roadAt(ego)
        param lane = network.laneAt(ego)
        param laneSection = network.laneSectionAt(ego)
        param laneGroup = network.laneGroupAt(ego)
        param crossing = network.crossingAt(ego)
        param intersection = network.intersectionAt(ego)
    """)
    scene = sampleScene(scenario, maxIterations=1000)
    ego = scene.egoObject
    for param in ('element', 'road', 'lane', 'laneSection', 'laneGroup', 'crossing',
                  'intersection'):
        assert scene.params[param] is getattr(ego, param), param

def test_intersection():
    scenario = compileDrivingScenario("""
        intersection = Uniform(*network.intersections)
        lane = Uniform(*intersection.incomingLanes)
        maneuver = Uniform(*lane.maneuvers)
        ego = Car on maneuver.connectingLane.centerline
    """)
    for i in range(50):
        ego = sampleEgo(scenario, maxIterations=1000)
        intersection = ego.intersection
        assert intersection is not None
        directions = intersection.nominalDirectionsAt(ego)
        assert any(ego.heading == pytest.approx(direction) for direction in directions)
