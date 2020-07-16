
import os
import glob
import pytest

from tests.utils import compileScenic, sampleScene

preamble = """
        from scenic.simulators.domains.driving.network import loadNetwork
        loadNetwork('{map}')
        from scenic.simulators.domains.driving.model import *
        ego = Car
"""

def makePreamble(path='tests/simulators/formats/opendrive/maps/CARLA/Town01.xodr'):
    return preamble.format(map=path)

maps = glob.glob('tests/simulators/formats/opendrive/maps/**/*.xodr')
@pytest.mark.parametrize("path", maps)
def test_opendrive(path):
    scenario = compileScenic(makePreamble(path))
    sampleScene(scenario, maxIterations=1000)

def test_elements_at():
    scenario = compileScenic(makePreamble() + """
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
