
import pytest

import scenic
from scenic.core.errors import InvalidScenarioError
from tests.utils import compileScenic, sampleScene

def test_everywhere():
    scenario = compileScenic('ego = Object with regionContainedIn everywhere')
    sampleScene(scenario, maxIterations=1)

def test_nowhere():
    with pytest.raises(InvalidScenarioError):
        compileScenic('ego = Object with regionContainedIn nowhere')

def test_polygonal_empty_intersection():
    scenario = compileScenic(
        'r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])\n'
        'ego = Object at -10@0, facing Range(-90, 0) deg, with viewAngle 60 deg\n'
        'Object in visible r1, with requireVisible False'
    )
    for i in range(30):
        sampleScene(scenario, maxIterations=1000)
