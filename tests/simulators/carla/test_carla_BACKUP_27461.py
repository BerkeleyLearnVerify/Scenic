
import pytest

from scenic import scenarioFromString as compileScenic

<<<<<<< HEAD
pytest.importorskip('Polygon')  # CARLA maps are too big for pypoly2tri
=======
pytest.importorskip('carla')
>>>>>>> carla

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scenario.generate(maxIterations=1000)
