
import pytest

from tests.utils import compileScenic

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('crossing.scenic')
    scenario.generate(maxIterations=1000)

def test_road_only(loadLocalScenario):
    scenario = loadLocalScenario('simple.scenic')
    scenario.generate(maxIterations=100)

def test_curb(loadLocalScenario):
    scenario = loadLocalScenario('curb.scenic')
    scenario.generate(maxIterations=200)

@pytest.mark.slow
def test_noninterference(runLocally):
    code = (
        "import scenic.simulators.webots.road.world as world\n"
        "world.worldPath = '{world}'\n"
        "from scenic.simulators.webots.road.model import *\n"
        "ego = Pedestrian"
    )
    with runLocally():
        scenario = compileScenic(code.format(world='richmond.wbt'))
        assert len(scenario.objects) == 1
        roads1 = scenario.workspace.roads
        assert len(roads1) > 1
        scenario = compileScenic(code.format(world='simple.wbt'))
        assert len(scenario.objects) == 1
        roads2 = scenario.workspace.roads
        assert len(roads2) == 1
        scenario = compileScenic(code.format(world='richmond.wbt'))
        assert len(scenario.objects) == 1
        roads3 = scenario.workspace.roads
        assert len(roads1) == len(roads3)
