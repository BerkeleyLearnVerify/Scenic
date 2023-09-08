import matplotlib.pyplot as plt
import pytest

from tests.utils import compileScenic


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("crossing.scenic", mode2D=True)
    scenario.generate(maxIterations=1000)


def test_road_only(loadLocalScenario):
    scenario = loadLocalScenario("simple.scenic", mode2D=True)
    scenario.generate(maxIterations=100)


@pytest.mark.graphical
def test_show2D(loadLocalScenario):
    scenario = loadLocalScenario("crossing.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=1000)
    scene.show2D(block=False)
    plt.close()


def test_curb(loadLocalScenario):
    scenario = loadLocalScenario("curb.scenic", mode2D=True)
    scenario.generate(maxIterations=200)


@pytest.mark.slow
def test_noninterference(runLocally):
    code = (
        "import scenic.simulators.webots.road.world as world\n"
        "world.worldPath = '{world}'\n"
        "from scenic.simulators.webots.road.model import *\n"
        "ego = new Pedestrian"
    )
    with runLocally():
        scenario = compileScenic(code.format(world="richmond.wbt"), mode2D=True)
        assert len(scenario.objects) == 1
        roads1 = scenario.workspace.roads
        assert len(roads1) > 1
        scenario = compileScenic(code.format(world="simple.wbt"), mode2D=True)
        assert len(scenario.objects) == 1
        roads2 = scenario.workspace.roads
        assert len(roads2) == 1
        scenario = compileScenic(code.format(world="richmond.wbt"), mode2D=True)
        assert len(scenario.objects) == 1
        roads3 = scenario.workspace.roads
        assert len(roads1) == len(roads3)
