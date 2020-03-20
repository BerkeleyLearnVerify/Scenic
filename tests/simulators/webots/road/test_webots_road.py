
from scenic import scenarioFromString as compileScenic

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('crossing.sc')
    scenario.generate(maxIterations=1000)

def test_road_only(loadLocalScenario):
    scenario = loadLocalScenario('simple.sc')
    scenario.generate(maxIterations=100)

def test_curb(loadLocalScenario):
    scenario = loadLocalScenario('curb.sc')
    scenario.generate(maxIterations=200)

def test_noninterference(runLocally):
    code = (
        "import scenic.simulators.webots.road.world as world\n"
        "world.worldPath = 'richmond.wbt'\n"
        "from scenic.simulators.webots.road.model import *\n"
        "ego = Pedestrian"
    )
    with runLocally():
        scenario = compileScenic(code)
        assert len(scenario.objects) == 1
        roads1 = scenario.workspace.roads
        scenario = compileScenic(code)
        assert len(scenario.objects) == 1
        roads2 = scenario.workspace.roads
        assert len(roads1) == len(roads2)
