
import pytest

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.sc')
    scenario.generate(maxIterations=1000)

def test_bumper_to_bumper(loadLocalScenario):
    scenario = loadLocalScenario('bumperToBumper.sc')
    scenario.generate(maxIterations=1000)
