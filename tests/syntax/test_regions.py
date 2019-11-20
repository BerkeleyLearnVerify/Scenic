
import pytest

import scenic
from scenic import scenarioFromString as compileScenic
from scenic.core.utils import InvalidScenarioError

def test_everywhere():
    scenario = compileScenic('ego = Object with regionContainedIn everywhere')
    scenario.generate(maxIterations=1)

def test_nowhere():
    with pytest.raises(InvalidScenarioError):
        compileScenic('ego = Object with regionContainedIn nowhere')
