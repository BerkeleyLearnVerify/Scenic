
import sys

from scenic import scenarioFromString as compileScenic
from scenic.simulators.simulators import Simulator
import scenic.syntax.veneer as veneer

## Scene generation utilities

# Static scenes

def sampleScene(scenario, maxIterations=1):
    return generateChecked(scenario, maxIterations)[0]

def sampleEgo(scenario, maxIterations=1):
    scene, iterations = generateChecked(scenario, maxIterations)
    return scene.egoObject

def sampleEgoFrom(code, maxIterations=1):
    scenario = compileScenic(code)
    return sampleEgo(scenario, maxIterations=maxIterations)

def sampleParamP(scenario, maxIterations=1):
    scene, iterations = generateChecked(scenario, maxIterations)
    return scene.params['p']

def sampleParamPFrom(code, maxIterations=1):
    scenario = compileScenic(code)
    return sampleParamP(scenario, maxIterations=maxIterations)

# Dynamic simulations

def sampleEgoActions(scenario, maxIterations=1, maxSteps=1):
    scene, iterations = generateChecked(scenario, maxIterations)
    return sampleEgoActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps)

def sampleEgoActionsFromScene(scene, maxIterations=1, maxSteps=1):
    allActions = sampleActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps)
    return [actions[0] for actions in allActions]

def sampleActions(scenario, maxIterations=1, maxSteps=1):
    scene, iterations = generateChecked(scenario, maxIterations)
    return sampleActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps)

def sampleActionsFromScene(scene, maxIterations=1, maxSteps=1):
    sim = Simulator()
    traj = sim.simulate(scene, maxSteps=maxSteps, maxIterations=maxIterations)
    return traj[1:]

# Helpers

def generateChecked(scenario, maxIterations):
    checkVeneerIsInactive()
    scene, iterations = scenario.generate(maxIterations=maxIterations)
    checkVeneerIsInactive()
    return scene, iterations

def checkVeneerIsInactive():
    assert veneer.activity == 0
    assert not veneer.evaluatingRequirement
    assert len(veneer.allObjects) == 0
    assert veneer.egoObject is None
    assert len(veneer.globalParameters) == 0
    assert len(veneer.externalParameters) == 0
    assert len(veneer.pendingRequirements) == 0
    assert len(veneer.inheritedReqs) == 0
    assert len(veneer.behaviors) == 0
    assert len(veneer.monitors) == 0

## Error checking utilities

def checkErrorLineNumber(line, exc_info=None):
    if exc_info is None:
        tb = sys.exc_info()[2]
    else:
        tb = exc_info.tb
    while tb.tb_next is not None:
        tb = tb.tb_next
    assert tb.tb_lineno == line
