
import sys
import inspect

from scenic import scenarioFromString
from scenic.core.simulators import DummySimulator
import scenic.syntax.veneer as veneer

## Scene generation utilities

# Compilation

def compileScenic(code, removeIndentation=True):
    if removeIndentation:
        # to allow indenting code to line up with test function
        code = inspect.cleandoc(code)
    return scenarioFromString(code)

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

def sampleEgoActions(scenario, maxIterations=1, maxSteps=1, maxScenes=1, singleAction=True):
    allActions = sampleActions(scenario, maxIterations, maxSteps, maxScenes,
                               singleAction, asMapping=False)
    return [actions[0] for actions in allActions]

def sampleEgoActionsFromScene(scene, maxIterations=1, maxSteps=1, singleAction=True):
    allActions = sampleActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps,
                                        singleAction=singleAction, asMapping=False)
    if allActions is None:
        return None
    return [actions[0] for actions in allActions]

def sampleActions(scenario, maxIterations=1, maxSteps=1, maxScenes=1,
                  singleAction=True, asMapping=False):
    for i in range(maxScenes):
        scene, iterations = generateChecked(scenario, maxIterations)
        actions = sampleActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps,
                                         singleAction=singleAction, asMapping=asMapping)
        if actions is not None:
            return actions
    assert False, f'unable to find successful simulation over {maxScenes} scenes'

def sampleActionsFromScene(scene, maxIterations=1, maxSteps=1,
                           singleAction=True, asMapping=False):
    sim = DummySimulator()
    result = sim.simulate(scene, maxSteps=maxSteps, maxIterations=maxIterations)
    if not result:
        return None
    actionSequence = result.actions
    if singleAction:
        for i, allActions in enumerate(actionSequence):
            for agent, actions in allActions.items():
                assert len(actions) <= 1
                allActions[agent] = actions[0] if actions else None
    if asMapping:
        return actionSequence
    else:
        return [tuple(actions.values()) for actions in actionSequence]

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
    assert veneer.currentSimulation is None
    assert veneer.currentBehavior is None

## Error checking utilities

def checkErrorLineNumber(line, exc_info=None):
    if exc_info is None:
        tb = sys.exc_info()[2]
    else:
        tb = exc_info.tb
    while tb.tb_next is not None:
        tb = tb.tb_next
    assert tb.tb_lineno == line
