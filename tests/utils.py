
import sys
import inspect

from scenic import scenarioFromString
from scenic.core.simulators import DummySimulator, RejectSimulationException
import scenic.syntax.veneer as veneer

## Scene generation utilities

# Compilation

def compileScenic(code, removeIndentation=True, scenario=None):
    if removeIndentation:
        # to allow indenting code to line up with test function
        code = inspect.cleandoc(code)
    checkVeneerIsInactive()
    scenario = scenarioFromString(code, scenario=scenario)
    checkVeneerIsInactive()
    return scenario

# Static scenes

def sampleScene(scenario, maxIterations=1):
    return generateChecked(scenario, maxIterations)[0]

def sampleSceneFrom(code, maxIterations=1, scenario=None):
    scenario = compileScenic(code, scenario=scenario)
    return sampleScene(scenario, maxIterations=maxIterations)

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

def sampleEgoActions(scenario, maxIterations=1, maxSteps=1, maxScenes=1,
                     singleAction=True, timestep=1):
    allActions = sampleActions(scenario, maxIterations, maxSteps, maxScenes,
                               singleAction, asMapping=False, timestep=timestep)
    return [actions[0] for actions in allActions]

def sampleEgoActionsFromScene(scene, maxIterations=1, maxSteps=1, singleAction=True, timestep=1):
    allActions = sampleActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps,
                                        singleAction=singleAction, asMapping=False,
                                        timestep=timestep)
    if allActions is None:
        return None
    return [actions[0] for actions in allActions]

def sampleActions(scenario, maxIterations=1, maxSteps=1, maxScenes=1,
                  singleAction=True, asMapping=False, timestep=1):
    for i in range(maxScenes):
        scene, iterations = generateChecked(scenario, maxIterations)
        actions = sampleActionsFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps,
                                         singleAction=singleAction, asMapping=asMapping,
                                         timestep=timestep)
        if actions is not None:
            return actions
    raise RejectSimulationException(
        f'unable to find successful simulation over {maxScenes} scenes')

def sampleActionsFromScene(scene, maxIterations=1, maxSteps=1,
                           singleAction=True, asMapping=False, timestep=1):
    sim = DummySimulator(timestep=timestep)
    simulation = sim.simulate(scene, maxSteps=maxSteps, maxIterations=maxIterations)
    if not simulation:
        return None
    actionSequence = simulation.result.actions
    if singleAction:
        for i, allActions in enumerate(actionSequence):
            for agent, actions in allActions.items():
                assert len(actions) <= 1
                allActions[agent] = actions[0] if actions else None
    if asMapping:
        return actionSequence
    else:
        return [tuple(actions.values()) for actions in actionSequence]

def sampleTrajectory(scenario, maxIterations=1, maxSteps=1, maxScenes=1,
                     raiseGuardViolations=False, timestep=1):
    for i in range(maxScenes):
        scene, iterations = generateChecked(scenario, maxIterations)
        trajectory = sampleTrajectoryFromScene(scene, maxIterations=maxIterations,
                                               maxSteps=maxSteps,
                                               raiseGuardViolations=raiseGuardViolations,
                                               timestep=timestep)
        if trajectory is not None:
            return trajectory
    raise RejectSimulationException(
        f'unable to find successful simulation over {maxScenes} scenes')

def sampleResult(scenario, maxIterations=1, maxSteps=1, maxScenes=1, timestep=1):
    for i in range(maxScenes):
        scene, iterations = generateChecked(scenario, maxIterations)
        result = sampleResultFromScene(scene, maxIterations=maxIterations,
                                       maxSteps=maxSteps, timestep=timestep)
        if result is not None:
            return result
    raise RejectSimulationException(
        f'unable to find successful simulation over {maxScenes} scenes')

def sampleResultOnce(scenario, maxSteps=1, timestep=1):
    scene = sampleScene(scenario)
    sim = DummySimulator(timestep=timestep)
    return sim.simulate(scene, maxSteps=maxSteps, maxIterations=1)

def sampleResultFromScene(scene, maxIterations=1, maxSteps=1, raiseGuardViolations=False,
                          timestep=1):
    sim = DummySimulator(timestep=timestep)
    simulation = sim.simulate(scene, maxSteps=maxSteps, maxIterations=maxIterations,
                              raiseGuardViolations=raiseGuardViolations)
    if not simulation:
        return None
    return simulation.result

def sampleTrajectoryFromScene(scene, maxIterations=1, maxSteps=1, raiseGuardViolations=False,
                              timestep=1):
    result = sampleResultFromScene(scene, maxIterations=maxIterations, maxSteps=maxSteps,
                                   raiseGuardViolations=raiseGuardViolations, timestep=timestep)
    if not result:
        return None
    return result.trajectory

# Helpers

def generateChecked(scenario, maxIterations):
    checkVeneerIsInactive()
    scene, iterations = scenario.generate(maxIterations=maxIterations)
    checkVeneerIsInactive()
    return scene, iterations

def checkVeneerIsInactive():
    assert veneer.activity == 0
    assert not veneer.scenarioStack
    assert not veneer.currentScenario
    assert not veneer.evaluatingRequirement
    assert not veneer.evaluatingGuard
    assert not veneer.scenarios
    assert not veneer._globalParameters
    assert not veneer.lockedParameters
    assert not veneer.lockedModel
    assert not veneer.currentSimulation
    assert not veneer.currentBehavior

## Error checking utilities

def checkErrorLineNumber(line, exc_info=None):
    if exc_info is None:
        tb = sys.exc_info()[2]
    else:
        tb = exc_info.tb
    while tb.tb_next is not None:
        tb = tb.tb_next
    assert tb.tb_lineno == line
