
from scenic import scenarioFromString as compileScenic
import scenic.syntax.veneer as veneer

## Scene generation utilities

# Top level

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
    assert len(veneer.pendingRequirements) == 0
    assert len(veneer.inheritedReqs) == 0
