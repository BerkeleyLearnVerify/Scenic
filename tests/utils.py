
from scenic import scenarioFromString as compileScenic

def sampleEgo(scenario, maxIterations=1):
    scene, iterations = scenario.generate(maxIterations=maxIterations)
    return scene.egoObject

def sampleEgoFrom(code, maxIterations=1):
    scenario = compileScenic(code)
    return sampleEgo(scenario, maxIterations=maxIterations)

def sampleParamPFrom(code, maxIterations=1):
    scenario = compileScenic(code)
    scene, iterations = scenario.generate(maxIterations=maxIterations)
    return scene.params['p']
