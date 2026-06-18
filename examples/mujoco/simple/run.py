import scenic
from scenic.core.scenarios import Scene
from scenic.simulators.mujoco.__archive__.simulator3 import MujocoSimulator

if __name__ == "__main__":
    SAMPLES = 1

    for sample_index in range(SAMPLES):
        simulator = MujocoSimulator(xml="")
        scenario = scenic.scenarioFromFile("./examples/mujoco/simple.scenic")

        scene, _ = scenario.generate()
        simulation = simulator.simulate(scene, maxSteps=100000)

        result = simulation.result
