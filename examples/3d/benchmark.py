import scenic
import trimesh


for i in range(1,9):
    print(f"Starting benchmark {i}")
    scenario = scenic.scenarioFromFile(f"examples/3d/3d_debug_{i}.scenic")
    scenario.generate()
