import scenic
import sys
import time

_, file, iters = sys.argv

iters = int(iters)

scenario = scenic.scenarioFromFile(file)

start_time = time.time()

for _ in range(iters):
    scenario.generate()

print("Mean Time:", (time.time() - start_time) / iters)
