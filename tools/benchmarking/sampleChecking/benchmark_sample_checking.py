import time
import math
from multiprocess import Process
from pathlib import Path
import json
import itertools

import scenic
from scenic.core.sample_checking import WeightedAcceptanceChecker

MAX_TIME = 10*60
TRIALS_PER = 10
SCENE_COUNT = [1,10]
BENCHMARKS = [
                ("narrowGoalOld.scenic", {"mode2D": True}),
                ("bumperToBumper.scenic", {"mode2D": True}),
                ("badlyParkedCarPullingIn.scenic", {"mode2D": True}),
                ("adjacentOpposingPair.scenic", {"mode2D": True}),
                ("carInFront.scenic", {"mode2D": True}),
                ("mediumTraffic.scenic", {"mode2D": True}),
                ("narrowGoalNew.scenic", {}),
                ("city_intersection.scenic", {}),
                ("vacuum.scenic", {"numToys": 0}),
                ("vacuum.scenic", {"numToys": 1}),
                ("vacuum.scenic", {"numToys": 2}),
                ("vacuum.scenic", {"numToys": 4}),
                ("vacuum.scenic", {"numToys": 8}),
                ("vacuum.scenic", {"numToys": 16}),
             ]

SAMPLE_CHECKERS = ["BasicChecker", "WeightedAcceptanceChecker"]

def warmup(path, params):
    params = params.copy()
    if "mode2D" in params:
        del params['mode2D']
        mode2D = True
    else:
        mode2D = False
    scenario = scenic.scenarioFromFile(Path("benchmarks") / path, params=params, mode2D=mode2D)

    scenario.generateBatch(1)

def run_benchmark(path, params, sample_checker, num_scenes):
    if "mode2D" in params:
        del params['mode2D']
        mode2D = True
    else:
        mode2D = False
    scenario = scenic.scenarioFromFile(Path("benchmarks") / path, params=params, mode2D=mode2D)

    if sample_checker == "WeightedAcceptanceChecker":
        scenario.setSampleChecker(WeightedAcceptanceChecker())

    scenario.generateBatch(numScenes=num_scenes)

if __name__ == '__main__':
    # Warmup
    for benchmark_name, benchmark_params in BENCHMARKS:
        print("Warmup:", benchmark_name, benchmark_params)
        warmup(benchmark_name, benchmark_params)

    # Gather times
    sc_results = {}

    for sample_checker in SAMPLE_CHECKERS:
        results_val = {}
        for benchmark, num_scenes in itertools.product(BENCHMARKS, SCENE_COUNT):
            benchmark_name, benchmark_params = benchmark
            total_time = 0
            for trial_iter in range(TRIALS_PER):
                p = Process(target=run_benchmark, args=[benchmark_name, benchmark_params, sample_checker, num_scenes])
                start = time.time()
                p.start()
                p.join(timeout=MAX_TIME)
                p.kill()
                trial_time = min(time.time()-start, MAX_TIME)

                print(f"({sample_checker},{benchmark},{num_scenes},{trial_iter}): {trial_time:.2f}s")

                total_time += trial_time

            mean_time = total_time/TRIALS_PER
            assert 0 <= mean_time <= MAX_TIME
            results_val[(str(benchmark), num_scenes)] = mean_time
        sc_results[sample_checker] = results_val

    # Dump results
    for sample_checker in SAMPLE_CHECKERS:
        sc_json = {
            "preamble": {
                "program": sample_checker,
            },
            "stats": {}
        }

        stats_json = sc_json["stats"]

        for name, mean_time in sc_results[sample_checker].items():
            stats_json[str(name)] = {
                "status": True,
                "rtime": mean_time
            }


        with open(Path("results") / f"{sample_checker}.json", "w") as f:
            json.dump(sc_json, f, indent=4)
