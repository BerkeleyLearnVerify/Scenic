import itertools
import json
import math
from pathlib import Path
import statistics
import time

from multiprocess import Process
from threadpoolctl import threadpool_limits

import scenic
from scenic.core.sample_checking import WeightedAcceptanceChecker

MAX_TIME = 20 * 60
TRIALS_PER = {1: 25, 10: 10, 100: 5}
SCENE_COUNT = [1, 10, 100]
BENCHMARKS = [
    ("narrowGoalOld.scenic", {"mode2D": True}),
    ("bumperToBumper.scenic", {"mode2D": True}),
    ("badlyParkedCarPullingIn.scenic", {"mode2D": True}),
    ("adjacentOpposingPair.scenic", {"mode2D": True}),
    ("carInFront.scenic", {"mode2D": True}),
    ("mediumTraffic.scenic", {"mode2D": True}),
    ("opposingPair.scenic", {"mode2D": True}),
    ("angledPlatoonInFront.scenic", {"mode2D": True}),
    ("parkedPlatoon.scenic", {"mode2D": True}),
    ("bypassing_03.scenic", {"mode2D": True}),
    ("pedestrian_02.scenic", {"mode2D": True}),
    ("narrowGoalNew.scenic", {}),
    ("city_intersection.scenic", {}),
    ("vacuum.scenic", {"numToys": 0}),
    ("vacuum.scenic", {"numToys": 1}),
    ("vacuum.scenic", {"numToys": 2}),
    ("vacuum.scenic", {"numToys": 4}),
    ("vacuum.scenic", {"numToys": 8}),
    ("vacuum.scenic", {"numToys": 16}),
]

SAMPLE_CHECKERS = [
    "BasicChecker",
    "WeightedAcceptanceChecker_1",
    "WeightedAcceptanceChecker_10",
    "WeightedAcceptanceChecker_100",
]

NUM_CORES = 16


def make_scenario(path, params):
    params = params.copy()
    if "mode2D" in params:
        del params["mode2D"]
        mode2D = True
    else:
        mode2D = False
    return scenic.scenarioFromFile(
        Path("benchmarks") / path, params=params, mode2D=mode2D
    )


def run_benchmark(scenario, sample_checker, num_scenes):
    if sample_checker == "BasicChecker":
        pass
    elif sample_checker == "WeightedAcceptanceChecker_1":
        scenario.setSampleChecker(WeightedAcceptanceChecker(bufferSize=1))
    elif sample_checker == "WeightedAcceptanceChecker_10":
        scenario.setSampleChecker(WeightedAcceptanceChecker(bufferSize=10))
    elif sample_checker == "WeightedAcceptanceChecker_100":
        scenario.setSampleChecker(WeightedAcceptanceChecker(bufferSize=100))

    with threadpool_limits(limits=NUM_CORES, user_api="blas"):
        scenario.generateBatch(numScenes=num_scenes)


if __name__ == "__main__":
    # Gather times
    sc_results = {}

    print("Compiling scenarios...")
    scenarios = {str(benchmark): make_scenario(*benchmark) for benchmark in BENCHMARKS}
    print("Compiling done!")

    for sample_checker in SAMPLE_CHECKERS:
        results_val = {}
        for benchmark, num_scenes in itertools.product(BENCHMARKS, SCENE_COUNT):
            benchmark_name, benchmark_params = benchmark
            scenario = scenarios[str(benchmark)]
            times = []
            for trial_iter in range(TRIALS_PER[num_scenes]):
                p = Process(
                    target=run_benchmark, args=[scenario, sample_checker, num_scenes]
                )
                start = time.perf_counter()
                p.start()
                p.join(timeout=MAX_TIME)
                p.kill()
                trial_time = min(time.perf_counter() - start, MAX_TIME)

                print(
                    f"({sample_checker},{benchmark},{num_scenes},{trial_iter}): {trial_time:.2f}s"
                )

                times.append(trial_time)

            median_times = statistics.median(times)
            assert 0 <= median_times <= MAX_TIME
            results_val[(str(benchmark), num_scenes)] = median_times
        sc_results[sample_checker] = results_val

    # Dump results
    for sample_checker in SAMPLE_CHECKERS:
        sc_json = {
            "preamble": {
                "program": sample_checker,
            },
            "stats": {},
        }

        stats_json = sc_json["stats"]

        for name, median_times in sc_results[sample_checker].items():
            stats_json[str(name)] = {"status": True, "rtime": median_times}

        with open(Path("results") / f"{sample_checker}.json", "w") as f:
            json.dump(sc_json, f, indent=4)
