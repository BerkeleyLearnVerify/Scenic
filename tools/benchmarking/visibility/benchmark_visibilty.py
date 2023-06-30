import math
import time

import matplotlib
from multiprocess import Process

import scenic

matplotlib.rcParams.update({"font.size": 4})

MAX_TIME = 60
TRIALS_PER = 25
COLORS = ["red", "blue", "green", "orange", "yellow"]

BENCHMARKS = [
    "fully_visible.scenic",
    "fully_occluded.scenic",
    "partially_occluded.scenic",
    "enclosed_occluded.scenic",
    "enclosed_visible.scenic",
    "occlusion_field.scenic",
]

PARAMS = {
    "viewRayDensity": (0.25, 1, 3, 5, 7, 9),
    "batchSize": (1, 8, 64, 128, 512, 2048),
}


def run_benchmark(path, params):
    scenario = scenic.scenarioFromFile(path, params=params)
    scenario.generate(maxIterations=float("inf"))


if __name__ == "__main__":
    # Gather times
    results = {}

    for benchmark in BENCHMARKS:
        for param in PARAMS:
            results_val = []

            for param_val in PARAMS[param]:
                total_time = 0
                for trial_iter in range(TRIALS_PER):
                    p = Process(
                        target=run_benchmark, args=[benchmark, {param: param_val}]
                    )
                    start = time.time()
                    p.start()
                    p.join(timeout=MAX_TIME)
                    p.kill()
                    trial_time = min(time.time() - start, MAX_TIME)

                    print(
                        f"({benchmark},{param},{param_val},{trial_iter}): {trial_time:.2f}s"
                    )

                    total_time += trial_time

                mean_time = total_time / TRIALS_PER
                assert 0 <= mean_time <= MAX_TIME
                results_val.append((param_val, mean_time))

            results[(benchmark, param)] = results_val

    # Plot times
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(
        len(BENCHMARKS), len(PARAMS), figsize=(8 * len(PARAMS), 1.75 * len(BENCHMARKS))
    )

    for b_iter, benchmark in enumerate(BENCHMARKS):
        for p_iter, param in enumerate(PARAMS):
            target_axis = axis[b_iter, p_iter]
            target_results = results[(benchmark, param)]

            if p_iter == 0:
                target_axis.xaxis.set_label_position("top")
                target_axis.set_ylabel(benchmark)

            if b_iter == 0:
                target_axis.xaxis.set_label_position("top")
                target_axis.set_xlabel(param)

            x, y = [v[0] for v in target_results], [v[1] for v in target_results]

            target_axis.set_ylim([0, MAX_TIME])
            target_axis.plot(x, y, color=COLORS[p_iter])

    plt.savefig("visibility_results.pdf")
