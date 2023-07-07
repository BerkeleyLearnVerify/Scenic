import math
import time

import matplotlib
from multiprocess import Process

import scenic

matplotlib.rcParams.update({"font.size": 4})

MAX_TIME = 120
TRIALS_PER = 50
COLORS = ["red", "blue", "green", "orange", "yellow"]

BENCHMARKS = [
    ("narrowGoalOld.scenic", {"mode2D": True}),
    ("narrowGoalNew.scenic", {}),
    ("city_intersection.scenic", {}),
    ("vacuum.scenic", {"numToys": 0}),
    ("vacuum.scenic", {"numToys": 1}),
    ("vacuum.scenic", {"numToys": 2}),
    ("vacuum.scenic", {"numToys": 4}),
    ("vacuum.scenic", {"numToys": 8}),
    ("vacuum.scenic", {"numToys": 16}),
]

PARAMS = {
    "initialCollisionCheck": (False, True),
}


def run_benchmark(path, params):
    if "mode2D" in params:
        del params["mode2D"]
        mode2D = True
    else:
        mode2D = False
    scenario = scenic.scenarioFromFile(path, params=params, mode2D=mode2D)
    scenario.generate(maxIterations=float("inf"))


if __name__ == "__main__":
    # Gather times
    results = {}

    for benchmark, benchmark_params in BENCHMARKS:
        for param in PARAMS:
            results_val = []

            for param_val in PARAMS[param]:
                params = benchmark_params.copy()
                params[param] = param_val
                total_time = 0
                for trial_iter in range(TRIALS_PER):
                    p = Process(target=run_benchmark, args=[benchmark, params])
                    start = time.time()
                    p.start()
                    p.join(timeout=MAX_TIME)
                    p.kill()
                    trial_time = min(time.time() - start, MAX_TIME)

                    print(
                        f"({benchmark},{benchmark_params},{param},{param_val},{trial_iter}): {trial_time:.2f}s"
                    )

                    total_time += trial_time

                mean_time = total_time / TRIALS_PER
                assert 0 <= mean_time <= MAX_TIME
                results_val.append((param_val, mean_time))

            results[(str((benchmark, benchmark_params)), param)] = results_val

    # Plot times
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(
        len(BENCHMARKS), len(PARAMS), figsize=(6 * len(PARAMS), 1.75 * len(BENCHMARKS))
    )
    for b_iter, benchmark in enumerate(BENCHMARKS):
        for p_iter, param in enumerate(PARAMS):
            if len(PARAMS) > 1:
                target_axis = axis[b_iter, p_iter]
            else:
                target_axis = axis[b_iter]
            target_results = results[(str(benchmark), param)]

            if p_iter == 0:
                target_axis.xaxis.set_label_position("top")
                target_axis.set_ylabel(f"{benchmark[0]}, {str(benchmark[1])}")

            if b_iter == 0:
                target_axis.xaxis.set_label_position("top")
                target_axis.set_xlabel(param)

            x, y = [v[0] for v in target_results], [v[1] for v in target_results]

            # target_axis.set_ylim([0, MAX_TIME])

            label_locs = []
            label_vals = []
            for l_iter, l_val in enumerate(PARAMS[param]):
                label_locs.append(x[l_iter])
                label_vals.append(l_val)

            target_axis.set_xticks(label_locs, label_vals)
            # target_axis.plot(x, y, color=COLORS[p_iter])
            target_axis.bar(x, y, color="maroon")

    plt.savefig("collision_results.pdf")
