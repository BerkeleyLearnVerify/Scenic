"""Run lane-following simulations and plot lane error over time.

Runs both PID and Pure Pursuit controllers (or a subset), averages lane
error across complete runs at each timestep, and plots them on the same
chart for direct comparison.

Usage::

    python -m scenic.utils.plot_lane_error [OPTIONS]

Example::

    python -m scenic.utils.plot_lane_error --map assets/maps/CARLA/Town05.xodr --sumo-map assets/maps/CARLA/Town05.net.xml
    python -m scenic.utils.plot_lane_error --controller pid              # only PID
    python -m scenic.utils.plot_lane_error --controller pure-pursuit     # only Pure Pursuit
    python -m scenic.utils.plot_lane_error --iterations 20               # more runs for smoother averages
"""

import argparse
import csv
import os
from pathlib import Path

import numpy as np

from scenic import scenarioFromString
from scenic.simulators.metadrive import MetaDriveSimulator

PLOTS_DIR = Path(__file__).resolve().parent / "plots"

CONTROLLER_METHODS = {
    "pure-pursuit": "getPurePursuitControllers",
    "pid": "getLaneFollowingControllers",
}


def build_scenario(map_path, steps, controller="pure-pursuit"):
    """Compile a Scenic scenario that records lane error."""
    duration = int(steps) / 8
    method = CONTROLLER_METHODS[controller]
    code = f"""\
param map = r'{map_path}'

model scenic.simulators.metadrive.model

behavior LaneFollowWithController():
    lon, lat = simulation().{method}(self)
    do FollowLaneBehavior(target_speed=10, lon_controller=lon, lat_controller=lat)

ego = new Car with behavior LaneFollowWithController()

record distance from ego to ego.lane.centerline as laneError
terminate after {duration} seconds
"""
    return scenarioFromString(code, mode2D=True)


def run_simulations(scenario, sumo_map, iterations, expected_steps):
    """Run *iterations* simulations, keeping only complete runs.

    A run is complete if it produced at least *expected_steps* data points.
    Incomplete runs (crashes, off-road, early termination) are discarded.
    """
    all_errors = []
    discarded = 0
    for i in range(iterations):
        try:
            scene, _ = scenario.generate()
            simulator = MetaDriveSimulator(sumo_map, render=False, real_time=False)
            simulation = simulator.simulate(scene)
            if simulation is None or simulation.result is None:
                discarded += 1
                print(f"  run {i + 1}/{iterations} — simulation returned None [discarded]")
                continue
            records = simulation.result.records.get("laneError")
            if records is None:
                discarded += 1
                print(f"  run {i + 1}/{iterations} — no laneError record [discarded]")
                continue
            errors = [record[1] for record in records]
        except Exception as e:
            discarded += 1
            print(f"  run {i + 1}/{iterations} — error: {e} [discarded]")
            continue
        if len(errors) >= expected_steps:
            all_errors.append(errors[:expected_steps])
            print(f"  run {i + 1}/{iterations} — {len(errors)} samples")
        else:
            discarded += 1
            print(f"  run {i + 1}/{iterations} — {len(errors)} samples [discarded, need {expected_steps}]")
    if discarded:
        print(f"  discarded {discarded}/{iterations} incomplete runs")
    return all_errors


def next_run_number(directory, prefix):
    """Find the next available run number for files with the given prefix."""
    existing = (
        [f.name for f in directory.iterdir() if f.name.startswith(prefix)]
        if directory.exists()
        else []
    )
    numbers = []
    for name in existing:
        stem = Path(name).stem
        suffix = stem[len(prefix):]
        if suffix.isdigit():
            numbers.append(int(suffix))
    return max(numbers, default=0) + 1


def write_csv(results, path):
    """Write per-controller averaged lane error data to a CSV file.

    *results* is a dict mapping controller name to a 1-D array of
    per-timestep averaged errors.
    """
    os.makedirs(path.parent, exist_ok=True)
    # All controllers should have the same number of steps
    n_steps = len(next(iter(results.values())))
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["step"] + [f"{name}_avg_error" for name in results]
        writer.writerow(header)
        for step in range(n_steps):
            row = [step] + [results[name][step] for name in results]
            writer.writerow(row)
    print(f"Wrote {path}")


def plot_comparison(results, output_path, n_steps):
    """Plot averaged lane error for each controller on the same chart."""
    import matplotlib.pyplot as plt

    steps = np.arange(n_steps)
    fig, ax = plt.subplots(figsize=(12, 5))

    for name, avg_errors in results.items():
        ax.plot(steps, avg_errors, label=name)

    ax.set_xlabel("Timestep Number")
    ax.set_ylabel("Distance from Center Line (meters)")
    ax.set_title(f"Deviation from Center Line measured for the first {n_steps} timesteps")
    ax.legend()
    ax.set_xlim(0, n_steps - 1)

    plot_path = output_path.with_suffix(".png")
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    print(f"Saved plot to {plot_path}")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run lane-following simulations and compare controller error."
    )
    parser.add_argument(
        "--map",
        default="assets/maps/CARLA/Town05.xodr",
        help="Path to the OpenDRIVE map file",
    )
    parser.add_argument(
        "--sumo-map",
        default="assets/maps/CARLA/Town05.net.xml",
        help="Path to the SUMO network file",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=25,
        help="Number of simulation runs per controller (default: 25)",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=150,
        help="Timesteps per run (default: 150)",
    )
    parser.add_argument(
        "--controller",
        choices=list(CONTROLLER_METHODS),
        nargs="+",
        default=list(CONTROLLER_METHODS),
        help="Controller(s) to test (default: all)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip generating the plot",
    )

    args = parser.parse_args(argv)

    # Build numbered output path
    prefix = "lane_error_"
    run_num = next_run_number(PLOTS_DIR, prefix)
    output_path = PLOTS_DIR / f"{prefix}{run_num:03d}.csv"

    results = {}
    for controller in args.controller:
        print(f"\n=== {controller} ===")
        print(f"Building scenario with map={args.map}")
        scenario = build_scenario(args.map, args.steps, controller=controller)

        print(f"Running {args.iterations} simulations …")
        all_errors = run_simulations(
            scenario, args.sumo_map, args.iterations, args.steps
        )

        if not all_errors:
            print(f"  WARNING: no complete runs for {controller}, skipping")
            continue

        # Average across runs at each timestep
        avg = np.array(all_errors).mean(axis=0)
        results[controller] = avg
        print(f"  averaged {len(all_errors)} complete runs, {len(avg)} timesteps")

    if not results:
        print("No data collected — nothing to write.")
        return

    write_csv(results, output_path)

    if not args.no_plot:
        n_steps = len(next(iter(results.values())))
        plot_comparison(results, output_path, n_steps)


if __name__ == "__main__":
    main()
