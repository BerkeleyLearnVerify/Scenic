from collections import defaultdict
import json
from pathlib import Path
from statistics import mean, median, quantiles, stdev

from matplotlib.collections import PatchCollection
from matplotlib.patches import PathPatch
from matplotlib.path import Path as PlotPath
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, box

output_dir = Path(__file__).parent / "logs"

# find all json files in the output directory
files = output_dir.glob("*.json")

vacuum_radius = 0.335 / 2


def main(debug=False, plot=False):
    # number of toys to list of coverage
    aggregate = defaultdict(list)

    for file in files:
        with file.open() as f:
            if debug:
                print(f"processing {file.name}...")
            json_str = f.read()
            obj = json.loads(json_str)

            floor = box(-2.5, -2.5, 2.5, 2.5)

            positions = []

            # Keep one of every 2 points to avoid memory blowup.
            count = 0

            for position in obj.get("results").get("VacuumPosition"):
                if count % 2 == 0:
                    x = position[1][0]
                    y = position[1][1]
                    positions.append((x, y))
                count += 1

            line = LineString(positions)
            dilated = line.buffer(vacuum_radius)

            if plot:
                fig, ax = plt.subplots()
                ax.set_xlim([-2.5, 2.5])
                ax.set_ylim([-2.5, 2.5])
                plot_polygon(ax, dilated, facecolor="lightblue", edgecolor="red")
                plt.show()

            floor_area = floor.area
            cleaned_area = dilated.area
            if debug:
                print(
                    "coverage: {:.2} / {:.2} = {:.2%}".format(
                        cleaned_area, floor_area, cleaned_area / floor_area
                    ),
                )
            aggregate[obj.get("params").get("numToys")].append(cleaned_area / floor_area)

    for numToys, fractions in sorted(aggregate.items(), key=lambda x: x[0]):
        stat = {
            "sample": len(fractions),
            "mean": mean(fractions),
            "median": median(fractions),
            "stdev": stdev(fractions) if len(fractions) > 1 else None,
            "mean robustness": (mean(fractions) - 0.33),
        }
        print(f"coverage for {numToys} toy(s)")
        for key, value in stat.items():
            print(f"    {key}: {value}")
        if debug:
            print("Raw Data:")
            for val in fractions:
                print("{:.2%}".format(val))
            print()


# Plots a Polygon to pyplot `ax`
def plot_polygon(ax, poly, **kwargs):
    path = PlotPath.make_compound_path(
        PlotPath(np.asarray(poly.exterior.coords)[:, :2]),
        *[PlotPath(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors],
    )

    patch = PathPatch(path, **kwargs)
    collection = PatchCollection([patch], **kwargs)

    ax.add_collection(collection, autolim=True)
    ax.autoscale_view()
    return collection


if __name__ == "__main__":
    main()
