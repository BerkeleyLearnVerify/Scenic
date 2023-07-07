import json
from pathlib import Path
import sys

from matplotlib.collections import PatchCollection
from matplotlib.patches import PathPatch
from matplotlib.path import Path as PlotPath
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, box

vacuum_radius = 0.335 / 2

# pass the path to the log to argv[1] and it will make a plot


def main(debug=False, plot=False):
    with open(sys.argv[1]) as f:
        json_str = f.read()
        obj = json.loads(json_str)

        floor = box(-2.5, -2.5, 2.5, 2.5)

        positions = []

        for position in obj.get("results").get("VacuumPosition"):
            x = position[1][0]
            y = position[1][1]
            positions.append((x, y))
        line = LineString(positions)
        dilated = line.buffer(vacuum_radius)

        fig, ax = plt.subplots()
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        plot_polygon(ax, dilated, facecolor="lightblue", edgecolor="red")
        plt.show()


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
