import numpy as np
from scenic.simulators.carlo.carlo.world import World
from scenic.simulators.carlo.carlo.agents import Car, RingBuilding, CircleBuilding, RectangleBuilding, Painting, Pedestrian
from scenic.simulators.carlo.carlo.geometry import Point
import time
from tkinter import *

# Basic Circular Road and Basic Intersection credited to Erdem Biyik and CARLO
def basic_circular_road():
    dt = 0.1  # time steps in terms of seconds. In other words, 1/dt is the FPS.
    world_width = 120  # in meters
    world_height = 120
    inner_building_radius = 30
    num_lanes = 2
    lane_marker_width = 0.5
    num_of_lane_markers = 50
    lane_width = 3.5

    w = World(dt, width=world_width, height=world_height,
              ppm=6)  # The world is 120 meters by 120 meters. ppm is the pixels per meter.

    # Let's add some sidewalks and RectangleBuildings.
    # A Painting object is a rectangle that the vehicles cannot collide with. So we use them for the sidewalks / zebra crossings / or creating lanes.
    # A CircleBuilding or RingBuilding object is also static -- they do not move. But as opposed to Painting, they can be collided with.

    # To create a circular road, we will add a CircleBuilding and then a RingBuilding around it
    cb = CircleBuilding(Point(world_width / 2, world_height / 2), inner_building_radius, 'gray80')
    w.add(cb)
    rb = RingBuilding(Point(world_width / 2, world_height / 2),
                      inner_building_radius + num_lanes * lane_width + (num_lanes - 1) * lane_marker_width,
                      1 + np.sqrt((world_width / 2) ** 2 + (world_height / 2) ** 2), 'gray80')
    w.add(rb)

    # Let's also add some lane markers on the ground. This is just decorative. Because, why not.
    for lane_no in range(num_lanes - 1):
        lane_markers_radius = inner_building_radius + (lane_no + 1) * lane_width + (lane_no + 0.5) * lane_marker_width
        lane_marker_height = np.sqrt(2 * (lane_markers_radius ** 2) * (1 - np.cos(
            (2 * np.pi) / (2 * num_of_lane_markers))))  # approximate the circle with a polygon and then use cosine theorem
        for theta in np.arange(0, 2 * np.pi, 2 * np.pi / num_of_lane_markers):
            dx = lane_markers_radius * np.cos(theta)
            dy = lane_markers_radius * np.sin(theta)
            w.add(Painting(Point(world_width / 2 + dx, world_height / 2 + dy), Point(lane_marker_width, lane_marker_height),
                           'white', heading=theta))
    return w

def basic_intersection():
    dt = 0.1  # time steps in terms of seconds. In other words, 1/dt is the FPS.
    w = World(dt, width=120, height=120, ppm=6)  # The world is 120 meters by 120 meters. ppm is the pixels per meter.

    # Let's add some sidewalks and RectangleBuildings.
    # A Painting object is a rectangle that the vehicles cannot collide with. So we use them for the sidewalks.
    # A RectangleBuilding object is also static -- it does not move. But as opposed to Painting, it can be collided with.
    # For both of these objects, we give the center point and the size.
    w.add(Painting(Point(71.5, 106.5), Point(97, 27), 'gray80'))  # We build a sidewalk.
    w.add(RectangleBuilding(Point(72.5, 107.5),
                            Point(95, 25)))  # The RectangleBuilding is then on top of the sidewalk, with some margin.

    # Let's repeat this for 4 different RectangleBuildings.
    w.add(Painting(Point(8.5, 106.5), Point(17, 27), 'gray80'))
    w.add(RectangleBuilding(Point(7.5, 107.5), Point(15, 25)))

    w.add(Painting(Point(8.5, 41), Point(17, 82), 'gray80'))
    w.add(RectangleBuilding(Point(7.5, 40), Point(15, 80)))

    w.add(Painting(Point(71.5, 41), Point(97, 82), 'gray80'))
    w.add(RectangleBuilding(Point(72.5, 40), Point(95, 80)))

    # Let's also add some zebra crossings, because why not.
    w.add(Painting(Point(18, 81), Point(0.5, 2), 'white'))
    w.add(Painting(Point(19, 81), Point(0.5, 2), 'white'))
    w.add(Painting(Point(20, 81), Point(0.5, 2), 'white'))
    w.add(Painting(Point(21, 81), Point(0.5, 2), 'white'))
    w.add(Painting(Point(22, 81), Point(0.5, 2), 'white'))
    return w

circular_road = basic_circular_road()
intersection = basic_intersection()

NAME_MAP = {"circular_road": circular_road, "intersection": intersection}