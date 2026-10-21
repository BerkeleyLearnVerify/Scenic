"""View an LHT map in Scenic's 2D schematic.

  scenic --2d examples/driving/lht_view.scenic -z 0

Override the map with -p:

  scenic --2d examples/driving/lht_view.scenic -z 0 -p map assets/maps/left-hand-traffic/handmade_oneway_two_lane.xodr
  scenic --2d examples/driving/lht_view.scenic -z 0 -p map assets/maps/left-hand-traffic/handmade_oneway_three_lane.xodr
  scenic --2d examples/driving/lht_view.scenic -z 0 -p map assets/maps/left-hand-traffic/handmade_two_way.xodr
  scenic --2d examples/driving/lht_view.scenic -z 0 -p map assets/maps/left-hand-traffic/handmade_north_south.xodr
  scenic --2d examples/driving/lht_view.scenic -z 0 -p map assets/maps/left-hand-traffic/handmade_t_junction.xodr
  scenic --2d examples/driving/lht_view.scenic -z 0 -p map assets/maps/left-hand-traffic/handmade_two_way_junction.xodr

-z 0 shows the whole map instead of zooming on the car.
"""

param map = localPath('../../assets/maps/left-hand-traffic/handmade_two_way.xodr')
param map_options = {'useCache': False}

model scenic.domains.driving.model

lane = Uniform(*network.lanes)
ego = new Car in lane
