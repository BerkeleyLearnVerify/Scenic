model scenic.domains.driving.model

# parse out roads with two lanes, with opposite traffic directions
two_lane_roads = filter(lambda i: len(i.lanes)==2 and len(i.laneGroups)==2, network.roads)
select_road = Uniform(*two_lane_roads)
lane = Uniform(*select_road.laneGroups)

ego = Car on lane
spot = OrientedPoint on visible road
offset = Range(5,10)
Car ahead of spot by Range(10,20),
	facing offset relative to roadDirection
Car behind spot by Range(5,10)