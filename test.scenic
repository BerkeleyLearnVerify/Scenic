param map = localPath('./assets/maps/CARLA/Town01.xodr')
param time_step = 1.0 / 10
model scenic.simulators.newtonian.driving_model

# Define the starting positions for the car
# startingPositions = [
#     new Point at (0, 0),  # Replace with actual coordinates
#     # new Point at (15, 25),  # Replace with actual coordinates
#     # new Point at (20, 30)   # Replace with actual coordinates
# ]

# Choose a random starting position from the list
# ego_start = Uniform(*startingPositions)

roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

# Create the ego car at the chosen starting position
ego = new Car on select_lane.centerline, with behavior FollowLaneBehavior

# Define the target position (directly in front of the stopping point)
# target = new Point at (30, 35)  # Replace with the actual stopping point coordinates

# Record distance to the lane centerline
record distance to ego.lane.centerline

# Terminate the simulation when the ego car is close to the target position
# terminate when ((distance from ego to target) <= max(ego.width, ego.length) + 0.05)

# Terminate the simulation after 5 seconds
terminate after 5 seconds
