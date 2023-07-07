"""
Generate a room for the i-roomba create vacuum
"""

from vacuum_lib import *

## Scene Layout ##

# Create room region and set it as the workspace
room_region = RectangularRegion(0 @ 0, 0, 5.09, 5.09)
workspace = Workspace(room_region)

# Create floor and walls
floor = new Floor
wall_offset = floor.width/2 + 0.04/2 + 1e-4
right_wall = new Wall at (wall_offset, 0, 0.25), facing toward floor
left_wall = new Wall at (-wall_offset, 0, 0.25), facing toward floor
front_wall = new Wall at (0, wall_offset, 0.25), facing toward floor
back_wall = new Wall at (0, -wall_offset, 0.25), facing toward floor

# Place vacuum on floor
ego = new Vacuum on floor

# Create a "safe zone" around the vacuum so that it does not start stuck
safe_zone = CircularRegion(ego.position, radius=1)

# Create a dining room region where we will place dining room furniture
dining_room_region = RectangularRegion(1.25 @ 0, 0, 2.5, 5).difference(safe_zone)

# Place a table with 3 chairs around it, and one knocked over on the floor
dining_table = new DiningTable contained in dining_room_region, on floor,
    facing Range(0, 360 deg)

chair_1 = new DiningChair behind dining_table by -0.1, on floor,
                facing toward dining_table, with regionContainedIn dining_room_region
chair_2 = new DiningChair ahead of dining_table by -0.1, on floor,
                facing toward dining_table, with regionContainedIn dining_room_region
chair_3 = new DiningChair left of dining_table by -0.1, on floor,
                facing toward dining_table, with regionContainedIn dining_room_region

fallen_orientation = Uniform((0, -90 deg, 0), (0, 90 deg, 0), (0, 0, -90 deg), (0, 0, 90 deg))

chair_4 = new DiningChair contained in dining_room_region, facing fallen_orientation,
                on floor, with baseOffset(0,0,-0.2)

# Add some noise to the positions and yaw of the chairs around the table
mutate chair_1, chair_2, chair_3

# Create a living room region where we will place living room furniture
living_room_region = RectangularRegion(-1.25 @ 0, 0, 2.5, 5).difference(safe_zone)

couch = new Couch ahead of left_wall by 0.335,
            on floor, facing away from left_wall

coffee_table = new CoffeeTable ahead of couch by 0.336,
            on floor, facing away from couch

# Add some noise to the positions of the couch and coffee table
mutate couch, coffee_table

toy_stack = new BlockToy on floor
toy_stack = new BlockToy on toy_stack
toy_stack = new BlockToy on toy_stack

# Spawn some toys
for _ in range(globalParameters.numToys):
    new Toy on floor

## Simulation Setup ##
terminate after globalParameters.duration * 60 seconds
record (ego.x, ego.y) as VacuumPosition
