from amiga import Amiga, Plant
from scenic.simulators.mujoco.simulator import MujocoSimulator
from scenic.simulators.mujoco.model import Ground, Hill

from scenic.core.regions import RectangularRegion
from scenic.core.vectors import Vector

simulator MujocoSimulator(base_xml_file='amiga_base.xml', use_viewer=True)

# Creating region for spawning objects.
field_region = RectangularRegion(Vector(-20, -10), 0, 40, 20)

# Creating irregular terrain (two hills).
hill1 = new Hill at (10, 10, 0), with height 2, with spread 0.3
hill2 = new Hill at (-10, -10, 0), with height 1.5, with spread 0.4

new Ground at (0, 0, 0),
    with width 50,
    with length 50,
    with gridSize 50,
    with terrain (hill1, hill2)

# Defining regions for the crop rows to ensure that they're parallel.
row_y_positions = [-6, -3, 0, 3, 6]

# Spawning multiple Amiga objects at different positions.   
amiga1 = new Amiga at (8, 1, 0.5),
            facing 0 deg

amiga2 = new Amiga at (-10, 2, 0.5),
            facing 0 deg

# Creating plants along the crop rows.
# Row 1 (y = -6)
for i in range(-8, 6, 2):
  new Plant at (i, -6, 0.1), with plant_type "corn"

# Row 2 (y = -3)
for j in range(-8, 6, 2):
    new Plant at (j, -3, 0.1), with plant_type "wheat"

# # Row 3 (y = 0)
for k in range(-8, 6, 2):
    new Plant at (k, 0, 0.1), with plant_type "soybean"

# # Row 4 (y = 3)
for l in range(-8, 6, 2):
    new Plant at (l, 3, 0.1), with plant_type "lettuce"

# Row 5 (y = 6)
for m in range(-8, 6, 2):
    new Plant at (m, 6, 0.1), with plant_type "tomato"