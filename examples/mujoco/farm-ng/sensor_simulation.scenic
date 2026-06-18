from amiga import Amiga, Plant
from scenic.simulators.mujoco.model import Ground, Hill, Terrain
from scenic.simulators.mujoco.simulator import MujocoSimulator
from scenic.simulators.mujoco.sensors import MujocoRGBSensor, MujocoSSSensor

param timestep = 0.01

simulator MujocoSimulator(base_xml_file='amiga_base.xml', use_viewer=True)

hill1 = new Hill at (3, 0, 0), with height 0.5, with spread 0.3, with width 4, with length 4

new Ground at (0, 0, 0),
    with width 20,
    with length 20,
    with gridSize 20,
    with terrain (hill1,)

ego = new Amiga at (0, 0, 0),
    with sensors {
        "rgb_camera": MujocoRGBSensor(
            offset=(0.0, 0.0, 1.5), # On top of the robot.
            width=640,               
            height=480,              
            attributes={
                "fovy": 60,
                "look_direction": (1, 0, 0)  # Look forward (positive x in robot frame).
            }
        ),
        "ss_camera": MujocoSSSensor(
            offset=(0.0, 0.0, 1.5),
            width=640,
            height=480,
            attributes={
                "fovy": 60,
                "look_direction": (1, 0, 0)  # Look forward.
            },
            use_object_types=False
        )
    }

param recordFolder = "out/{simulation}"
record ego.observations["rgb_camera"] to "videos/rgb/test_video_{simulation}.mp4"
record ego.observations["ss_camera"] every 1 seconds to "pictures/ss/ss{simulation}_{time}.jpg"
record ego.observations["rgb_camera"] every 1 seconds to "pictures/rgb/ss{simulation}_{time}.jpg"

# Row 1 (y = -6)
for i in range(-8, 6, 2):
  new Plant at (i, -6, 0.1), with plant_type "corn"

# Row 2 (y = -3)
for j in range(-8, 6, 2):
    new Plant at (j, -3, 0.1), with plant_type "wheat"

# Row 4 (y = 3)
for l in range(-8, 6, 2):
    new Plant at (l, 3, 0.1), with plant_type "lettuce"

# Row 5 (y = 6)
for m in range(-8, 6, 2):
    new Plant at (m, 6, 0.1), with plant_type "tomato"

behavior RealisticDriving():
    """
    Realistic driving behavior using torque control.
    Target velocities are in rad/s for the wheels.
    
    With torque control, the robot will:
    - Slow down going uphill (gravity resists)
    - Speed up going downhill (gravity assists)
    - React realistically to terrain changes
    """
    while True:
        self.target_velocities = [4.0, 4.0, 4.0, 4.0]
        wait

ego.behavior = RealisticDriving()
terminate after 25 seconds