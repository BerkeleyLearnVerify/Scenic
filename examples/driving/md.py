import scenic
from scenic.simulators.metadrive import MetaDriveSimulator
from scenic.simulators.newtonian import NewtonianSimulator

# Compile the scenario in 2D compatibility mode, specifying MetaDrive as the model
scenario = scenic.scenarioFromFile(
  "examples/driving/md.scenic",
  model="scenic.simulators.metadrive.model",
  params={
    "sumo_map": "assets/maps/CARLA/Town04.net.xml",
    "render": True,
    "render3D": True,
    "timestep": 0.1,
    "real_time": True,
    "use2DMap": 0,  # Use 2D map compatibility
  }
)

scene, _ = scenario.generate()
simulator = MetaDriveSimulator(sumo_map=scenario.params['sumo_map'],
                               render=scenario.params['render'],
                               render3D=scenario.params['render3D'],
                               timestep=scenario.params['timestep'],
                               real_time=scenario.params['real_time'])
# simulator = NewtonianSimulator()
simulation = simulator.simulate(scene)
#if simulation:  # `simulate` can return None if simulation fails
#    result = simulation.result
#    for i, state in enumerate(result.trajectory):
#        egoPos, parkedCarPos = state
#        print(f'Time step {i}: ego at {egoPos}; parked car at {parkedCarPos}')