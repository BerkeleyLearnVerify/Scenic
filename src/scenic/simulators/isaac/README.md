## **Installation**

To interface with Isaac Sim, follow these steps:

1. Follow the instructions [here](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html) to install Isaac Sim Python packages into your Scenic Python Virtual Environment.

2. To test that everything is working, try the following:
```python
import scenic
from scenic.simulators.isaac.simulator import IsaacSimSimulator
scenario = scenic.scenarioFromFile("examples/isaacsim/robot/create3.scenic",
                                   model='scenic.simulators.isaac.model')
scene, _ = scenario.generate()
simulator = IsaacSimSimulator()
simulation = simulator.simulate(scene, maxSteps=1000)
```

When running Isaac Sim examples from the command line, use the `-S` flag:

`scenic -S -b examples/isaacsim/robot/create3.scenic --count 1`

Some examples use a non-default backend. For example,
`examples/isaacsim/robot/franka_move_to.scenic` sets
`param isaacBackend = "experimental_60"` and can be run with:

`scenic -S -b examples/isaacsim/robot/franka_move_to.scenic --count 1`

## **Known Issues**

1. There seems to be a slight mismatch between the location of objects placed in Scenic and those placed in Isaac Sim. This causes problems with the Scenic mutate statement, as differences in positions will cause well-defined Scenic programs to create Isaac Sim programs with intersecting 3D meshes. To see this, try the vacuum example with `mutate couch, coffee_table` (and default standard deviations).

2. Sometimes, repairing a complex converted mesh will not result in a reasonable volume. One example of this is the Jetbot robot.

3. Some robots, like Franka, require setup after being added to the simulator. The current Franka primitive-control path performs this setup lazily before the first primitive action, but new robot backends may need similar setup.

## **Assets**

A local copy of Isaac Sim assets can be obtained [here](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#isaac-sim-latest-release). To convert assets from USD to glTF, the `usd_to_mesh.py` script can be used under `scenic/simulators/isaac`. Example usage for a folder of assets looks like:

`python src/scenic/simulators/isaac/usd_to_mesh.py --folders /path/to/assets --environments warehouse.usd`

In this example, we convert a folder of usd assets to glTF, and we specify that one of the assets in the folder is an environment. For each environment file, a JSON info file will generated. The generated files will be located at `/path/to/assets_converted`. Important note: USD environment files must be flattened by the user before they are converted (the process involves moving Prims, which requires a flattened usd file).
