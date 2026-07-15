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

Some examples use a non-default backend. For example, the Franka primitive
examples can be run with:

`scenic -S -b examples/isaacsim/robot/franka_example_core.scenic --count 1`

`scenic -S -b examples/isaacsim/robot/franka_example_experimental.scenic --count 1`

`scenic -S -b examples/isaacsim/robot/ur5e_example_core.scenic --count 1`

`scenic -S -b examples/isaacsim/robot/ur5e_example_experimental.scenic --count 1`

## **Known Issues / Notes**

1. There can still be small differences between Scenic geometry and the corresponding Isaac Sim USD geometry/colliders, especially for converted or complex assets. The current backend compensates for USD bounding-box offsets when placing assets, but scenarios using `mutate` can still expose unexpected intersections.

2. Sometimes, repairing a complex converted mesh will not result in a reasonable volume.

3. Manipulator profiles still contain values which cannot be inferred reliably
   from USD: the arm and gripper roles, control link, TCP, useful home pose,
   grasp orientation, and open/closed gripper meaning. These values are valid
   only for the exact asset and variant named by the profile. Gripper drive and
   contact values may also require manual tuning for a different asset or task.
   The example clearances and thresholds are scenario policy, not robot
   properties, and are exposed as Scenic parameters.

4. The per-robot values above now live in `backends/profiles.py` as a typed
   `ManipulatorProfile` base with `FrankaProfile` / `UR5eProfile` subclasses
   (`FRANKA_PROFILE` / `UR5E_PROFILE`). `core_51.py` and `experimental_60.py`
   both import these, replacing the two separate, previously divergent `UR5E_*`
   constant blocks they each used to carry. Some of these values could in
   principle be read back from the initialized Isaac articulation rather than
   authored by hand — DOF names and ordering, joint position/velocity limits, and
   the asset's authored gripper drive stiffness/damping (DOF names are already
   validated against the live articulation in `_dof_indices`). The rest are
   genuine design or task choices that are not asset properties: TCP offset,
   home/default pose, downward grasp orientation, IK damping and step scale,
   grasp friction/contact tuning, and the gripper open/closed setpoints.

5. `arm_max_velocities` (a `UR5e` property) is honored only by the experimental
   backend, which clamps the articulation's joint velocities. The `core_51`
   RMPFlow path ignores it, so a cap set in a `core_51` scenario has no effect,
   and the `FrankaPanda` model class exposes no velocity-cap property at all.

6. The Franka pick-place phase geometry (approach/lift heights and per-phase step
   counts) is hardcoded in
   `Experimental60Backend._move_franka_pick_place_helper` which uses the isaac sim built in pick and place controller; the specific franka.scenic example uses this.

## **Assets**

A local copy of Isaac Sim assets can be obtained [here](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#isaac-sim-latest-release).
