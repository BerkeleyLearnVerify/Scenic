..  _new_simulator:

Interfacing to New Simulators
=============================

To interface Scenic to a new simulator, there are two steps: using the Scenic API to compile scenarios, generate scenes, and orchestrate dynamic simulations, and writing a Scenic library defining the virtual world provided by the simulator.

Using the Scenic API
--------------------

Scenic's Python API is covered in more detail in our :doc:`api` page; we summarize the main steps here.

Compiling a Scenic scenario is easy: just call the `scenic.scenarioFromFile` function with the path to a Scenic file (there's also a variant `scenic.scenarioFromString` which works on strings).
This returns a `Scenario` object representing the scenario; to sample a scene from it, call its `generate` method.
Scenes are represented by `Scene` objects, from which you can extract the objects and their properties as well as the values of the global parameters (see the `Scene` documentation for details).

Supporting dynamic scenarios requires additionally implementing a subclass of `Simulator` which communicates periodically with your simulator to implement the actions taken by dynamic agents and read back the state of the simulation.
See the :obj:`scenic.simulators.carla.simulator` and :obj:`scenic.simulators.lgsvl.simulator` modules for examples.

Defining a World Model
----------------------

To make writing scenarios for your simulator easier, you should write a Scenic library specifying all the relevant information about the simulated world.
This "world model" could include:

	* Scenic classes (subclasses of :obj:`~scenic.core.object_types.Object`) corresponding to types of objects in the simulator;
	* instances of :obj:`~scenic.core.regions.Region` corresponding to locations of interest (e.g. one for each road);
	* a :obj:`~scenic.core.workspaces.Workspace` specifying legal locations for objects (and optionally providing methods for schematically rendering scenes);
	* a set of actions (subclasses of :obj:`~scenic.core.simulators.Action`) which can be taken by dynamic agents during simulations;
	* any other information or utility functions that might be useful in scenarios.

Then any Scenic programs for your simulator can import this world model and make use of the information within.

Each of the simulators natively supported by Scenic has a corresponding ``model.scenic`` file containing its world model.
See the :ref:`simulators` page for links to the module under `scenic.simulators` for each simulator, where the world model can be found.
The `scenic.simulators.webots.mars` model is particularly simple and would be a good place to start.
For a more complex example, see the `scenic.simulators.lgsvl` model, which specializes the simulator-agnostic model provided by the :ref:`driving_domain` (in `scenic.domains.driving.model`).
