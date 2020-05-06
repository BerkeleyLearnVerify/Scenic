Interfacing to New Simulators
=============================

To interface Scenic to a new simulator, there are two steps: using the Scenic API to compile scenarios and generate scenes, and writing a Scenic library defining the virtual world provided by the simulator.

Using the Scenic API
--------------------

Compiling a Scenic scenario is easy: just call the `scenic.scenarioFromFile` function with the path to a Scenic file (there's also a variant `scenic.scenarioFromString` which works on strings).
This returns a `Scenario` object representing the scenario; to sample a scene from it, call its `generate` method.
Scenes are represented by `Scene` objects, from which you can extract the objects and their properties as well as the values of the global parameters (see the `Scene` documentation for details).

Defining a World Model
----------------------

To make writing scenarios for your simulator easier, you should write a Scenic library specifying all the relevant information about the simulated world.
This "world model" could include:

	* Scenic classes (subclasses of :obj:`~scenic.core.object_types.Object`) corresponding to types of objects in the simulator;
	* instances of :obj:`~scenic.core.regions.Region` corresponding to locations of interest (e.g. one for each road);
	* a :obj:`~scenic.core.workspaces.Workspace` specifying legal locations for objects (and optionally providing methods for schematically rendering scenes);
	* any other information that might be useful in scenarios.

Then any Scenic programs for your simulator can import this world model and make use of the information within.

Each of the simulators natively supported by Scenic has a corresponding ``model.sc`` file containing its world model.
See the :doc:`simulators` page for links to the module under `scenic.simulators` for each simulator, where the world model can be found.
The `scenic.simulators.webots.mars` model is particularly simple and would be a good place to start.
