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
See the documentation of `Simulator` and `Simulation` for details on the methods which need to be implemented, and the :obj:`scenic.simulators.carla.simulator` and :obj:`scenic.simulators.webots.simulator` modules for examples.

.. _defining_world_model:

Defining a World Model
----------------------

To make writing scenarios for your simulator easier, you should write a Scenic library specifying all the relevant information about the simulated world.
This :term:`world model` could include:

	* Scenic classes (subclasses of `Object`) corresponding to types of objects in the simulator;
	* instances of `Region` corresponding to locations of interest (e.g. one for each road);
	* a :term:`workspace` specifying legal locations for objects (and optionally providing methods for schematically rendering scenes);
	* a set of :term:`actions` which can be taken by dynamic agents during simulations;
	* any other information or utility functions that might be useful in scenarios.

Then any Scenic programs for your simulator can import this world model and make use of the information within.

Each of the simulators natively supported by Scenic has a corresponding :file:`model.scenic` file containing its world model.
See the :ref:`simulators` page for links to the module under `scenic.simulators` for each simulator, where the world model can be found.
For an example, see the `scenic.simulators.lgsvl` model, which specializes the simulator-agnostic model provided by the :ref:`driving_domain` (in `scenic.domains.driving.model`).
