:orphan:

Glossary
========

.. glossary::
	:sorted:

	action
		A primitive operation executed by an agent during a single step of a dynamic
		simulation. For example, a car might take an action which sets its throttle, or
		turns on its headlights. Actions are defined by the simulator interfaces (or
		abstract domains like `scenic.domains.driving`) as subclasses of
		:obj:`~scenic.core.simulators.Action`.

	agent
		A Scenic :obj:`~scenic.core.object_types.Object` which has a :term:`dynamic behavior` (set as its ``behavior`` property).

	dynamic behavior
		A function defining the behavior of an :term:`agent` during a simulation.
		The function runs in parallel with the simulation, taking :term:`actions` at each time step.
		See our tutorial on :ref:`dynamics` for examples.

	dynamic properties
		Properties of Scenic objects which are updated at each time step of a dynamic
		simulation. The built-in properties representing positions, orientations,
		velocities, etc. are all dynamic (see `Object`). See the source code of
		:obj:`scenic.domains.driving.model.DrivingObject` for an example of defining a
		dynamic property.

	external parameters
		Values which are determined by an external tool instead of Scenic's own sampler.
		These allow using optimization or other techniques to explore parameters of
		Scenic scenarios beyond simple random sampling. For how to define external
		parameters or interface to new external samplers, see
		:mod:`scenic.core.external_params`.

	modular scenario
		A scenario defined using the :sampref:`scenario <modularScenarioDef>` statement (rather than simply being the content of a Scenic file).
		Such scenarios can take arguments, be instantiated multiple times, and be composed with other scenarios: see :ref:`composition`.

	preferred orientation
		A :ref:`vector field <Vector Fields>` set as the ``orientation`` attribute of a `Region`, indicating that objects placed within that region should be oriented to align along that vector field unless otherwise specified.
		For example, the :obj:`~scenic.domains.driving.model.road` region provided by the :ref:`driving_domain` has as its preferred orientation the :obj:`~scenic.domains.driving.model.roadDirection` vector field, so that vehicles positioned using the specifier ``on road`` will be facing the nominal traffic direction at their position by default (i.e., the specifier specifies ``heading`` optionally, so that an explicit :samp:`facing {H}` specifier will override it).

	world model
		A Scenic library defining classes, regions, :term:`actions`, helper functions, etc. for use by scenarios targeting a particular simulator or application domain.
		For example, the world model for the :ref:`driving_domain`, `scenic.domains.driving.model`, defines classes for vehicles, actions for steering, and regions for different parts of the road network.
		In the line ``Car in intersection``, only the ``in`` specifier is built into Scenic: the class :obj:`~scenic.domains.driving.model.Car` and the region :obj:`~scenic.domains.driving.model.intersection` are defined by the world model.
		A world model can be used through the :sampref:`model <model {name}>` statement, or simply by importing it like any other Scenic module.

		.. seealso:: :ref:`defining_world_model` gives further examples and details on how to write a world model.
