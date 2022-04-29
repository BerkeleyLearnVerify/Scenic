:orphan:

Glossary
========

.. glossary::

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
		The function runs in parallel with the simulation, taking :term:`actions <action>` at each time step.

	dynamic properties
		Properties of Scenic objects which are updated at each time step of a dynamic
		simulation. The built-in properties representing positions, orientations,
		velocities, etc. are all dynamic. See the source code of
		:obj:`scenic.domains.driving.model.DrivingObject` for an example of defining a
		dynamic property.
