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
		A Scenic :obj:`~scenic.core.object_types.Object` which has a dynamic behavior.

	dynamic properties
		Properties of Scenic objects which are updated at each time step of a dynamic
		simulation. The built-in properties representing positions, orientations,
		velocities, etc. are all dynamic. See the source code of
		:obj:`scenic.domains.driving.model.DrivingObject` for an example of defining a
		dynamic property.
