..  _whats_new:

What's New in Scenic
====================

This page describes what new features have been added in each version of Scenic, as well as any syntax changes which break backwards compatibility.
Scenic uses semantic versioning, so a program written for Scenic 2.1 should also work in Scenic 2.5, but not necessarily in Scenic 3.0.
You can run ``scenic --version`` to see which version of Scenic you are using.

Scenic 2.x
++++++++++

The Scenic 2.x series is a major new version of Scenic which adds native support for dynamic scenarios, scenario composition, and more.

Scenic 2.1.0
------------

Major new features:

	* Modular scenarios and ways to compose them together, introduced as a prototype in 2.0.0, are now finalized, with many fixes and improvements. See :ref:`composition` for an overview of the new syntax.

	* The :samp:`record {expr} as {name}`, :samp:`record initial {expr}`, and :samp:`record final {expr}` statements for recording values at every step of dynamic simulations (or only at the start/end).

	* A built-in Newtonian physics simulator for debugging dynamic scenarios without having to install an external simulator (see `scenic.simulators.newtonian`).

	* The interface to the Webots simulator has been greatly generalized, and now supports dynamic scenarios (see `scenic.simulators.webots`).

Minor new features:

	* You can now write :samp:`require {expr} as {name}` to give a name to a requirement; similarly for :samp:`require always`, termination conditions, etc.

	* Compatibility with Python 3.7 is restored. Scenic now supports all versions of Python from 3.7 to 3.10.

Scenic 2.0.0
------------

Backwards-incompatible syntax changes:

	* The interval notation ``(low, high)`` for uniform distributions has been removed: use ``Range(low, high)`` instead. As a result of this change, the usual Python syntax for tuples is now legal in Scenic.

	* The ``height`` property of `Object`, measuring its extent along the Y axis, has been renamed ``length`` to better match its intended use. The name ``height`` will be used again in a future version of Scenic with native support for 3D geometry.

Major new features:

	* Scenic now supports writing and executing dynamic scenarios, where agents take actions over time according to behaviors specified in Scenic. See :ref:`dynamics` for an overview of the new syntax.

	* An abstract :ref:`driving_domain` allowing traffic scenarios to be written in a platform-agnostic way and executed in multiple simulators (in particular, both CARLA and LGSVL).
	  This library includes functionality to parse road networks from standard formats (currently OpenDRIVE) and expose information about them for use in Scenic scenarios.

	* A much generalized and improved interface to CARLA. (Many thanks to the CARLA team for contributing this.)

	* An interface to the LGSVL driving simulator. (Many thanks to the LG team for helping develop this interface.)

Minor new features:

	* Operators and specifiers which take vectors as arguments will now accept tuples and lists of length 2; for example, you can write ``Object at (1, 2)``. The old syntax ``Object at 1@2`` is still supported.

	* The ``model`` statement allows a scenario to specify which world model it uses, while being possible to override from the command line with the :option:`--model` option.

	* Global parameters can be overridden from the command line using the :option:`--param` option (e.g. to specify a different map to use for a scenario).

	* The unpacking operator ``*`` can now be used with ``Uniform`` to select a random element of a random list/tuple (e.g. :samp:`lane = Uniform(*network.lanes); sec = Uniform(*lane.sections)`).

	* The Python built-in function `filter` is now supported, and can be used along with unpacking as above to select a random element of a random list satisfying a given condition.

(Many other minor features didn't make it into this list.)
