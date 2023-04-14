..  _whats_new:

What's New in Scenic
====================

This page describes what new features have been added in each version of Scenic, as well as any syntax changes which break backwards compatibility.
Scenic uses semantic versioning, so a program written for Scenic 2.1 should also work in Scenic 2.5, but not necessarily in Scenic 3.0.
You can run :command:`scenic --version` to see which version of Scenic you are using.

Scenic 3.x
++++++++++

The Scenic 3.x series adds native support for 3D geometry, temporal requirements, and a new parser enabling clearer error messages, greater language extensibility, and various improvements to the syntax.

Scenic 3.0.0:
-------------

Backwards-incompatible syntax changes:

	* Objects must be explicitly created using the ``new`` keyword, e.g. ``new Object at (1, 2)`` instead of the old ``Object at (1, 2)``.
	  This removes an ambiguity in the Scenic grammar, and makes non-creation uses of class names like ``myClasses = [Car, Bicycle, Pedestrian]`` clearer.

	* As the ``heading`` property is now derived from the 3D ``orientation`` (see below), it can no longer be set directly.
	  This only matters if you wrote code like ``with heading 30 deg``; the more idiomatic ``facing 30 deg`` will still work.

Major new features:

	* Scenic uses 3D geometry.
	  Vectors now have 3 coordinates: if a third coordinate is not provided, it is assumed to be zero, so that scenarios taking place entirely in the z=0 plane will continue to work as before.
	  Orientations of objects in space are represented by a new ``orientation`` property (internally a quaternion), which is computed by applying intrinsic ``yaw``, ``pitch``, and ``roll`` rotations, given by new properties by those names.
	  These rotations are applied to the object's ``parentOrientation``, which by default aligns with the Scenic global coordinate system but is optionally specified by :sampref:`left of` and similar specifiers; this makes it easy to orient an object with respect to another object.

	* Scenic models the precise shapes of objects, rather than simply using bounding boxes for collision detection and visibility checks.
	  Objects have a new ``shape`` property (an instance of the `Shape` class) representing their shape; shapes can be created from standard 3D mesh formats such as STL.

	* The :keyword:`require` statement accepts arbitrary properties in Linear Temporal Logic (not just the :samp:`require always` and :samp:`require eventually` forms previously allowed).

Minor new features:

	* Syntax errors should now always indicate the correct part of the source code.

Scenic 2.x
++++++++++

The Scenic 2.x series is a major new version of Scenic which adds native support for dynamic scenarios, scenario composition, and more.

Scenic 2.1.0
------------

Major new features:

	* Modular scenarios and ways to compose them together, introduced as a prototype in 2.0.0, are now finalized, with many fixes and improvements. See :ref:`composition` for an overview of the new syntax.

	* The :keyword:`record` statement for recording values at every step of dynamic simulations (or only at the start/end).

	* A built-in Newtonian physics simulator for debugging dynamic scenarios without having to install an external simulator (see `scenic.simulators.newtonian`).

	* The interface to the Webots simulator has been greatly generalized, and now supports dynamic scenarios (see `scenic.simulators.webots`).

Minor new features:

	* You can now write :scenic:`require {expr} as {name}` to give a name to a requirement; similarly for :scenic:`require always`, termination conditions, etc.

	* Compatibility with Python 3.7 is restored. Scenic 2 now supports all versions of Python from 3.7 to 3.11.

Scenic 2.0.0
------------

Backwards-incompatible syntax changes:

	* The interval notation :scenic:`(low, high)` for uniform distributions has been removed: use :scenic:`Range(low, high)` instead. As a result of this change, the usual Python syntax for tuples is now legal in Scenic.

	* The :prop:`height` property of `Object`, measuring its extent along the Y axis, has been renamed :prop:`length` to better match its intended use. The name :prop:`height` will be used again in a future version of Scenic with native support for 3D geometry.

Major new features:

	* Scenic now supports writing and executing dynamic scenarios, where agents take actions over time according to behaviors specified in Scenic. See :ref:`dynamics` for an overview of the new syntax.

	* An abstract :ref:`driving_domain` allowing traffic scenarios to be written in a platform-agnostic way and executed in multiple simulators (in particular, both CARLA and LGSVL).
	  This library includes functionality to parse road networks from standard formats (currently OpenDRIVE) and expose information about them for use in Scenic scenarios.

	* A much generalized and improved interface to CARLA. (Many thanks to the CARLA team for contributing this.)

	* An interface to the LGSVL driving simulator. (Many thanks to the LG team for helping develop this interface.)

Minor new features:

	* Operators and specifiers which take vectors as arguments will now accept tuples and lists of length 2; for example, you can write :scenic:`Object at (1, 2)`. The old syntax :scenic:`Object at 1@2` is still supported.

	* The :keyword:`model` statement allows a scenario to specify which :term:`world model` it uses, while being possible to override from the command line with the :option:`--model` option.

	* Global parameters can be overridden from the command line using the :option:`--param` option (e.g. to specify a different map to use for a scenario).

	* The unpacking operator :scenic:`*` can now be used with :scenic:`Uniform` to select a random element of a random list/tuple (e.g. :scenic:`lane = Uniform(*network.lanes); sec = Uniform(*lane.sections)`).

	* The Python built-in function `filter` is now supported, and can be used along with unpacking as above to select a random element of a random list satisfying a given condition (see :ref:`filter_func` for an example).

(Many other minor features didn't make it into this list.)
