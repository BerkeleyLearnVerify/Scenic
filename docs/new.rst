..  _whats_new:

What's New in Scenic
====================

This page describes what new features have been added in each version of Scenic, as well as any syntax changes which break backwards compatibility.
Scenic uses semantic versioning, so a program written for Scenic 2.1 should also work in Scenic 2.5, but not necessarily in Scenic 3.0.
You can run ``scenic --version`` to see which version of Scenic you are using.

Scenic 2.x
++++++++++

The Scenic 2.x series is a major new version of Scenic which adds native support for dynamic scenarios, scenario composition, and more.

Scenic 2.0.0
------------

Backwards-incompatible syntax changes:

	* The interval notation ``(low, high)`` for uniform distributions has been removed: use ``Range(low, high)`` instead. As a result of this change, the usual Python syntax for tuples is now legal in Scenic.

	* The ``height`` property of `Object`, measuring its extent along the Y axis, has been renamed ``length`` to better match its intended use. The name ``height`` will be used again in a future version of Scenic with native support for 3D geometry.

Major new features:

	* under construction...

Minor new features:

	* Operators and specifiers which take vectors as arguments will now accept tuples and lists of length 2; for example, you can write ``Object at (1, 2)``. The old syntax ``Object at 1@2`` is still supported.
