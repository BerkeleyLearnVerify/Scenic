..  _data:

***********************************
Data Types Reference
***********************************

Primitive Data Types
====================

.. _Booleans:

Booleans
--------
Booleans represent truth values, and can be `True` or `False`.

.. _Scalars:

Scalars
-------
Scalars represent distances, angles, etc. as floating-point numbers, which can be sampled from various distributions.

.. _Vectors:

Vectors
-------
Vectors represent positions and offsets in space.
They are constructed from coordinates with the syntax :samp:`{X} @ {Y}` (inspired by `Smalltalk <http://stephane.ducasse.free.fr/FreeBooks/BlueBook/Bluebook.pdf>`_); using a length-2 list or tuple (:samp:`[{X}, {Y}]` or :samp:`({X}, {Y})`) is also allowed.
By convention, coordinates are in meters, although the semantics of Scenic does not depend on this.
More significantly, the vector syntax is specialized for 2-dimensional space.
The 2D assumption dramatically simplifies much of Scenic’s syntax (particularly that dealing with orientations, as we will see below), while still being adequate for a variety of applications.
Some :term:`world models`, such as that for the :ref:`driving_domain`, define an ``elevation`` property which defines a 3D location in combination with Scenic's 2D ``position`` property.
A future extension of Scenic will natively support 3D space.

For convenience, instances of `Point` can be used in any context where a vector is expected: so for example if ``P`` is a `Point`, then ``P offset by (1, 2)`` is equivalent to ``P.position offset by (1, 2)``.

.. _Headings:

Headings
--------
Headings represent orientations in space.
Conveniently, in 2D these can be expressed using a single angle (rather than Euler angles or a quaternion).
Scenic represents headings in radians, measured anticlockwise from North, so that a heading of 0 is due North and a heading of π/2 is due West.
We use the convention that the heading of a local coordinate system is the heading of its Y-axis, so that, for example, the vector ``-2 @ 3`` means 2 meters left and 3 ahead.

For convenience, instances of `OrientedPoint` can be used in any context where a heading is expected: so for example if ``OP`` is an `OrientedPoint`, then ``relative heading of OP`` is equivalent to ``relative heading of OP.heading``.
Since `OrientedPoint` is a subclass of `Point`, expressions involving two oriented points like ``OP1 relative to OP2`` can be ambiguous: the polymorphic operator ``relative to`` accepts both vectors and headings, and either version could be meant here.
Scenic rejects such expressions as being ambiguous: more explicit syntax like ``OP1.position relative to OP2`` must be used instead.

.. _Vector Fields:

Vector Fields
-------------
Vector fields associate an orientation (i.e. a heading) to each point in space.
For example, a vector field could represent the shortest paths to a destination, or the nominal traffic direction on a road (e.g. `scenic.domains.driving.model.roadDirection`).

.. _Regions:

Regions
-------
Regions represent sets of points in space.
Scenic provides a variety of ways to define Regions: rectangles, circular sectors, line segments, polygons, occupancy grids, and explicit lists of points (see the subclasses of `Region`).

Regions can have an associated vector field giving points in the region :term:`preferred orientations`.
For example, a Region representing a lane of traffic could have a preferred orientation aligned with the lane, so that we can easily talk about distances along the lane, even if it curves.
Another possible use of preferred orientations is to give the surface of an object normal vectors, so that other objects placed on the surface face outward by default.
