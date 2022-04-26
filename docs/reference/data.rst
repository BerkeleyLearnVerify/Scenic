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
Scalars represent distances, angles, etc. as floating-point numbers, which can be sampled from various distributions

.. _Vectors:

Vectors
-------
Vectors represent positions and offsets in space. They are constructed from coordinates with the syntax :samp:`{X} @ {Y}` (inspired by `Smalltalk <http://stephane.ducasse.free.fr/FreeBooks/BlueBook/Bluebook.pdf>`_); using a length-2 list or tuple (:samp:`[{X}, {Y}]` or :samp:`({X}, {Y})`) is also allowed.
By convention, coordinates are in meters, although the semantics of Scenic does not depend on this. More significantly, the vector syntax is specialized for 2-dimensional space. The 2D assumption dramatically simplifies much of Scenic’s syntax (particularly that dealing with orientations, as we will see below), while still being adequate for a variety of applications. However, it is important to note that the fundamental ideas of Scenic are not specific to 2D, and it would be easy to extend our implementation of the language to support 3D space.

.. _Headings:

Headings
--------
Headings represent orientations in space. Conveniently, in 2D these can be expressed using a single angle (rather than Euler angles or a quaternion). Scenic represents headings in radians, measured anticlockwise from North, so that a heading of 0 is due North and a heading of π/2 is due West. We use the convention that the heading of a local coordinate system is the heading of its y-axis, so that, for example, -2 @ 3 means 2 meters left and 3 ahead.

.. _Vector Fields:

Vector Fields
-------------
Vector fields associate an orientation (i.e. a heading) to each point in space. For example, a vector field could represent the shortest paths to a destination, or the nominal traffic direction on a road.

.. _Regions:

Regions
-------
Regions represent sets of points in space. Scenic provides a variety of ways to define Regions: rectangles, circular sectors, line segments, polygons, occupancy grids, and explicit lists of points. Regions can have an associated vector field giving points in the region preferred orientations. For example, a Region representing a lane of traffic could have a preferred orientation aligned with the lane, so that we can easily talk about distances along the lane, even if it curves. Another possible use of preferred orientations is to give the surface of an object normal vectors, so that other objects placed on the surface face outward by default.
