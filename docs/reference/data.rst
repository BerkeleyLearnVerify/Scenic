..  _data:

***********************************
Data Types Reference
***********************************

This page describes the primitive data types built into Scenic.
In addition to these types, Scenic provides a class hierarchy for `points <Point>`, `oriented points <OrientedPoint>`, and `objects <Object>`: see the :ref:`objects_and_classes`.


.. _Boolean:

Boolean
=======

Booleans represent truth values, and can be `True` or `False`.

.. note::

	These are equivalent to the Python `bool` type.


.. _Scalar:

Scalar
======

Scalars represent distances, angles, etc. as floating-point numbers, which can be sampled from various distributions.

.. note::

	These are equivalent to the Python `float` type; however, any context which accepts a scalar will also allow an `int` or a NumPy numeric type such as `numpy.single` (to be precise, any instance of `numbers.Real` is legal).


.. _Vector:

Vector
======

Vectors represent positions and offsets in space.
They are constructed from coordinates with the syntax :scenic:`{X} @ {Y}` (inspired by `Smalltalk <http://stephane.ducasse.free.fr/FreeBooks/BlueBook/Bluebook.pdf>`_); using a length-2 list or tuple (:scenic:`[{X}, {Y}]` or :scenic:`({X}, {Y})`) is also allowed.
By convention, coordinates are in meters, although the semantics of Scenic does not depend on this.
More significantly, the vector syntax is specialized for 2-dimensional space.
The 2D assumption dramatically simplifies much of Scenic’s syntax (particularly that dealing with orientations, as we will see below), while still being adequate for a variety of applications.
Some :term:`world models`, such as that for the :ref:`driving_domain`, define an :prop:`elevation` property which defines a 3D location in combination with Scenic's 2D :prop:`position` property.
A future extension of Scenic will natively support 3D space.

For convenience, instances of `Point` can be used in any context where a vector is expected: so for example if ``P`` is a `Point`, then :scenic:`P offset by (1, 2)` is equivalent to :scenic:`P.position offset by (1, 2)`.


.. _Heading:

Heading
=======
Headings represent orientations in space.
Conveniently, in 2D these can be expressed using a single angle (rather than Euler angles or a quaternion).
Scenic represents headings in radians, measured anticlockwise from North, so that a heading of 0 is due North and a heading of π/2 is due West.
We use the convention that the heading of a local coordinate system is the heading of its Y-axis, so that, for example, the vector :scenic:`-2 @ 3` means 2 meters left and 3 ahead.

For convenience, instances of `OrientedPoint` can be used in any context where a heading is expected: so for example if ``OP`` is an `OrientedPoint`, then :scenic:`relative heading of OP` is equivalent to :scenic:`relative heading of OP.heading`.
Since `OrientedPoint` is a subclass of `Point`, expressions involving two oriented points like :scenic:`OP1 relative to OP2` can be ambiguous: the polymorphic operator :scenic:`relative to` accepts both vectors and headings, and either version could be meant here.
Scenic rejects such expressions as being ambiguous: more explicit syntax like :scenic:`OP1.position relative to OP2` must be used instead.


.. _Vector Field:
.. _VectorField:

Vector Field
============

Vector fields associate an orientation (i.e. a heading) to each point in space.
For example, a vector field could represent the shortest paths to a destination, or the nominal traffic direction on a road (e.g. :obj:`scenic.domains.driving.model.roadDirection`).


.. _Region:

Region
======

Regions represent sets of points in space.
Scenic provides a variety of ways to define regions: rectangles, circular sectors, line segments, polygons, occupancy grids, and explicit lists of points, among others.

Regions can have an associated vector field giving points in the region :term:`preferred orientations`.
For example, a region representing a lane of traffic could have a preferred orientation aligned with the lane, so that we can easily talk about distances along the lane, even if it curves.
Another possible use of preferred orientations is to give the surface of an object normal vectors, so that other objects placed on the surface face outward by default.

The main operations available for use with all regions are the :sampref:`({vector} | {Object}) in {region}` operator to test containment within a region, the :sampref:`visible {region}` operator to get the part of a region which is visible from the :scenic:`ego`, and the :sampref:`(in | on) {region}` specifier to choose a position uniformly at random inside a region.

If you need to perform more complex operations on regions, or are writing a :term:`world model` and need to define your own regions, you will have to work with the :obj:`~scenic.core.regions.Region` class (which regions are instances of) and its subclasses for particular types of regions.
These are summarized below: if you are working on Scenic's internals, see the :mod:`scenic.core.regions` module for full details.

Abstract Regions
----------------

.. autoclass:: scenic.core.regions.Region
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: intersect, intersects, union

Simple Shapes
-------------

Unlike the more complex regions, these simple geometric shapes are allowed to depend on random values: for example, the :term:`visible region` of an `Object` is a `SectorRegion` based at the object's :prop:`position`, which might not be fixed.

.. autoclass:: scenic.core.regions.CircularRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.SectorRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.RectangularRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

Polylines and Polygons
----------------------

These subclasses represent fixed 1D and 2D regions defined by line segments and polygons.

.. autoclass:: scenic.core.regions.PolylineRegion
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: start, end, signedDistanceTo, pointAlongBy, __getitem__, __len__

.. autoclass:: scenic.core.regions.PolygonalRegion
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: boundary

Point Sets and Grids
--------------------

.. autoclass:: scenic.core.regions.PointSetRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.GridRegion
    :noindex:
    :no-members:
