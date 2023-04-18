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
They are constructed from coordinates using a length-3 list or tuple (:scenic:`[{X}, {Y}, {Z}]` or :scenic:`({X}, {Y}, {Z})`. Alternatively, they can be constructed with the syntax :scenic:`{X} @ {Y}` (inspired by `Smalltalk <http://stephane.ducasse.free.fr/FreeBooks/BlueBook/Bluebook.pdf>`_) or a length-2 list or tuple, with an implied z value of 0.
By convention, coordinates are in meters, although the semantics of Scenic does not depend on this.

For convenience, instances of `Point` can be used in any context where a vector is expected: so for example if ``P`` is a `Point`, then :scenic:`P offset by (1, 2)` is equivalent to :scenic:`P.position offset by (1, 2)`.

.. versionchanged:: 3.0

    Vectors are now 3 dimensional.

.. _Heading:

Heading
=======
Headings represent yaw in the global XY plane.
Scenic represents headings in radians, measured anticlockwise from North, so that a heading of 0 is due North and a heading of π/2 is due West.
We use the convention that the heading of a local coordinate system is the heading of its Y-axis, so that, for example, the vector :scenic:`-2 @ 3` means 2 meters left and 3 ahead.

For convenience, instances of `OrientedPoint` can be used in any context where a heading is expected: so for example if ``OP`` is an `OrientedPoint`, then :scenic:`relative heading of OP` is equivalent to :scenic:`relative heading of OP.heading`.
Since `OrientedPoint` is a subclass of `Point`, expressions involving two oriented points like :scenic:`OP1 relative to OP2` can be ambiguous: the polymorphic operator :scenic:`relative to` accepts both vectors and headings, and either version could be meant here.
Scenic rejects such expressions as being ambiguous: more explicit syntax like :scenic:`OP1.position relative to OP2` must be used instead.

.. _Orientation:

Orientation
===========
Orientations represent orientation in 3D space.
Scenic represents orientations internally using quaternions, though for convenience they can be created using Euler angles. Scenic follows the right hand rule with the X,Y,Z order of coordinates. In other words, Euler angles are given as (Yaw, Pitch, Roll), in radians, and applied in that order. To help visualize, one can consider their right hand with fingers extended orthogonally. The index finger points along positive X, the middle finger bends left along positive Y, and the thumb ends up pointing along positive Z. For rotations, align your right thumb with an axis and the way your fingers curl is a positive rotation.

.. versionadded:: 3.0

.. _Vector Field:
.. _VectorField:

Vector Field
============

Vector fields associate an orientation to each point in space.
For example, a vector field could represent the shortest paths to a destination, or the nominal traffic direction on a road (e.g. :obj:`scenic.domains.driving.model.roadDirection`).

.. versionchanged:: 3.0

    Vector fields now return an `Orientation` instead of a `Heading`.

.. _Region:

Region
======

Regions represent sets of points in space.
Scenic provides a variety of ways to define regions in 2D and 3D space: meshes, rectangles, circular sectors, line segments, polygons, occupancy grids, and explicit lists of points, among others.

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

Point Sets and Lines
--------------------

.. autoclass:: scenic.core.regions.PointSetRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.PolylineRegion
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: start, end, signedDistanceTo, pointAlongBy, __getitem__, __len__


.. autoclass:: scenic.core.regions.PathRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. _2D Regions:

2D Regions
----------

2D regions represent a 2D shape parallel to the XY plane, at a certain elevation in space. All 2D regions inherit from `PolygonalRegion`.

Unlike the more `PolygonalRegion`, the simple geometric shapes are allowed to depend on random values: for example, the :term:`visible region` of an `Object` is a `SectorRegion` based at the object's :prop:`position`, which might not be fixed.

Since 2D regions cannot contain an `Object` (which must be 3D), they define a :term:`footprint` for convenient. Footprints are always a `PolygonalFootprintRegion`, which represents a 2D poylgon extruded infinitely in the positive and negative vertical direction. When checking containment of an `Object` in a 2D region, Scenic will atuomatically use the footprint.

.. autoclass:: scenic.core.regions.PolygonalRegion
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: boundary, footprint

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

3D Regions
----------

3D regions represent points in 3D space.

Most 3D regions inherit from a descendant of `MeshRegion` (an abstract class), either `MeshVolumeRegion` or `MeshSurfaceRegion`. These represent the volume (of a watertight mesh) or the surface of a mesh respectively. Various region classes are also provided to create primitive shapes. `MeshVolumeRegion` can be converted to `MeshSurfaceRegion` (and vice versa) using the the ``getSurfaceRegion`` and ``getVolumeRegion`` methods.

PolygonalFootprintRegions represent the :term:`footprint` of a 2D region. See `2D Regions` for more details.

.. autoclass:: scenic.core.regions.MeshRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.MeshVolumeRegion
    :noindex:
    :no-members:
    :members: getSurfaceRegion

.. autoclass:: scenic.core.regions.MeshSurfaceRegion
    :noindex:
    :no-members:
    :members: getVolumeRegion

.. autoclass:: scenic.core.regions.BoxRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.SpheroidRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. autoclass:: scenic.core.regions.PolygonalFootprintRegion
    :noindex:
    :no-show-inheritance:
    :no-members:

.. versionadded::3.0

Niche Regions
-------------

.. autoclass:: scenic.core.regions.GridRegion
    :noindex:
    :no-members:


Shape
=====

Shapes represent the shape of an object. Shapes are automatically converted to unit size and centered. Furthermore it's assumed that the front of the shape is aligned with the y axis.

Shapes can be created with an arbitrary mesh, and several basic shapes are made available. A shape can be created with fixed dimensions or shape, which will set the default values for any `Object` created with that shape. When creating a `MeshShape`, if no dimensions are provided then dimensions will be inferred from the mesh. `MeshShape` also takes an optional ``initial_rotation`` parameter, which will be applied to orient the shape correctly.

.. autoclass:: scenic.core.shapes.MeshShape
    :noindex:
    :no-show-inheritance:

.. autoclass:: scenic.core.shapes.BoxShape
    :noindex:
    :no-show-inheritance:

.. autoclass:: scenic.core.shapes.CylinderShape
    :noindex:
    :no-show-inheritance:

.. autoclass:: scenic.core.shapes.ConeShape
    :noindex:
    :no-show-inheritance:

.. autoclass:: scenic.core.shapes.SpheroidShape
    :noindex:
    :no-show-inheritance:

