..  _region types:

***********************************
Region Types Reference
***********************************

This page covers the `scenic.core.regions.Region` class and its subclasses; for an introduction to the concept of regions in Scenic and the basic operations available for them, see :ref:`region`.

.. contents:: :local:

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

Since 2D regions cannot contain an `Object` (which must be 3D), they define a :term:`footprint` for convenience.
Footprints are always a `PolygonalFootprintRegion`, which represents a 2D polygon extruded infinitely in the positive and negative vertical direction.
When checking containment of an `Object` in a 2D region, Scenic will atuomatically use the footprint.

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

Most 3D regions inherit from either `MeshVolumeRegion` or `MeshSurfaceRegion`, which represent the volume (of a watertight mesh) and the surface of a mesh respectively. Various region classes are also provided to create primitive shapes. `MeshVolumeRegion` can be converted to `MeshSurfaceRegion` (and vice versa) using the the ``getSurfaceRegion`` and ``getVolumeRegion`` methods.

Mesh regions can use one of two engines for mesh operations: Blender or OpenSCAD. This can be controlled using the ``engine`` parameter, passing ``"blender"`` or ``"scad"`` respectively. Blender is generally more tolerant but can produce unreliable output, such as meshes that have microscopic holes. OpenSCAD is generally more precise, but may crash on certain inputs that it considers ill-defined. By default, Scenic uses Blender internally.

PolygonalFootprintRegions represent the :term:`footprint` of a 2D region. See `2D Regions` for more details.

.. autoclass:: scenic.core.regions.MeshVolumeRegion
    :noindex:
    :no-members:
    :members: getSurfaceRegion, fromFile

.. autoclass:: scenic.core.regions.MeshSurfaceRegion
    :noindex:
    :no-members:
    :members: getVolumeRegion, fromFile

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
