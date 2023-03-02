..  _operators:

*******************
Operators Reference
*******************

.. figure:: ../images/Operator_Figure.png
  :width: 70%
  :figclass: align-center
  :alt: Diagram illustrating several operators.

  Illustration of several operators.
  Each :scenic:`OrientedPoint` (e.g. ``P``) is shown as a bold arrow.


Scalar Operators
=================

.. _relative heading of {heading} [from {heading}]:

relative heading of *heading* [from *heading*]
----------------------------------------------
The relative heading of the given heading with respect to ego (or the heading provided with the optional from heading)

.. _apparent heading of {OrientedPoint} [from {vector}]:

apparent heading of *OrientedPoint* [from *vector*]
---------------------------------------------------
The apparent heading of the OrientedPoint, with respect to the line of sight from ego (or the position provided with the optional from vector)

.. _distance [from {vector}] to {vector}:

distance [from *vector*] to *vector*
-------------------------------------
The distance to the given position from ego (or the position provided with the optional from vector)

.. _angle [from {vector}] to {vector}:

angle [from *vector* ] to *vector*
----------------------------------
The heading to the given position from ego (or the position provided with the optional from vector ). For example, if angle to taxi is zero, then taxi is due North of ego


Boolean Operators
==================

.. _({Point} | {OrientedPoint}) can see ({vector} | {Object}):
.. _can see:

(*Point* | *OrientedPoint*) can see (*vector* | *Object*)
---------------------------------------------------------
Whether or not a position or `Object` is visible from a `Point` or `OrientedPoint`.
Visible regions are defined as follows: a `Point` can see out to a certain distance, and an `OrientedPoint` restricts this to the circular sector along its heading with a certain angle.
A position is then visible if it lies in the visible region, and an `Object` is visible if its bounding box intersects the visible region.

.. note::

  Technically, Scenic only checks that a corner of the object is visible, which could result in the side of a large object being visible but Scenic not counting it as so.
  Scenic’s visibility model also does not take into account occlusion, although this would be straightforward to add.

.. _({vector} | {Object}) in {region}:

(*vector* | *Object*) in *region*
----------------------------------
Whether a position or `Object` lies in the `Region`; for the latter, the object’s bounding box must be completely contained in the region.


Heading Operators
=================

.. _{scalar} deg:

*scalar* deg
------------
The given heading, interpreted as being in degrees. For example 90 deg evaluates to π/2

.. _{vectorField} at {vector}:

*vectorField* at *vector*
-------------------------
The heading specified by the vector field at the given position

.. _{direction} relative to {direction}:

*direction* relative to *direction*
------------------------------------
The first direction, interpreted as an offset relative to the second direction. For example, :scenic:`-5 deg relative to 90 deg` is simply 85 degrees. If either direction is a vector field, then this operator yields an expression depending on the :prop:`position` property of the object being specified.


Vector Operators
================

.. _{vector} (relative to | offset by) {vector}:

*vector* (relative to | offset by) *vector*
--------------------------------------------
The first vector, interpreted as an offset relative to the second vector (or vice versa).
For example, :scenic:`(5, 5) relative to (100, 200)` is :scenic:`(105, 205)`.
Note that this polymorphic operator has a specialized version for instances of `OrientedPoint`, defined :ref:`below <{vector} relative to {OrientedPoint}>`: so for example :scenic:`(-3, 0) relative to taxi` will not use the version of this operator for vectors (even though the `Object` taxi can be coerced to a vector).

.. _{vector} offset along {direction} by {vector}:

*vector* offset along *direction* by *vector*
----------------------------------------------
The second vector, interpreted in a local coordinate system centered at the first vector and oriented along the given direction (which, if a vector field, is evaluated at the first vector to obtain a heading)

Region Operators
================

.. _visible {region}:

visible *region*
----------------
The part of the given region which is visible from the ego object (i.e. the intersection of the given region with the :term:`visible region` of the ego).

.. _not visible {region}:

not visible *region*
--------------------
The part of the given region which is *not* visible from the ego object (as above, based on the ego's :term:`visible region`).

OrientedPoint Operators
=======================

.. _{vector} relative to {OrientedPoint}:

*vector* relative to *OrientedPoint*
-------------------------------------
The given vector, interpreted in the local coordinate system of the OrientedPoint. So for example :scenic:`(1, 2) relative to ego` is 1 meter to the right and 2 meters ahead of ego.

.. _{OrientedPoint} offset by {vector}:

*OrientedPoint* offset by *vector*
----------------------------------
Equivalent to :scenic:`{vector} relative to {OrientedPoint}` above

.. _(front | back | left | right) of {Object}:

(front | back | left | right) of *Object*
-----------------------------------------
The midpoint of the corresponding edge of the bounding box of the `Object`, oriented along its heading/

.. _(front | back) (left | right) of {Object}:


(front | back) (left | right) of *Object*
-----------------------------------------
The corresponding corner of the Object’s bounding box, also oriented along its heading.
