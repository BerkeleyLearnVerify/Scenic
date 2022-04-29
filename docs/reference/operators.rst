..  _operators:

*******************
Operators Reference
*******************

.. figure:: ../images/Operator_Figure.png
  :width: 70%
  :figclass: align-center
  :alt: Diagram illustrating several operators.

  Illustration of several operators.
  Each ``OrientedPoint`` (e.g. ``P``) is shown as a bold arrow.


Scalar Operators
=================

.. _relative heading of *heading* [from *heading*]:

relative heading of *heading* [from *heading*]
----------------------------------------------
The relative heading of the given heading with respect to ego (or the heading provided with the optional from heading)

.. _apparent heading of *OrientedPoint* [from *vector*]:

apparent heading of *OrientedPoint* [from *vector*]
---------------------------------------------------
The apparent heading of the OrientedPoint, with respect to the line of sight from ego (or the position provided with the optional from vector)

.. _distance [from *vector* ] to *vector*:

distance [from *vector* ] to *vector*
-------------------------------------
The distance to the given position from ego (or the position provided with the optional from vector)

.. _angle [from *vector* ] to *vector*:

angle [from *vector* ] to *vector*
----------------------------------
The heading to the given position from ego (or the position provided with the optional from vector ). For example, if angle to taxi is zero, then taxi is due North of ego


Boolean Operators
==================

.. _(*Point* | *OrientedPoint*) can see (*vector* | *Object*):

(*Point* | *OrientedPoint*) can see (*vector* | *Object*)
---------------------------------------------------------
Whether or not a position or Object is visible from a Point or OrientedPoint. Visible regions are defined as follows: a Point can see out to a certain distance, and an OrientedPoint restricts this to the circular sector along its heading with a certain angle. A position is then visible if it lies in the visible region, and an Object is visible if its bounding box intersects the visible region. Technically, Scenic only checks that a corner of the object is visible, which could result in the side of a large object being visible but Scenic not counting it as so. Note that Scenic’s visibility model does not take into account occlusion, although this would be straightforward to add.

.. _(*vector* | *Object*) in *region*:

(*vector* | *Object*) in *region*
----------------------------------
Whether a position or Object lies in the region; for the latter, the Object’s bounding box must be contained in the region. This allows us to use the predicate in two ways


Heading Operators
=================

.. _*scalar* deg:

*scalar* deg
------------
The given heading, interpreted as being in degrees. For example 90 deg evaluates to π/2

.. _*vectorField* at *vector*:

*vectorField* at *vector*
-------------------------
The heading specified by the vector field at the given position

.. _*direction* relative to *direction*:

*direction* relative to *direction*
------------------------------------
The first direction, interpreted as an offset relative to the second direction. For example, :samp:`-5 deg relative to 90 deg` is simply 85 degrees. If either direction is a vector field, then this operator yields an expression depending on the position property of the object being specified


Vector Operators
================

.. _*vector* (relative to | offset by) *vector*:

*vector* (relative to | offset by) *vector*
--------------------------------------------
The first vector, interpreted as an offset relative to the second vector (or vice versa). For example, :samp:`5@5 relative to 100@200` is :samp:`105@205`. Note that this polymorphic operator has a specialized version for instances of OrientedPoint, defined below (so for example :samp:`-3@0 relative to taxi` will not use this vector version, even though the Object taxi can be coerced to a vector)

.. _*vector* offset along *direction* by *vector*:

*vector* offset along *direction* by *vector*
----------------------------------------------
The second vector, interpreted in a local coordinate system centered at the first vector and oriented along the given direction (which, if a vector field, is evaluated at the first vector to obtain a heading)

Region Operators
================

.. _visible *region*:

visible *region*
----------------
The part of the given region visible from ego

OrientedPoint Operators
=======================

.. _*vector* relative to *OrientedPoint*:

*vector* relative to *OrientedPoint*
-------------------------------------
The given vector, interpreted in the local coordinate system of the OrientedPoint. So for example :samp:`1 @ 2 relative to ego` is 1 meter to the right and 2 meters ahead of ego.

.. _*OrientedPoint* offset by *vector*:

*OrientedPoint* offset by *vector*
----------------------------------
Equivalent to vector relative to OrientedPoint above

.. _(front | back | left | right) of *Object*:

(front | back | left | right) of *Object*
-----------------------------------------
The midpoint of the corresponding edge of the bounding box of the Object, oriented along its heading

.. _(front | back) (left | right) of *Object*:


(front | back) (left | right) of *Object*
-----------------------------------------
The corresponding corner of the Object’s bounding box, also oriented along its heading
