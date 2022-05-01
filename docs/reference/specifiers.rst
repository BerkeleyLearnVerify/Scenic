..  _specifiers:

********************
Specifiers Reference
********************

Specifiers are used to define the properties of an object when a Scenic class is :ref:`instantiated <objectCreate>`.
This page describes all the specifiers built into Scenic, and the procedure used to :ref:`resolve <specifier resolution>` a set of specifiers into an assignment of values to properties.

General Specifiers
==================

.. _with {property} {value}:

with *property* *value*
-----------------------
Assigns the given property to the given value.
This is currently the only specifier available for properties other than ``position`` and ``heading``.


Position Specifiers
===================

.. figure:: ../images/Specifier_Figure.png
  :width: 60%
  :figclass: align-center
  :alt: Diagram illustrating several specifiers.

  Illustration of the ``beyond``, ``behind``, and ``offset by`` specifiers.
  Each ``OrientedPoint`` (e.g. ``P``) is shown as a bold arrow.

.. _at {vector}:

at *vector*
-----------
Positions the object at the given global coordinates.

.. _offset by {vector}:

offset by *vector*
------------------
Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined).

.. _offset along {direction} by {vector}:

offset along *direction* by *vector*
------------------------------------
Positions the object at the given coordinates, in a local coordinate system centered at ego and oriented along the given direction (which, if a vector field, is evaluated at ego to obtain a heading).

.. _(left | right) of {vector} [by {scalar}]:

(left | right) of *vector* [by *scalar*]
----------------------------------------
Depends on ``heading`` and ``width``. Without the optional :samp:`by {scalar}`, positions the object immediately to the left/right of the given position; i.e., so that the midpoint of the object’s right/left edge is at that position.
If :samp:`by {scalar}` is used, the object is placed further to the left/right by the given distance.

.. _(ahead of | behind) {vector} [by {scalar}]:

(ahead of | behind) *vector* [by *scalar*]
--------------------------------------------
As above, except placing the object ahead of or behind the given position (so that the midpoint of the object’s back/front edge is at that position); thereby depending on ``heading`` and ``length``.

.. _beyond {vector} by {vector} [from {vector}]:

beyond *vector* by *vector* [from *vector*]
--------------------------------------------
Positions the object at coordinates given by the second vector, in a local coordinate system centered at the first vector and oriented along the line of sight from the third vector (i.e. a heading of 0 in the local coordinate system faces directly away from the first vector).
If no third vector is provided, it is assumed to be the ego.
For example, ``beyond taxi by (0, 3)`` means 3 meters directly behind the taxi as viewed by the camera.

.. _visible [from ({Point} | {OrientedPoint})]:

visible [from (*Point* | *OrientedPoint*)]
------------------------------------------
Positions the object uniformly at random in the visible region of the ego, or of the given Point/OrientedPoint if given.
Visible regions are defined as follows: a `Point` can see out to a certain distance (the ``viewDistance`` property), and an `OrientedPoint` restricts this to the circular sector along its ``heading`` with a certain angle (the ``viewAngle`` property).
A position is then visible if it lies in the visible region; this specifier sets the ``position`` of the object being created (i.e. its center) to be a uniformly-random point in the visible region.

.. _not visible [from ({Point} | {OrientedPoint})]:

not visible [from (Point* | *OrientedPoint*)]
----------------------------------------------
Like :sampref:`visible [from ({Point} | {OrientedPoint})]` except it positions the object uniformly at random in the **non-visible** region of the ego.

.. _(in | on) {region}:

(in | on) *region*
------------------
Positions the object uniformly at random in the given `Region`.
If the Region has a :term:`preferred orientation` (a vector field), also optionally specifies ``heading`` to be equal to that orientation at the object’s ``position``.

.. _(left | right) of ({OrientedPoint} | {Object}) [by {scalar}]:

(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]
------------------------------------------------------------
Positions the object to the left/right of the given `OrientedPoint`, depending on the object’s ``width``.
Also optionally specifies ``heading`` to be the same as that of the OrientedPoint.
If the OrientedPoint is in fact an `Object`, the object being constructed is positioned to the left/right of its left/right edge (i.e. the ``width`` of both objects is taken into account).

.. _(ahead of | behind) ({OrientedPoint} | {Object}) [by {scalar}]:

(ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar*]
---------------------------------------------------------------
As above, except positioning the object ahead of or behind the given OrientedPoint, thereby depending on ``length``.

.. _following {vectorField} [from {vector}] for {scalar}:

following *vectorField* [from *vector* ] for *scalar*
-----------------------------------------------------
Positions the object at a point obtained by following the given vector field for the given distance starting from ego (or the position optionally provided with :samp:`from {vector}`).
Optionally specifies ``heading`` to be the heading of the vector field at the resulting point.

.. note:

  This specifier uses a forward Euler approximation of the continuous vector field.
  The choice of step size can be customized for individual fields: see the documentation
  of `VectorField`. If necessary, you can also call the underlying method
  `VectorField.followFrom`  directly.


Heading Specifiers
==================

.. _facing {heading}:

facing *heading*
----------------
Orients the object along the given heading in global coordinates.

.. _facing {vectorField}:

facing *vectorField*
--------------------
Orients the object along the given vector field at the object’s ``position``.

.. _facing (toward | away from) {vector}:

facing (toward | away from) *vector*
------------------------------------
Orients the object so that it faces toward/away from the given position (thereby depending on the object’s ``position``).

.. _apparently facing {heading} [from {vector}]:

apparently facing *heading* [from *vector*]
--------------------------------------------
Orients the object so that it has the given heading with respect to the line of sight from ego (or the ``from`` vector).
For example, ``apparently facing 90 deg`` orients the object so that the camera views its left side head-on.

.. _specifier resolution:

Specifier Resolution
====================

Specifier resolution in Scenic is non-trivial, as a specifier can specify multiple properties, both non-optionally and optionally.
At a high level, Scenic must check for two error cases: properties that have been non-optionally specified multiple times and properties that have been optionally specified multiple times but not non-optionally specified.
In both cases the value of the property is ambiguous, and Scenic will raise an exception.
Otherwise, the value of each property will be determined by its unique non-optional specifier, unique optional specifier, or its default value, in that order.
If the default value is overridden in a subclass, the new value is used.
Scenic then checks that all dependencies exist and are non-cyclic, before evaluating the specifiers to fix the properties of the object.
