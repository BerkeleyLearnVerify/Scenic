..  _specifiers:

********************
Specifiers Reference
********************

Specifiers are used to define the properties of an object when a Scenic class is :ref:`instantiated <objectCreate>`.
This page describes all the specifiers built into Scenic, and the procedure used to :ref:`resolve <specifier resolution>` a set of specifiers into an assignment of values to properties.

Each specifier assigns values to one or more properties of an object, as a function of the arguments of the specifier and possibly other properties of the object assigned by other specifiers.
For example, the :specifier:`left of {X} by {Y}` specifier assigns the :prop:`position` property of the object being defined so that the object is a distance :scenic:`{Y}` to the left of :scenic:`{X}`: this requires knowing the :prop:`width` of the object first, so we say the :specifier:`left of` specifier **specifies** the :prop:`position` property and **depends** on the :prop:`width` property.

In fact, the :specifier:`left of` specifier also specifies the :prop:`parentOrientation` property (to be the :prop:`orientation` of :scenic:`{X}`), but it does this with a lower **priority**.
Multiple specifiers can specify the same property, but only the specifier that specifies the property with the highest priority is used.
If a property is specified multiple times with the same priority, an ambiguity error is raised.
We represent priorities as integers, with priority 1 being the highest and larger integers having progressively lower priorities (e.g. priority 2 supersedes priority 3).
When a specifier specifies a property with a priority lower than 1, we say it **optionally** specifies the property, since it can be overridden (for example using the :specifier:`with` specifier), whereas a specifier specifying the property with priority 1 cannot be overridden.

Certain specifiers can also *modify* already-specified values.
These **modifying specifiers** do not cause an ambiguity error as above if another specifier specifies the same property with the same priority: they take the already-specified value and manipulate it in some way (potentially also specifying other properties as usual).
Note that no property can be modified twice.
The only modifying specifier currently in Scenic is :specifier:`on {region}`, which can be used either as a standard specifier or a modifying specifier (the modifying version projects the already-specified position onto the given region -- see below).

The :ref:`specifier resolution` process works out which specifier determines each property of an object, as well as an appropriate order in which to evaluate the specifiers so that dependencies have already been computed when needed.

General Specifiers
==================

.. _with {property} {value}:

with *property* *value*
-----------------------

**Specifies**:

	* the given property, with priority 1

**Dependencies**: None

Assigns the given property to the given value.
This is currently the only specifier available for properties other than :prop:`position` and :prop:`orientation`.


Position Specifiers
===================

.. figure:: ../images/Specifier_Figure.png
  :width: 60%
  :figclass: align-center
  :alt: Diagram illustrating several specifiers.

  Illustration of the :specifier:`beyond`, :specifier:`behind`, and :specifier:`offset by` specifiers.
  Each :scenic:`OrientedPoint` (e.g. ``P``) is shown as a bold arrow.

.. _at {vector}:

at *vector*
-----------

**Specifies**:

	* :prop:`position` with priority 1

**Dependencies**: None

Positions the object at the given global coordinates.

.. _in {region}:
.. _in:

in *region*
-----------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3 (if the region has a :term:`preferred orientation`)

**Dependencies**: None


Positions the object uniformly at random in the given `Region`.
If the Region has a :term:`preferred orientation` (a vector field), also specifies :prop:`parentOrientation` to be equal to that orientation at the object’s :prop:`position`.

.. _contained in {region}:

contained in *region*
---------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`regionContainedIn` with priority 1
	* :prop:`parentOrientation` with priority 3 (if the region has a :term:`preferred orientation`)

**Dependencies**: None

Like :sampref:`in {region}`, but also enforces that the object be entirely contained in the given `Region`.

.. _on {region}:
.. _on:

on *region*
-----------

**Specifies**:

	* :prop:`position` with priority 1; **modifies** existing value, if any
	* :prop:`parentOrientation` with priority 2 (if the region has a :term:`preferred orientation`)

**Dependencies**: :prop:`baseOffset` • :prop:`contactTolerance` • :prop:`onDirection`

If :prop:`position` is not already specified with priority 1, positions the *base* of the object uniformly at random in the given `Region`, offset by :prop:`contactTolerance` (to avoid a collision).
The base of the object is determined by adding the object's :prop:`baseOffset` to its :prop:`position`.

Note that while :specifier:`on` can be used with `Region`, `Object` and `Vector`, it cannot be used with a distribution containing anything other than `Region`. When used with an object the base of the object being placed is placed on the target object's `onSurface` and when used with a vector the base of the object being placed is set to that position.

If instead :prop:`position` has already been specified with priority 1, then its value is modified by projecting it onto the given region.
More precisely, we find the closest point in the region along :prop:`onDirection` (or its negation [1]_), and place the base of the object at that point. If :prop:`onDirection` is not specified, a default value is inferred from the region. A region can either specify a default value to be used, or for volumes straight up is used and for surfaces the mean of the face normal values is used (weighted by the area of the faces).

If the region has a :term:`preferred orientation` (a vector field), :prop:`parentOrientation` is specified to be equal to that orientation at the object’s :prop:`position` (whether or not this specifier is being used as a modifying specifier).
Note that this is done with higher priority than all other specifiers which optionally specify :prop:`parentOrientation`, and in particular the :specifier:`ahead of` specifier and its variants: therefore the code :scenic:`new Object ahead of taxi by 100, on road` aligns the new object with the road at the point 100 m ahead of the taxi rather than with the taxi itself (while also using projection to ensure the new object is on the surface of the road rather than under or over it if the road isn't flat).

.. [1] This allows for natural projection even when an object is below the desired surface, such as placing a car, ahead of another car, on an uphill road.

.. _offset by {vector}:

offset by *vector*
------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at the given coordinates in the local coordinate system of :scenic:`ego` (which must already be defined).
Also specifies :prop:`parentOrientation` to be equal to the ego's orientation.

.. versionadded:: 3.0
	:specifier:`offset by` now specifies :prop:`parentOrientation`, whereas previously it did *not* optionally specify :prop:`heading`.

.. _offset along {direction} by {vector}:

offset along *direction* by *vector*
------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at the given coordinates in a local coordinate system centered at :scenic:`ego` and oriented along the given direction (which can be a `heading`, an `orientation`, or a `vector field`).
Also specifies :prop:`parentOrientation` to be equal to the ego's orientation.

.. _beyond {vector} by ({vector} | {scalar}) [from ({vector} | {OrientedPoint})]:
.. _beyond:

beyond *vector* by (*vector* | *scalar*) [from (*vector* | *OrientedPoint*)]
----------------------------------------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at coordinates given by the second vector, in a local coordinate system centered at the first vector and oriented along the line of sight from the third vector (i.e. an orientation of :scenic:`(0,0,0)` in the local coordinate system faces directly away from the third vector).
If the second argument is a scalar :scenic:`{D}` instead of a vector, it is interpreted as the vector :scenic:`(0, {D}, 0)`: thus :specifier:`beyond {X} by {D} from {Y}` places the new object a distance of :scenic:`{D}` behind :scenic:`{X}` from the perspective of :scenic:`{Y}`.
If no third argument is provided, it is assumed to be the :scenic:`ego`.

The value of :prop:`parentOrientation` is specified to be the orientation of the third argument if it is an `OrientedPoint` (including `Objects` such as :scenic:`ego`); otherwise the global coordinate system is used.
For example, :specifier:`beyond taxi by (1, 3, 0)` means 3 meters behind the taxi and one meter to the right as viewed by the :scenic:`ego`.

.. _visible [from ({Point} | {OrientedPoint})]:
.. _visible_spec:

visible [from (*Point* | *OrientedPoint*)]
------------------------------------------

**Specifies**:

	* :prop:`position` with priority 3
	* also adds a requirement (see below)

**Dependencies**: None

Requires that this object is visible from the :scenic:`ego` or the given `Point`/`OrientedPoint`. See the :ref:`Visibility System <visibility>` reference for a discussion of the visibility model.

Also optionally specifies :prop:`position` to be a uniformly random point in the :term:`visible region` of the ego, or of the given Point/OrientedPoint if given.
Note that the position set by this specifier is slightly stricter than simply adding a requirement that the ego :keyword:`can see` the object: the specifier makes the *center* of the object (its :prop:`position`) visible, while the :keyword:`can see` condition will be satisfied even if the center is not visible as long as some other part of the object is visible.

.. _not visible [from ({Point} | {OrientedPoint})]:

not visible [from (*Point* | *OrientedPoint*)]
----------------------------------------------

**Specifies**:

	* :prop:`position` with priority 3
	* also adds a requirement (see below)

**Dependencies**: :prop:`regionContainedIn`

Requires that this object is *not* visible from the ego or the given `Point`/`OrientedPoint`.

Similarly to :sampref:`visible [from ({Point} | {OrientedPoint})]`, this specifier can position the object uniformly at random in the *non-visible* region of the ego.
However, it depends on :prop:`regionContainedIn`, in order to restrict the non-visible region to the :term:`container` of the object being created, which is hopefully a bounded region (if the non-visible region is unbounded, it cannot be uniformly sampled from and an error will be raised).

.. _(left | right) of {vector} [by {scalar}]:
.. _left of:
.. _right of:

(left | right) of (*vector*) [by *scalar*]
------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1

**Dependencies**: :prop:`width` • :prop:`orientation`


Without the optional :scenic:`by {scalar}`, positions the object immediately to the left/right of the given position; i.e., so that the midpoint of the right/left side of the object's bounding box is at that position.
If :scenic:`by {scalar}` is used, the object is placed further to the left/right by the given distance.

.. _(left | right) of {OrientedPoint} [by {scalar}]:

(left | right) of *OrientedPoint* [by *scalar*]
-----------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`width`

Positions the object to the left/right of the given `OrientedPoint`.
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`.

.. _(left | right) of {Object} [by {scalar}]:
.. _left of {Object}:

(left | right) of *Object* [by *scalar*]
----------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`width` • :prop:`contactTolerance`

Positions the object to the left/right of the given `Object`.
This accounts for both objects' dimensions, placing them so that the distance between their bounding boxes is exactly the desired scalar distance (or :prop:`contactTolerance` if :scenic:`by {scalar}` is not used).
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`.

.. _(ahead of | behind) ({vector} | {Point}) [by {scalar}]:
.. _ahead of:
.. _behind:

(ahead of | behind) *vector* [by *scalar*]
------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1

**Dependencies**: :prop:`length` • :prop:`orientation`


Without the optional :scenic:`by {scalar}`, positions the object immediately ahead of/behind the given position; i.e., so that the midpoint of the front/back side of the object’s bounding box is at that position.
If :scenic:`by {scalar}` is used, the object is placed further ahead/behind by the given distance.

.. _(ahead of | behind) {OrientedPoint} [by {scalar}]:

(ahead of | behind) *OrientedPoint* [by *scalar*]
-------------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`length`

Positions the object ahead of/behind the given `OrientedPoint`.
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`.

.. _(ahead of | behind) {Object} [by {scalar}]:

(ahead of | behind) *Object* [by *scalar*]
------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`length` • :prop:`contactTolerance`

Positions the object ahead of/behind the given `Object`.
This accounts for both objects' dimensions, placing them so that the distance between their bounding boxes is exactly the desired scalar distance (or :prop:`contactTolerance` if :scenic:`by {scalar}` is not used).
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`.

.. _(above | below) {vector} [by {scalar}]:
.. _above:
.. _below:

(above | below) *vector* [by *scalar*]
--------------------------------------

**Specifies**:

	* :prop:`position` with priority 1

**Dependencies**: :prop:`height` • :prop:`orientation`


Without the optional :scenic:`by {scalar}`, positions the object immediately above/below the given position; i.e., so that the midpoint of the top/bottom side of the object’s bounding box is at that position.
If :scenic:`by {scalar}` is used, the object is placed further above/below by the given distance.

.. _(above | below) {OrientedPoint} [by {scalar}]:

(above | below) *OrientedPoint* [by *scalar*]
---------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`height`

Positions the object above/below the given `OrientedPoint`.
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`.

.. _(above | below) {Object} [by {scalar}]:

(above | below) *Object* [by *scalar*]
--------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`height` • :prop:`contactTolerance`

Positions the object above/below the given `Object`.
This accounts for both objects' dimensions, placing them so that the distance between their bounding boxes is exactly the desired scalar distance (or :prop:`contactTolerance` if :scenic:`by {scalar}` is not used).
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`.

.. _following {vectorField} [from {vector}] for {scalar}:

following *vectorField* [from *vector*] for *scalar*
----------------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at a point obtained by following the given `vector field` for the given distance starting from :scenic:`ego` (or the position optionally provided with :scenic:`from {vector}`).
Specifies :prop:`parentOrientation` to be the orientation of the vector field at the resulting point.

.. note::

  This specifier uses a forward Euler approximation of the continuous vector field.
  The choice of step size can be customized for individual fields: see the documentation
  of `VectorField`. If necessary, you can also call the underlying method
  `VectorField.followFrom`  directly.


Orientation Specifiers
======================

.. _facing {orientation}:

facing *orientation*
--------------------

**Specifies**:

	* :prop:`yaw` with priority 1
	* :prop:`pitch` with priority 1
	* :prop:`roll` with priority 1

**Dependencies**: :prop:`parentOrientation`


Sets the object's :prop:`yaw`, :prop:`pitch`, and :prop:`roll` so that its orientation in global coordinates is equal to the given orientation.
If a single scalar is given, it is interpreted as a `heading`: so for example :specifier:`facing 45 deg` orients the object in the XY plane, facing northwest.
If a triple of scalars is given, it is interpreted as a triple of global Euler angles: so for example :specifier:`facing (45 deg, 90 deg, 0)` would orient the object to face northwest as above but then apply a 90° pitch upwards.

.. _facing {vectorField}:

facing *vectorField*
--------------------

**Specifies**:

	* :prop:`yaw` with priority 1
	* :prop:`pitch` with priority 1
	* :prop:`roll` with priority 1

**Dependencies**: :prop:`position` • :prop:`parentOrientation`

Sets the object's :prop:`yaw`, :prop:`pitch`, and :prop:`roll` so that its orientation in global coordinates is equal to the orientation provided by the given `vector field` at the object’s :prop:`position`.

.. _facing (toward | away from) {vector}:

facing (toward | away from) *vector*
------------------------------------

**Specifies**:

	* :prop:`yaw` with priority 1

**Dependencies**: :prop:`position` • :prop:`parentOrientation`

Sets the object's :prop:`yaw` so that it faces toward/away from the given position (thereby depending on the object’s :prop:`position`).

.. _facing directly (toward | away from) {vector}:

facing directly (toward | away from) *vector*
---------------------------------------------

**Specifies**:

	* :prop:`yaw` with priority 1
	* :prop:`pitch` with priority 1

**Dependencies**: :prop:`position` • :prop:`parentOrientation`

Sets the object's :prop:`yaw` *and* :prop:`pitch` so that it faces directly toward/away from the given position (thereby depending on the object’s :prop:`position`).


.. _apparently facing {heading} [from {vector}]:

apparently facing *heading* [from *vector*]
-------------------------------------------

**Specifies**:

	* :prop:`yaw` with priority 1

**Dependencies**: :prop:`position` • :prop:`parentOrientation`

Sets the :prop:`yaw` of the object so that it has the given heading with respect to the line of sight from :scenic:`ego` (or the ``from`` vector).
For example, if the :scenic:`ego` is in the XY plane, then :specifier:`apparently facing 90 deg` orients the new object so that the ego's camera views its left side head-on.

.. _specifier resolution:

Specifier Resolution
====================

Specifier resolution is the process of determining, given the set of specifiers used to define an object, which properties each specifier should determine and what order to evaluate the specifiers in.
As each specifier can specify multiple properties with various priorities, and can depend on the results of other specifiers, this process is somewhat non-trivial.
Assuming there are no cyclic dependencies or conflicts, the process will conclude with each property being determined by its unique highest-priority specifier if one exists (possibly modified by a modifying specifier), and otherwise by its default value, with default values from subclasses overriding those in superclasses.

The full procedure, given a set of specifiers *S* used to define an instance of class *C*, works as follows:

1. If a property is specified at the same priority level by multiple specifiers in *S*, an ambiguity error is raised.
2. The set of properties *P* for the new object is found by combining the properties specified by all members of *S* with the properties inherited from the class *C*.
3. Default value specifiers from *C* (or if not overridden, from its superclasses) are added to *S* as needed so that each property in *P* is paired with a unique non-modifying specifier in *S* specifying it (taking the highest-priority specifier, if there are multiple), plus up to one modifying specifier modifying it.
4. The dependency graph of the specifiers *S* is constructed (with edges from each specifier to the others which depend on its results). If it is cyclic, an error is raised.
5. The graph is topologically sorted and the specifiers are evaluated in this order to determine the values of all properties *P* of the new object.
