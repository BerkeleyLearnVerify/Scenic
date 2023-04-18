..  _specifiers:

********************
Specifiers Reference
********************

Specifiers are used to define the properties of an object when a Scenic class is :ref:`instantiated <objectCreate>`.
This page describes all the specifiers built into Scenic, and the procedure used to :ref:`resolve <specifier resolution>` a set of specifiers into an assignment of values to properties.

Each specifier assigns values to one or more properties of an object, as a function of the arguments of the specifier and possibly other properties of the object assigned by other specifiers.
For example, the :scenic:`left of {X} by {Y}` specifier assigns the :prop:`position` property of the object being defined so that the object is a distance :scenic:`{Y}` to the left of :scenic:`{X}`: this requires knowing the :prop:`width` of the object first, so we say the :scenic:`left of` specifier *specifies* the :prop:`position` property and *depends* on the :prop:`width` property.

In fact, the :scenic:`left of` specifier also specifies the :prop:`parentOrientation` property (to be the same as :scenic:`{X}`), but it does this with a lower *priority*. Multiple specifiers can specify the same property, but only the specifier that specifies the property with the highest priority is used. If a property is specified multiple times with the same priority, an ambiguity error is raised. In general, a smaller priority value equates to higher priority (e.g. Priority 1 supersedes priority 3).

Certain specifiers can also *modify* already specified values. So called **modifying specifiers** take an already specified value, and manipulate it in some way. They can also specify other values while doing this. Note that no property can be modified twice. The only modifying specifier currently in Scenic is `on {region}`, which can be used either as a standard specifier or a modifying specifier.

The :ref:`specifier resolution` process works out which specifier determines each property of an object, as well as an appropriate order in which to evaluate the specifiers so that dependencies have already been computed when needed.

General Specifiers
==================

.. _with {property} {value}:

with *property* *value*
-----------------------

**Specifies**:

	* :prop:`property` with priority 1

**Dependencies**: None

Assigns the given property to the given value.
This is currently the only specifier available for properties other than :prop:`position` and :prop:`heading`.


Position Specifiers
===================

.. figure:: ../images/Specifier_Figure.png
  :width: 60%
  :figclass: align-center
  :alt: Diagram illustrating several specifiers.

  Illustration of the :scenic:`beyond`, :scenic:`behind`, and :scenic:`offset by` specifiers.
  Each :scenic:`OrientedPoint` (e.g. ``P``) is shown as a bold arrow.

.. _at {vector}:

at *vector*
-----------

**Specifies**:

	* :prop:`position` with priority 1

**Dependencies**: None

Positions the object at the given global coordinates.

.. _in {region}:

in *region*
-----------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None


Positions the object uniformly at random in the given `Region`.
If the Region has a :term:`preferred orientation` (a vector field), also specifies :prop:`parentOrientation` to be equal to that orientation at the object’s :prop:`position`.

.. _contained in {region}:

contained in *region*
---------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`regionContainedIn` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Like `in {region}`, but also enforces that the object be entirely contained in the given `Region`.

.. _on {region}:

on *region*
-----------

**Specifies**:

	* :prop:`position` with priority 1; **modifies** existing value
	* :prop:`parentOrientation` with priority 2

**Dependencies**: :prop:`baseOffset` • :prop:`contactTolerance` • :prop:`onDirection`

Positions the base of the object uniformly at random in the given `Region`, offset by :prop:`contactTolerance` (to avoid a collision). The base of the object is determined by adding the object's :prop:`position` to its :prop:`baseOffset`.

If :prop:`position` is already set by another specifier, then position is instead modified. This is done by finding the closest point in the given `Region` along :prop:`onDirection` or its negation, and setting the position to that point.

If the Region has a :term:`preferred orientation` (a vector field), :prop:`parentOrientation` is specified to be equal to that orientation at the object’s :prop:`position`.

.. note::

	:prop:`parentOrientation` is specified whether or not this specifier is modifying. It is also specified with priority 2 (higher than all other specifiers for :prop:`parentOrientation`). This is helpful for ensuring that an object is always aligned correctly, for example when ``on`` is being used to place or project onto the surface of a `MeshSurfaceRegion`.

.. _offset by {vector}:

offset by *vector*
------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined). Also specifies :prop:`parentOrientation` to be equal to the ego's orientation.

.. _offset along {direction} by {vector}:

offset along *direction* by *vector*
------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at the given coordinates, in a local coordinate system centered at ego. Also specifies :prop:`parentOrientation` to be equal to the ego's orientation.

.. _beyond {vector} by {vector} [from {vector | OrientedPoint}]:

beyond *vector* by *vector* [from (*vector* | *OrientedPoint*)]
---------------------------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at coordinates given by the second vector, in a local coordinate system centered at the first vector and oriented along the line of sight from the third vector (i.e. an orientation of (0,0,0) in the local coordinate system faces directly away from the first vector).
If no third vector is provided, it is assumed to be the ego. :prop:`parentOrientation` is inherited from the third value if an `OrientedPoint` is provided, and otherwise the global coordinate system is used.
For example, :scenic:`beyond taxi by (0, 3, 0)` means 3 meters directly behind the taxi as viewed by the ego.

.. _visible [from ({Point} | {OrientedPoint})]:
.. _visible_spec:

visible [from (*Point* | *OrientedPoint*)]
------------------------------------------

**Specifies**:

	* :prop:`_observingEntity` with priority 1
	* :prop:`position` with priority 3

**Dependencies**: None

Ensures this object is visible from the ego or given `Point`/`OrientedPoint`.

Can also position the object uniformly at random in the :term:`visible region` of the ego, or of the given Point/OrientedPoint if given. More precisely, this specifier can set the :prop:`position` of the object being created (i.e. its center) to be a uniformly-random point in the visible region. (The position set by this specifier is therefore slightly stricter than a requirement that the ego :sampref:`can see` the object: the specifier makes the *center* visible, while the :sampref:`can see` condition will be satisfied if the center is not visible but some other part of the object is visible.)

.. _not visible [from ({Point} | {OrientedPoint})]:

not visible [from (*Point* | *OrientedPoint*)]
----------------------------------------------

**Specifies**:

	* :prop:`_nonObservingEntity` with priority 1
	* :prop:`position` with priority 3

**Dependencies**: :prop:`regionContainedIn`

Ensures that this object is *not* visible from the ego or given `Point`/`OrientedPoint`.

Similar to :sampref:`visible [from ({Point} | {OrientedPoint})]`, this specifier can position the object uniformly at random in the **non-visible** region of the ego.
Depends on :prop:`regionContainedIn`, in order to restrict the non-visible region to the :term:`container` of the object being created, which is hopefully a bounded region (if the non-visible region is unbounded, it cannot be uniformly sampled from and an error will be raised).

.. _(left | right) of {vector} [by {scalar}]:

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
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`

.. _(left | right) of {Object} [by {scalar}]:

(left | right) of *Object* [by *scalar*]
----------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`width` • :prop:`contactTolerance`

Positions the object to the left/right of the given `Object`. This accounts for both objects' dimensions, placing them so that the distance between bounding boxes is exactly the desired scalar distance (or :prop:`contactTolerance` without :scenic:`by {scalar}`).
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`

.. _(ahead of | behind) ({vector} | {Point}) [by {scalar}]:

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

Positions the object to the ahead of/behind the given `OrientedPoint`.
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`

.. _(ahead of | behind) {Object} [by {scalar}]:

(ahead of | behind) *Object* [by *scalar*]
------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`length` • :prop:`contactTolerance`

Positions the object ahead of/behind the given `Object`.This accounts for both objects' dimensions, placing them so that the distance between bounding boxes is exactly the desired scalar distance (or :prop:`contactTolerance` without :scenic:`by {scalar}`).
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`

.. _(above | below) {vector} [by {scalar}]:

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

Positions the object to the above/below the given `OrientedPoint`.
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`

.. _(above | below) {Object} [by {scalar}]:

(above | below) *Object* [by *scalar*]
--------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: :prop:`height` • :prop:`contactTolerance`

Positions the object above/below the given `Object`. This accounts for both objects' dimensions, placing them so that the distance between bounding boxes is exactly the desired scalar distance (or :prop:`contactTolerance` without :scenic:`by {scalar}`).
Also inherits :prop:`parentOrientation` from the given `OrientedPoint`

.. _following {vectorField} [from {vector}] for {scalar}:

following *vectorField* [from *vector*] for *scalar*
----------------------------------------------------

**Specifies**:

	* :prop:`position` with priority 1
	* :prop:`parentOrientation` with priority 3

**Dependencies**: None

Positions the object at a point obtained by following the given vector field for the given distance starting from ego (or the position optionally provided with :scenic:`from {vector}`).
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


Sets the object's :prop:`yaw`, :prop:`pitch`, and :prop:`roll` so that its orientation in global coordinates is equal to the given orientation. If a tuple of floats is given, it is interpreted as a tuple of Euler angles in radians and converted to an orientation.

.. _facing {vectorField}:

facing *vectorField*
--------------------

**Specifies**:

	* :prop:`yaw` with priority 1
	* :prop:`pitch` with priority 1
	* :prop:`roll` with priority 1

**Dependencies**: :prop:`position` • :prop:`parentOrientation`

Sets the object's :prop:`yaw`, :prop:`pitch`, and :prop:`roll` so that its orientation in global coordinates is equal to orientation provided by the given vector field at the object’s :prop:`position`.

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

Sets the object's :prop:`yaw` **and** :prop:`pitch` so that it faces directly toward/away from the given position (thereby depending on the object’s :prop:`position`).


.. _apparently facing {heading} [from {vector}]:

apparently facing *heading* [from *vector*]
-------------------------------------------

**Specifies**:

	* :prop:`yaw` with priority 1

**Dependencies**: :prop:`position` • :prop:`parentOrientation`

Sets the :prop:`yaw` of the object so that it has the given heading with respect to the line of sight from ego (or the ``from`` vector).
For example, :scenic:`apparently facing 90 deg` orients the object so that the camera views its left side head-on.

.. _specifier resolution:

Specifier Resolution
====================

Specifier resolution is the process of determining, given the set of specifiers used to define an object, which properties each specifier should determine and what order to evaluate the specifiers in.
As each specifier can specify multiple properties, both non-optionally and optionally, and can depend on the results of other specifiers, this process is somewhat non-trivial.
Assuming there are no cyclic dependencies or conflicts, the process will conclude with each property being determined by its unique non-optional specifier if one exists; otherwise its unique optional specifier if one exists; or finally by its default value if no specifiers apply at all (with default values from subclasses overriding those in superclasses).

The full procedure, given a set of specifiers *S* used to define an instance of class *C*, works as follows:

1. If a property is specified at the same priority level by mutiple specifiers in *S*, an ambiguity error is raised.
2. The set of properties *P* for the new object is found by combining the properties specified by all members of *S* with the properties inherited from the class *C*.
3. Default value specifiers from *C* (or if not overridden, from its superclasses) are added to *S* as needed so that each property in *P* is paired with a unique specifier in *S* specifying it.
4. The dependency graph of the specifiers *S* is constructed. If it is cyclic, an error is raised.
5. The graph is topologically sorted and the specifiers are evaluated in this order to determine the values of all properties *P* of the new object.
6. Modifying specifiers are evaluated to modify relevant properties of the object.
