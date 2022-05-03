..  _syntax_guide:

Syntax Guide
============

This page summarizes the syntax of Scenic (excluding syntax inherited from Python).
For more details, click the links for individual language constructs to go to the corresponding reference section.


Primitive Data Types
--------------------
============================= ==================================================================
`Booleans <Boolean>`          expressing truth values
`Scalars <Scalar>`            representing distances, angles, etc. as floating-point numbers
`Vectors <Vector>`            representing positions and offsets in space
`Headings <Heading>`   		    representing orientations in space
`Vector Fields <VectorField>` associating an orientation (i.e. a heading) to each point in space
`Regions <Region>`            representing sets of points in space
============================= ==================================================================


Distributions
-------------
================================================================ ==================================
:sampref:`Range({low}, {high})`                                  uniformly-distributed real number in the interval
:sampref:`DiscreteRange({low}, {high})`                          uniformly-distributed integer in the (fixed) interval
:sampref:`Normal({mean}, {stdDev})`                              normal distribution with the given mean and standard deviation
:sampref:`TruncatedNormal({mean}, {stdDev}, {low}, {high})`      normal distribution truncated to the given window
:sampref:`Uniform({value}, {...})`                               uniform over a finite set of values
:sampref:`Discrete(\{{value}: {weight}, {...}\})<DiscreteDistr>` discrete with given values and weights
:sampref:`Point (in | on) {region} <uniform_in_region>`                 uniformly-distributed `Point` in a region
================================================================ ==================================

Statements
----------

Compound Statements
+++++++++++++++++++

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Syntax
     - Meaning
   * - :sampref:`class {name}[({superclass})]: <classDef>`
     - Defines a Scenic class.
   * - :sampref:`behavior {name}({arguments}): <behaviorDef>`
     - Defines a :term:`dynamic behavior`.
   * - :sampref:`monitor {name}: <monitorDef>`
     - Defines a :term:`monitor`.
   * - :sampref:`scenario {name}({arguments}): <modularScenarioDef>`
     - Defines a :term:`modular scenario`.
   * - :sampref:`try: {...} interrupt when {boolean}:<tryInterruptStmt>`
     - Run code with interrupts inside a dynamic behavior or modular scenario.

Simple Statements
+++++++++++++++++

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Syntax
     - Meaning
   * - :sampref:`model {name}`
     - Select the :term:`world model`.
   * - :sampref:`import {module}`
     - Import a Scenic or Python module.
   * - :sampref:`param {identifier} = {value}, {...}`
     - Define global parameters of the scenario.
   * - :sampref:`require {boolean}`
     - Define a hard requirement.
   * - :sampref:`require[{number}] {boolean}`
     - Define a soft requirement.
   * - :sampref:`require (always | eventually) {boolean}`
     - Define a dynamic hard requirement.
   * - :sampref:`terminate when {boolean}`
     - Define a termination condition.
   * - :sampref:`mutate {identifier}, {...} [by {number}]`
     - Enable mutation of the given list of objects.
   * - :sampref:`record [(initial | final)] {value} as {name}`
     - Save a value at every time step or only at the start/end of the simulation.

Dynamic Statements
++++++++++++++++++

These statements can only be used inside a :term:`dynamic behavior`, :term:`monitor`, or :sampref:`compose` block of a :term:`modular scenario`.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Syntax
     - Meaning
   * - :sampref:`take {action}, {...}`
     - Take the action(s) specified.
   * - :sampref:`wait`
     - Take no actions this time step.
   * - :sampref:`terminate`
     - Immediately end the scenario.
   * - :sampref:`do {behavior/scenario}, {...}`
     - Run one or more sub-behaviors/sub-scenarios until they complete.
   * - :sampref:`do {behavior/scenario}, {...} until {boolean}`
     - Run sub-behaviors/scenarios until they complete or a condition is met.
   * - :sampref:`do {behavior/scenario}, {...} for {scalar} (seconds | steps)`
     - Run sub-behaviors/scenarios for (at most) a specified period of time.
   * - :sampref:`do choose {behavior/scenario}, {...}`
     - Run *one* choice of sub-behavior/scenario whose preconditions are satisfied.
   * - :sampref:`do shuffle {behavior/scenario}, {...}`
     - Run several sub-behaviors/scenarios in a random order, satisfying preconditions.
   * - :sampref:`abort`
     - Break out of the current :sampref:`try-interrupt` statement.
   * - :sampref:`override {object} {specifier}, {...}`
     - Override properties of an object for the duration of the current scenario.

Objects
-------

The syntax :sampref:`{class} {specifier}, {...} <objectCreate>` creates an instance of a Scenic class.

The Scenic class `Point` provides the basic position properties in the first table below; its subclass `OrientedPoint` adds the orientation properties in the second table.
Finally, the class `Object`, which represents physical objects and is the default superclass of user-defined Scenic classes, adds the properties in the third table.
See the :ref:`objects_and_classes` for details.

===================  ==============  ================================================
   **Property**       **Default**                    **Meaning**
-------------------  --------------  ------------------------------------------------
 position [1]_        (0, 0)         position in global coordinates
 viewDistance          50            distance for the ‘can see’ operator
 mutationScale         0             overall scale of :ref:`mutations <mutate>`
 positionStdDev        1             mutation standard deviation for ``position``
===================  ==============  ================================================

Properties added by `OrientedPoint`:

===================  ==============  ================================================
   **Property**       **Default**                    **Meaning**
-------------------  --------------  ------------------------------------------------
 heading [1]_          0             heading in global coordinates
 viewAngle            360 degrees    angle for the ‘can see’ operator
 headingStdDev         5 degrees     mutation standard deviation for ``heading``
===================  ==============  ================================================

Properties added by `Object`:

===================  ==============  ================================================
   **Property**       **Default**                    **Meaning**
-------------------  --------------  ------------------------------------------------
 width                 1             width of bounding box (X axis)
 length                1             length of bounding box (Y axis)
 speed [1]_            0             initial speed (later, instantaneous speed)
 velocity [1]_       from ``speed``  initial velocity (later, instantaneous velocity)
 angularSpeed [1]_     0             angular speed (change in heading/time)
 behavior              `None`        :term:`dynamic behavior`, if any
 allowCollisions      `False`        whether collisions are allowed
 requireVisible       `True`         whether object must be visible from ego
 regionContainedIn    workspace      Region the object must lie within
 cameraOffset          (0, 0)        position of camera for ‘can see’
===================  ==============  ================================================

.. [1] These are :term:`dynamic properties`, updated automatically every time step during
    dynamic simulations.

Specifiers
----------

The :sampref:`with {property} {value}` specifier can specify any property, including new properties not built into Scenic.
Additional specifiers for the ``position`` and ``heading`` properties are listed below.

.. figure:: images/Specifier_Figure.png
  :width: 60%
  :figclass: align-center
  :alt: Diagram illustrating several specifiers.

  Illustration of the ``beyond``, ``behind``, and ``offset by`` specifiers.
  Each ``OrientedPoint`` (e.g. ``P``) is shown as a bold arrow.

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifier for ``position``
     - Meaning
   * - :sampref:`at {vector}`
     - Positions the object at the given global coordinates
   * - :sampref:`offset by {vector}`
     - Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined)
   * - :sampref:`offset along {direction} by {vector}`
     - Positions the object at the given coordinates, in a local coordinate system centered at ego and oriented along the given direction
   * - :sampref:`(left | right) of {vector} [by {scalar}]`
     - Positions the object further to the left/right by the given scalar distance
   * - :sampref:`(ahead of | behind) {vector} [by {scalar}]`
     - As above, except placing the object ahead of or behind the given position
   * - :sampref:`beyond {vector} by {vector} [from {vector}]`
     - Positions the object at coordinates given by the second vector, centered at the first vector and oriented along the line of sight from the third vector/ego
   * - :sampref:`visible [from ({Point} | {OrientedPoint})]`
     - Positions the object uniformly at random in the visible region of the ego, or of the given Point/OrientedPoint if given
   * - :sampref:`not visible [from ({Point} | {OrientedPoint})]`
     - Positions the object uniformly at random in the non-visible region of the ego, or of the given Point/OrientedPoint if given

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifier for ``position`` and optionally ``heading``
     - Meaning
   * - :sampref:`(in | on) {region}`
     - Positions the object uniformly at random in the given Region
   * - :sampref:`(left | right) of ({OrientedPoint} | {Object}) [by {scalar}]`
     - Positions the object to the left/right of the given OrientedPoint, depending on the object’s width
   * - :sampref:`(ahead of | behind) ({OrientedPoint} | {Object}) [by {scalar}]`
     - As above, except positioning the object ahead of or behind the given OrientedPoint, thereby depending on length
   * - :sampref:`following {vectorField} [from {vector}] for {scalar}`
     - Position by following the given vector field for the given distance starting from ego or the given vector


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifier for ``heading``
     - Meaning
   * - :sampref:`facing {heading}`
     - Orients the object along the given heading in global coordinates
   * - :sampref:`facing {vectorField}`
     - Orients the object along the given vector field at the object’s position
   * - :sampref:`facing (toward | away from) {vector}`
     - Orients the object toward/away from the given position (thereby depending on the object’s position)
   * - :sampref:`apparently facing {heading} [from {vector}]`
     - Orients the object so that it has the given heading with respect to the line of sight from ego (or the given vector)


Operators
---------

.. figure:: images/Operator_Figure.png
  :width: 70%
  :figclass: align-center
  :alt: Diagram illustrating several operators.

  Illustration of several operators.
  Each ``OrientedPoint`` (e.g. ``P``) is shown as a bold arrow.

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Scalar Operators
     - Meaning
   * - :sampref:`relative heading of {heading} [from {heading}]`
     - The relative heading of the given heading with respect to ego (or the ``from`` heading)
   * - :sampref:`apparent heading of {OrientedPoint} [from {vector}]`
     -  The apparent heading of the `OrientedPoint`, with respect to the line of sight from ego (or the given vector)
   * - :sampref:`distance [from {vector}] to {vector}`
     - The distance to the given position from ego (or the ``from`` vector)
   * - :sampref:`angle [from {vector}] to {vector}`
     - The heading to the given position from ego (or the ``from`` vector)

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Boolean Operators
     - Meaning
   * - :sampref:`({Point} | {OrientedPoint}) can see ({vector} | {Object})`
     - Whether or not a position or `Object` is visible from a `Point` or `OrientedPoint`.
   * - :sampref:`({vector} | {Object}) in {region}`
     -  Whether a position or `Object` lies in the region


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Heading Operators
     - Meaning
   * - :sampref:`{scalar} deg`
     - The given heading, interpreted as being in degrees
   * - :sampref:`{vectorField} at {vector}`
     - The heading specified by the vector field at the given position
   * - :sampref:`{direction} relative to {direction}`
     - The first direction, interpreted as an offset relative to the second direction


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Vector Operators
     - Meaning
   * - :sampref:`{vector} (relative to | offset by) {vector}`
     - The first vector, interpreted as an offset relative to the second vector (or vice versa)
   * - :sampref:`{vector} offset along {direction} by {vector}`
     - The second vector, interpreted in a local coordinate system centered at the first vector and oriented along the given direction


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Region Operators
     - Meaning
   * - :sampref:`visible {region}`
     - The part of the given region visible from ego

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - OrientedPoint Operators
     - Meaning
   * - :sampref:`{vector} relative to {OrientedPoint}`
     - The given vector, interpreted in the local coordinate system of the OrientedPoint
   * - :sampref:`{OrientedPoint} offset by {vector}`
     - Equivalent to ``vector relative to OrientedPoint`` above
   * - :sampref:`(front | back | left | right) of {Object}`
     - The midpoint of the corresponding edge of the bounding box of the Object, oriented along its heading
   * - :sampref:`(front | back) (left | right) of {Object}`
     - The corresponding corner of the Object’s bounding box, also oriented along its heading

Built in Functions
------------------

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Function
     - Description
   * - :ref:`Misc Python functions <gen_lifted_funcs>`
     - Various Python functions including ``min``, ``max``, ``sin``, ``cos``, etc.
   * - :ref:`filter_func`
     - Filter a possibly-random list (allowing limited randomized control flow).
   * - :ref:`resample_func`
     - Sample a new value from a distribution.
   * - :ref:`localPath_func`
     - Convert a relative path to an absolute path, based on the current directory.
   * - :ref:`verbosePrint_func`
     - Like `print`, but silent at low-enough verbosity levels.
   * - :ref:`simulation_func`
     - Get the the current simulation object.
