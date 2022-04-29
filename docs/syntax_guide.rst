..  _syntax_guide:

Syntax Guide
============

This page summarizes the syntax of Scenic (excluding syntax inherited from Python).
For more details, click the links for individual language constructs to go to the corresponding reference section.


Primitive Data Types
--------------------
======================= ==============================================================
:ref:`Booleans`          expressing truth values
:ref:`Scalars`           representing distances, angles, etc. as floating-point numbers
:ref:`Vectors`           representing positions and offsets in space
:ref:`Headings`   		   representing orientations in space
:ref:`Vector Fields`     associating an orientation (i.e. a heading) to each point in space
:ref:`Regions`           representing sets of points in space
======================= ==============================================================


Distributions
-------------
========================================  ==============================================================
:ref:`Range(low, high)`                   uniformly-distributed real number in the interval
:ref:`DiscreteRange(low, high)`           uniformly-distributed integer in the (fixed) interval
:ref:`Normal(mean, stdDev)`               normal distribution with the given mean and standard deviation
:ref:`Uniform(value, ...)`                uniform over a finite set of values
:ref:`DiscreteDistr`                      discrete with given values and weights
========================================  ==============================================================

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
     - Defines a dynamic behavior.
   * - :sampref:`monitor {name}: <monitorDef>`
     - Defines a monitor.
   * - :sampref:`scenario {name}({arguments}): <modularScenarioDef>`
     - Defines a modular scenario.
   * - :sampref:`try: {...} interrupt when {boolean}:<tryInterruptStmt>`
     - A try-interrupt block inside a dynamic behavior or modular scenario.

Simple Statements
+++++++++++++++++

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Syntax
     - Meaning
   * - :ref:`model *name*`
     - Select the world model.
   * - :ref:`import *module*`
     - Import a Scenic or Python module
   * - :ref:`param *identifier* = *value*, . . .`
     - Define global parameters of the scenario
   * - :ref:`require *boolean*`
     - Define a hard requirement
   * - :ref:`require[*number*] *boolean*`
     - Define a soft requirement
   * - :ref:`require (always | eventually) *boolean*`
     - Define a dynamic hard requirement
   * - :ref:`terminate when *boolean*`
     - Define a termination condition
   * - :ref:`mutate *identifier*, . . . [by *number* ]`
     - Enable mutation of the given list of objects
   * - :ref:`record [(initial | final)] *expression* as *name*`
     - Save a value at every time step or only at the start/end of the simulation.

Dynamic Statements
++++++++++++++++++

These statements can only be used inside a :term:`dynamic behavior`, monitor, or ``compose`` block of a modular scenario.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Syntax
     - Meaning
   * - :ref:`take *action*, ...`
     - Take the action(s) specified.
   * - :ref:`wait`
     - Take no actions this time step.
   * - :ref:`terminate`
     - Immediately end the scenario.
   * - :ref:`do *behavior* [until *boolean*]`
     - Perform a behavior until it completes or an optional ``until`` condition is met.
   * - :ref:`do *behavior* (for *scalar* seconds | for *scalar* steps)`
     - Perform a behavior for (at most) a specified period of time.
   * - :ref:`abort`
     - Breaks out of the current :ref:`tryInterruptStmt`
   * - :ref:`override *name* *specifier*`
     - Override properties of an object for the duration of the current scenario.

Objects
-------

The syntax :sampref:`{class} {specifier}, {...} <objectCreate>` creates an instance of a Scenic class.

Scenic objects representing physical objects are instances of the class `Object`, which provides the following built-in properties.
The basic position properties are inherited from `Point`, and the orientation properties are added by `OrientedPoint`.
See the :ref:`objects_and_classes` for details.

===================  =============  ===========================================
   **Property**       **Default**                   **Meaning**
-------------------  -------------  -------------------------------------------
 position             (0, 0)         position in global coordinates
 viewDistance          50            distance for the ‘can see’ operator
 mutationScale         0             overall scale of mutations
 positionStdDev        1             mutation standard deviation for position
-------------------  -------------  -------------------------------------------
 heading               0             heading in global coordinates
 viewAngle            360 degrees    angle for the ‘can see’ operator
 headingStdDev         5 degrees     mutation standard deviation for heading
-------------------  -------------  -------------------------------------------
 width                 1             width of bounding box (X axis)
 length                1             length of bounding box (Y axis)
 regionContainedIn    workspace      Region the object must lie within
 allowCollisions      false          whether collisions are allowed
 requireVisible        true          whether object must be visible from ego
===================  =============  ===========================================


Specifiers
----------

.. figure:: images/Specifier_Figure.png
  :width: 60%
  :figclass: align-center
  :alt: Diagram illustrating several specifiers.

  Illustration of the ``beyond``, ``behind``, and ``offset by`` specifiers.
  Each ``OrientedPoint`` (e.g. ``P``) is shown as a bold arrow.

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifier for Position
     - Meaning
   * - :ref:`at *vector*`
     - Positions the object at the given global coordinates
   * - :ref:`offset by *vector*`
     - Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined)
   * - :ref:`offset along *direction* by *vector*`
     - Positions the object at the given coordinates, in a local coordinate system centered at ego and oriented along the given direction
   * - :ref:`(left | right) of *vector* [by *scalar*]`
     - Positions the object further to the left/right by the given scalar distance
   * - :ref:`(ahead of | behind) *vector* [by *scalar*]`
     - As above, except placing the object ahead of or behind the given position
   * - :ref:`beyond *vector* by *vector* [from *vector*]`
     - Positions the object at coordinates given by the second vector, centered at the first vector and oriented along the line of sight from the third vector/ego
   * - :ref:`visible [from (*Point* | *OrientedPoint*)]`
     - Positions the object uniformly at random in the visible region of the ego, or of the given Point/OrientedPoint if given
   * - :ref:`not visible [from (*Point* | *OrientedPoint*)]`
     - Positions the object uniformly at random in the non-visible region of the ego, or of the given Point/OrientedPoint if given

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifiers for position and optionally heading
     - Meaning
   * - :ref:`(in | on) *region*`
     - Positions the object uniformly at random in the given Region
   * - :ref:`(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]`
     - Positions the object to the left/right of the given OrientedPoint, depending on the object’s width
   * - :ref:`(ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar* ]`
     - As above, except positioning the object ahead of or behind the given OrientedPoint, thereby depending on length
   * - :ref:`following *vectorField* [from *vector* ] for *scalar*`
     - Positions the object at a point obtained by following the given vector field for the given distance starting from ego


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifiers for heading
     - Meaning
   * - :ref:`facing *heading*`
     - Orients the object along the given heading in global coordinates
   * - :ref:`facing *vectorField*`
     - Orients the object along the given vector field at the object’s position
   * - :ref:`facing (toward | away from) *vector*`
     - Orients the object toward/away from the given position (thereby depending on the object’s position)
   * - :ref:`apparently facing *heading* [from *vector*]`
     - Orients the object so that it has the given heading with respect to the line of sight from ego (or from the position given by the optional from vector)


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
   * - :ref:`relative heading of *heading* [from *heading*]`
     - The relative heading of the given heading with respect to ego (or the heading provided with the optional from heading)
   * - :ref:`apparent heading of *OrientedPoint* [from *vector*]`
     -  The apparent heading of the OrientedPoint, with respect to the line of sight from ego (or the position provided with the optional from vector)
   * - :ref:`distance [from *vector* ] to *vector*`
     - The distance to the given position from ego (or the position provided with the optional from vector)
   * - :ref:`angle [from *vector* ] to *vector*`
     - The heading to the given position from ego (or the position provided with the optional from vector)

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Boolean Operators
     - Meaning
   * - :ref:`(*Point* | *OrientedPoint*) can see (*vector* | *Object*)`
     - Whether or not a position or Object is visible from a Point or OrientedPoint.
   * - :ref:`(*vector* | *Object*) in *region*`
     -  Whether a position or Object lies in the region


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Heading Operators
     - Meaning
   * - :ref:`*scalar* deg`
     - The given heading, interpreted as being in degrees
   * - :ref:`*vectorField* at *vector*`
     - The heading specified by the vector field at the given position
   * - :ref:`*direction* relative to *direction*`
     - The first direction, interpreted as an offset relative to the second direction


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Vector Operators
     - Meaning
   * - :ref:`*vector* (relative to | offset by) *vector*`
     - The first vector, interpreted as an offset relative to the second vector (or vice versa)
   * - :ref:`*vector* offset along *direction* by *vector*`
     - The second vector, interpreted in a local coordinate system centered at the first vector and oriented along the given direction


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Region Operators
     - Meaning
   * - :ref:`visible *region*`
     - The part of the given region visible from ego

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - OrientedPoint Operators
     - Meaning
   * - :ref:`*vector* relative to *OrientedPoint*`
     - The given vector, interpreted in the local coordinate system of the OrientedPoint
   * - :ref:`*OrientedPoint* offset by *vector*`
     - Equivalent to vector relative to OrientedPoint above
   * - :ref:`(front | back | left | right) of *Object*`
     - The midpoint of the corresponding edge of the bounding box of the Object, oriented along its heading
   * - :ref:`(front | back) (left | right) of *Object*`
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
