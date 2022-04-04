..  _syntax_guide:

Syntax Guide
============

This page summarizes the syntax of Scenic (excluding syntax inherited from Python).
For more details, click the links for individual language constructs to go to the corresponding section of the :ref:`syntax_details`.


Primitive Data Types
--------------------
======================= ==============================================================
Booleans   		           expressing truth values
:ref:`Scalars`           representing distances, angles, etc. as floating-point numbers
:ref:`Vectors`           representing positions and offsets in space
:ref:`Headings`   		   representing orientations in space
:ref:`Vector Fields`     associating an orientation (i.e. a heading) to each point in space
:ref:`Regions`           representing sets of points in space
======================= ==============================================================


Distributions
-------------
================================== ==============================================================
Range(low, high)                    uniformly-distributed real number in the interval
DiscreteRange(low, high)            uniformly-distributed integer in the (fixed) interval
Normal(mean, stdDev)                normal distribution with the given mean and standard deviation
Uniform(value, ...)                 uniform over a finite set of values
Discrete({value: weight, . . . })   discrete with given values and weights
================================== ==============================================================


Objects
-------

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
   * - at *vector*
     - Positions the object at the given global coordinates
   * - offset by *vector*
     - Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined)
   * - :ref:`offset along *direction* by *vector*`
     - Positions the object at the given coordinates, in a local coordinate system centered at ego and oriented along the given direction
   * - :ref:`(left | right) of *vector* [by *scalar*]`
     - Positions the object further to the left/right by the given scalar distance
   * - :ref:`(ahead of | behind) *vector* [by *scalar*]`
     - As above, except placing the object ahead of or behind the given position
   * - :ref:`beyond *vector* by *vector* [from *vector*]`
     - Positions the object at coordinates given by the second vector, centered at the first vector and oriented along the line of sight from the ego
   * - visible [from (*Point* | *OrientedPoint*)]
     - Positions the object uniformly at random in the visible region of the ego, or of the given Point/OrientedPoint if given

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifiers for position and optionally heading
     - Meaning
   * - :ref:`(in | on) *region*`
     - Positions the object uniformly at random in the given Region
   * - :ref:`(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]`
     - Positions the object to the left/right of the given OrientedPoint, depending on the object’s width
   * - (ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar* ]
     - As above, except positioning the object ahead of or behind the given OrientedPoint, thereby depending on height
   * - :ref:`following *vectorField* [from *vector* ] for *scalar*`
     - Positions the object at a point obtained by following the given vector field for the given distance starting from ego


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Specifiers for heading
     - Meaning
   * - facing *heading*
     - Orients the object along the given heading in global coordinates
   * - facing *vectorField*
     - Orients the object along the given vector field at the object’s position
   * - facing (toward | away from) *vector*
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
   * - relative heading of *heading* [from *heading*]
     - The relative heading of the given heading with respect to ego (or the heading provided with the optional from heading)
   * - apparent heading of *OrientedPoint* [from *vector* ]
     -  The apparent heading of the OrientedPoint, with respect to the line of sight from ego (or the position provided with the optional from vector )
   * - distance [from *vector* ] to *vector*
     - The distance to the given position from ego (or the position provided with the optional from vector )
   * - :ref:`angle [from *vector* ] to *vector*`
     - The heading to the given position from ego (or the position provided with the optional from vector)

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Boolean Operators
     - Meaning
   * - :ref:`(*Point* | *OrientedPoint*) can see (*vector* | *Object*)`
     - Whether or not a position or Objectis visible from a Point or OrientedPoint. V
   * - :ref:`(*vector* | *Object*) in *region*`
     -  Whether a position or Object lies in the region


.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - Heading Operators
     - Meaning
   * - :ref:`*scalar* deg`
     - The given heading, interpreted as being in degrees
   * - *vectorField* at *vector*
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
   * - visible *region*
     - The part of the given region visible from ego
   * - *region* visible from (*Point* | *OrientedPoint*)
     - The part of the given region visible from the given Point/OrientedPoint

.. list-table::
   :widths: 80 20
   :header-rows: 1

   * - OrientedPoint Operators
     - Meaning
   * - :ref:`*vector* relative to *OrientedPoint*`
     - The given vector, interpreted in the local coordinate system of the OrientedPoint
   * - *OrientedPoint* offset by *vector*
     - Equivalent to vector relative to OrientedPoint above
   * - (front | back | left | right) of *Object*
     - The midpoint of the corresponding edge of the bounding box of the Object, oriented along its heading
   * - (front | back) (left | right) of *Object*
     - The corresponding corner of the Object’s bounding box, also oriented along its heading


Statements
----------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Syntax
     - Meaning
   * - :ref:`import *module*`
     - Imports a Scenic or Python module
   * - :ref:`param *identifier* = *value*, . . .`
     - Defines global parameters of the scenario
   * - :ref:`require *boolean*`
     - Defines a hard requirement
   * - :ref:`mutate *identifier*, . . . [by *number* ]`
     - Enables mutation of the given list of objects
