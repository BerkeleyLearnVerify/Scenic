..  _syntax_details:

****************
Syntax Reference
****************

Primitive Data Types
====================

.. _Booleans:

Booleans
--------
Booleans represent truth values, and can be `True` or `False`.

.. _Scalars:

Scalars
-------
Scalars represent distances, angles, etc. as floating-point numbers, which can be sampled from various distributions

.. _Vectors:

Vectors
-------
Vectors represent positions and offsets in space. They are constructed from coordinates with the syntax :samp:`{X} @ {Y}` (inspired by `Smalltalk <http://stephane.ducasse.free.fr/FreeBooks/BlueBook/Bluebook.pdf>`_); using a length-2 list or tuple (:samp:`[{X}, {Y}]` or :samp:`({X}, {Y})`) is also allowed.
By convention, coordinates are in meters, although the semantics of Scenic does not depend on this. More significantly, the vector syntax is specialized for 2-dimensional space. The 2D assumption dramatically simplifies much of Scenic’s syntax (particularly that dealing with orientations, as we will see below), while still being adequate for a variety of applications. However, it is important to note that the fundamental ideas of Scenic are not specific to 2D, and it would be easy to extend our implementation of the language to support 3D space.

.. _Headings:

Headings
--------
Headings represent orientations in space. Conveniently, in 2D these can be expressed using a single angle (rather than Euler angles or a quaternion). Scenic represents headings in radians, measured anticlockwise from North, so that a heading of 0 is due North and a heading of π/2 is due West. We use the convention that the heading of a local coordinate system is the heading of its y-axis, so that, for example, -2 @ 3 means 2 meters left and 3 ahead.

.. _Vector Fields:

Vector Fields
-------------
Vector fields associate an orientation (i.e. a heading) to each point in space. For example, a vector field could represent the shortest paths to a destination, or the nominal traffic direction on a road.

.. _Regions:

Regions
-------
Regions represent sets of points in space. Scenic provides a variety of ways to define Regions: rectangles, circular sectors, line segments, polygons, occupancy grids, and explicit lists of points. Regions can have an associated vector field giving points in the region preferred orientations. For example, a Region representing a lane of traffic could have a preferred orientation aligned with the lane, so that we can easily talk about distances along the lane, even if it curves. Another possible use of preferred orientations is to give the surface of an object normal vectors, so that other objects placed on the surface face outward by default.


Position Specifiers
===================

.. _at *vector*:

at *vector*
-----------
Positions the object at the given global coordinates.

.. _offset by *vector*:

offset by *vector*
------------------
Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined).

.. _offset along *direction* by *vector*:

offset along *direction* by *vector*
------------------------------------
Positions the object at the given coordinates, in a local coordinate system centered at ego and oriented along the given direction (which, if a vector field, is evaluated at ego to obtain a heading).

.. _(left | right) of *vector* [by *scalar*]:

(left | right) of *vector* [by *scalar*]
----------------------------------------
Depends on heading and width. Without the optional by scalar, positions the object immediately to the left/right of the given position; i.e., so that the midpoint of the object’s right/left edge is at that position. If by scalar is used, the object is placed further to the left/right by the given distance.

.. _(ahead of | behind) *vector* [by *scalar*]:

(ahead of | behind) *vector* [by *scalar*]
--------------------------------------------
As above, except placing the object ahead of or behind the given position (so that the midpoint of the object’s back/front edge is at that position); thereby depending on heading and length.

.. _beyond *vector* by *vector* [from *vector*]:

beyond *vector* by *vector* [from *vector*]
--------------------------------------------
Positions the object at coordinates given by the second vector, in a local coordinate system centered at the first vector and oriented along the line of sight from the third vector (i.e. a heading of 0 in the local coordinate system faces directly away from the first vector). If no third vector is provided, it is assumed to be the ego. For example, beyond taxi by 0 @ 3 means 3 meters directly behind the taxi as viewed by the camera.

.. _visible [from (*Point* | *OrientedPoint*)]:

visible [from (*Point* | *OrientedPoint*)]
------------------------------------------
Positions the object uniformly at random in the visible region of the ego, or of the given Point/OrientedPoint if given. Visible regions are defined as follows: a Point can see out to a certain distance, and an OrientedPoint restricts this to the circular sector along its heading with a certain angle. A position is then visible if it lies in the visible region and the position of the object being specified is set such that the center of the object is in this visible region.

.. _(in | on) *region*:

(in | on) *region*
------------------
Positions the object uniformly at random in the given Region. If the Region has a preferred orientation (a vector field), also optionally specifies heading to be equal to that orientation at the object’s position.

.. _(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]:

(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]
------------------------------------------------------------
Positions the object to the left/right of the given OrientedPoint, depending on the object’s width. Also optionally specifies heading to be the same as that of the OrientedPoint. If the OrientedPoint is in fact an Object, the object being constructed is positioned to the left/right of its left/right edge.

.. _(ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar* ]:

(ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar* ]
---------------------------------------------------------------
As above, except positioning the object ahead of or behind the given OrientedPoint, thereby depending on length

.. _following *vectorField* [from *vector* ] for *scalar*:

following *vectorField* [from *vector* ] for *scalar*
-----------------------------------------------------
Positions the object at a point obtained by following the given vector field for the given distance starting from ego (or the position optionally provided with from vector ). Optionally specifies heading to be the heading of the vector field at the resulting point. Uses a forward Euler approximation of the continuous vector field


Heading Specifiers
==================

.. _facing *heading*:

facing *heading*
----------------
Orients the object along the given heading in global coordinates

.. _facing *vectorField*:

facing *vectorField*
--------------------
Orients the object along the given vector field at the object’s position

.. _facing (toward | away from) *vector*:

facing (toward | away from) *vector*
------------------------------------
Orients the object toward/away from the given position (thereby depending on the object’s position)

.. _apparently facing *heading* [from *vector*]:

apparently facing *heading* [from *vector*]
--------------------------------------------
Orients the object so that it has the given heading with respect to the line of sight from ego (or from the position given by the optional from vector). For example, apparently facing 90 deg orients the object so that the camera views its left side head-on


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

Standard Statements
===================

.. _import *module*:

import *module*
----------------
Imports a Scenic or Python module. This statement behaves as in Python, but when importing a Scenic module M it also imports any objects created and requirements imposed in M. Scenic also supports the form :samp:`from {module} import {identifier}, {...}` , which as in Python imports the module plus one or more identifiers from its namespace. Note that a Scenic module can only be imported at the top level, or in a top level try except block that does not create any objects. Python modules however, can be imported dynamically.

.. _param *identifier* = *value*, . . .:

param *identifier* = *value*, . . .
---------------------------------------
Defines global parameters of the scenario. These have no semantics in Scenic, simply having their values included as part of the generated scene, but provide a general-purpose way to encode arbitrary global information.
If multiple ``param`` statements define parameters with the same name, the last statement takes precedence, except that Scenic world models imported using the ``model`` statement do not override existing values for global parameters.
This allows models to define default values for parameters which can be overridden by particular scenarios.
Global parameters can also be overridden at the command line using the :option:`--param` option.
To access global parameters, you must access the appropriate field in the ``globalParameters`` object. For example, if you declare ``param carSize = 3``, you could then access this parameter later in the program via ``globalParameters.carSize``. If the parameter was not overriden, then this would evaluate to 3. If it was overriden, it would evaluate to whatever it was set to at compilation time.

.. _require *boolean*:

require *boolean*
------------------
Defines a hard requirement, requiring that the given condition hold in all instantiations of the scenario. As noted above, this is equivalent to an observe statement in other probabilistic programming languages.

.. _mutate *identifier*, . . . [by *number* ]:

mutate *identifier*, . . . [by *number* ]
------------------------------------------
Enables mutation of the given list of objects, adding Gaussian noise with the given standard deviation (default 1) to their position and heading properties. If no objects are specified, mutation applies to every Object already created.

Dynamic Statements
==================

.. _behavior *identifier*:

behavior *identifier*
---------------------
Defines a behaviour. These can also contain ``precondition`` and ``invariant`` statements. For more details, see the :ref:`dynamics` section.

.. _take *action*, ...:

take *action*, ...
------------------
Takes the action(s) specified and pass control to the simulator until the next timestep.

.. _do *behavior* [until *boolean*]:

do *behavior* [until *boolean*]
-------------------------------
Perform a behavior. If an ``until`` condition is specified then, the behavior will terminate when the condition is met.

.. _do *behavior* (for *scalar* seconds | for *scalar* steps):

do *behavior* (for *scalar* seconds | for *scalar* steps)
---------------------------------------------------------
Perform a behavior for a set number of simulation seconds/timesteps.

.. _interrupt when *boolean*:

interrupt when *boolean*
------------------------
Defines an interrupt for a behavior. For more details, see the :ref:`dynamics` section.

.. _abort:

abort
-----
Used in an interrupt to terminate the current behavior.

.. _require (always | eventually) *boolean*:

require (always | eventually) *boolean*
---------------------------------------
Require a condition hold at each timestep (``always``) or at some point during the simulation (``eventually``).

.. _monitor *identifier*:

monitor *identifier*
--------------------
Defines a monitor. For more details, see the :ref:`dynamics` section.

.. _terminate [when *boolean*]:

terminate [when *boolean*]
--------------------------
Terminates a simulation. If a conditional is added via the when keyword, the simulation is terminated when that conditional
evaluates to true.
