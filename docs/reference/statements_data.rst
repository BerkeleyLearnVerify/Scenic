..  _statements_data:

***********************************
Data Types and Statements Reference
***********************************

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


Compound Statements
===================

.. _class:

Classes
-------

    class *name*[(*superclass*)]:
        (*property* : *value*)*

Defines a Scenic class. Any class that does not have an explicitly defined parent is assumed to inherit from the Scenic ``Object`` base class. Scenic objects attributes and methods in the same way as native Python classes.

.. _object:

Objects
-------

    *class* [*specifier*] [, *specifier*]

Instantiates a Scenic object from a Scenic class. Parameters are set through the use of specifiers. For more details, see the :ref:`specifiers`.

.. _behavior:

Behaviors
---------

    behavior *name*(*params*):
        (precondition: *boolean*)*
        (invariant: *boolean*)*
        (*statement*)*

Defines a Scenic behavior, which a Scenic object can perform by using the `with behavior *behavior*` syntax. Behavior preconditions are checked when a behavior is started, and invariants are checked at every timestep of the simulation (including the first like preconditions). Each timestep, behaviors must :ref:`take` specified action(s) or :ref:`wait` and perform no actions. Then the simulation advances one step and the behaviors resume right after the ``take`` or ``wait`` statement that was enacted in the last timestep. Behaviors can also be composed using :ref:`do<do *behavior* [until *boolean*]>` statements. When performing sub-behaviors, you may wish to interupt them when certain conditions are met. This can be done by using :ref:`try interrupt<try>` statements. For more information on behaviors, see :ref:`dynamic`.

.. _monitor:

Monitors
-------------------

    monitor *name*:
        (*statement*)*

Defines a Scenic monitory, which runs in parallel with the 

.. _scenario:

Scenarios
--------------------

.. _try:
Try Interrupt
--------------------


Standard Statements
===================

The following statements can occur throughout a Scenic program unless otherwise stated.

.. _model *name*:
model *name*
------------
Select the world model.

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

.. _require[*number*] **

require[*number*] *boolean*
---------------------------
Defines a soft requirement, requiring that the given condition hold at least the percentage of the time specified. For example, ``require[0.75] ego in parking_lot`` would require that the ego be in the parking lot at least 75% percent of the time.

.. _require (always | eventually) *boolean*:

require (always | eventually) *boolean*
---------------------------------------
Require a condition hold at each timestep (``always``) or at some point during the simulation (``eventually``).

.. _terminate [when *boolean*]:

terminate when *boolean*
--------------------------
Terminates the simulation when the provided conditional evaluates to true.

.. _mutate *identifier*, . . . [by *number* ]:

mutate *identifier*, . . . [by *number* ]
------------------------------------------
Enables mutation of the given list of objects, adding Gaussian noise with the given standard deviation (default 1) to their position and heading properties. If no objects are specified, mutation applies to every Object already created.

Interior Statements
===================

The following statements are valid only in ``behavior``, ``monitor``, and ``compose`` blocks.

.. _take *action*, ...:

take *action*, ...
------------------
Takes the action(s) specified and pass control to the simulator until the next timestep.

.. _wait:

wait
----
Take no actions this timestep.

.. _terminate:

terminate
---------
Immediately end the scenario.

.. _do *behavior* [until *boolean*]:

do *behavior* [until *boolean*]
-------------------------------
Perform a behavior. If an ``until`` condition is specified then, the behavior will terminate when the condition is met.

.. _do *behavior* (for *scalar* seconds | for *scalar* steps):

do *behavior* (for *scalar* seconds | for *scalar* steps)
---------------------------------------------------------
Perform a behavior for a set number of simulation seconds/timesteps.

.. _abort:

abort
-----
Used in an interrupt to terminate the current behavior.

.. _override *name* *specifier*:

override *name* *specifier*
---------------------------
Override the property of an object dynamically.
