..  _statements:

***********************************
Statements Reference
***********************************

Compound Statements
===================

.. _classDef:

Class Definition
----------------

::

    class <name>[<superclass>]:
        (<property> : <value>)*

Defines a Scenic class. Any class that does not have an explicitly defined parent is assumed to inherit from the Scenic `Object` base class. Scenic objects attributes and methods in the same way as native Python classes.

.. _objectCreate:

Object Creation
---------------

::

    class [<specifier>] [, <specifier>]

Instantiates a Scenic object from a Scenic class. Parameters are set through the use of specifiers. For more details on specifiers, see the :ref:`specifiers`. If the name of an object is followed immediately by punctuation, then an object is not created. This allows us to refer to a Scenic class without creating an instance of that class in the environment, which is useful for statements like ``isinstance(obj, Car)``, ``[Taxi, Truck]``, ``Car.staticMethod``, etc...

.. _behaviorDef:

Behavior Definition
--------------------

::

    behavior <name>(<params>):
        (precondition: <boolean>)*
        (invariant: <boolean>)*
        <statement>*

Defines a Scenic behavior, which a Scenic object can perform by using the ``with behavior *behavior*`` syntax. Behavior preconditions are checked when a behavior is started, and invariants are checked at every timestep of the simulation (including the first like preconditions). Each timestep, behaviors must :ref:`take` specified action(s) or :ref:`wait` and perform no actions. Then the simulation advances one step and the behaviors resume right after the ``take`` or ``wait`` statement that was enacted in the last timestep. Behaviors also have the option to :ref:`terminate` the simulation, ending it immediately. Behaviors can also be composed using :ref:`do<do *behavior* [until *boolean*]>` statements. When performing sub-behaviors, you may wish to interupt them when certain conditions are met. This can be done by using :ref:`try interrupt<try>` statements. For more information on behaviors, see :ref:`dynamics`.

.. _monitorDef:

Monitor Definition
------------------

::

    monitor <name>:
        <statement>*

Defines a Scenic monitor, which runs in parallel with the simulation like a behavior. Monitors however cannot take actions, and instead can either :ref:`wait` or :ref:`terminate` the simulation. For more information on monitors, see :ref:`dynamics`.

.. _modularScenarioDef:

Modular Scenario Definition 
---------------------------

::

    scenarios <name>(<params>):
        (precondition: <boolean>)*
        (invariant: <boolean>)*
        [setup:
            <statement>*]
        [compose:
            <statement>*]

Defines a Scenic modular scenario. Scenario definitions, like behavior definitions, have preconditions and invariants. The body of a scenario consists of two optional parts: a setup block and a compose block. The setup block contains code that runs once when the scenario begins to execute, and is a list of statements like a top-level Scenic program. The compose block orchestrates the execution of sub-scenarios during a dynamic scenario, and may use do and any of the other statements allowed inside behaviors (except take, which only makes sense for an individual agent).

.. _tryInterruptStmt:

Try-Interrupt Statement
-----------------------

::

    try:
        <statement>*
    (interrupt when <boolean>:
        <statement>)*
    (except <exception>:
        <statement>*)*

A ``try-interrupt`` block can be placed inside a behavior to run a series of statements, including sub behaviors, while being able to interrupt at any point if certain conditions are violated. When a ``try-interrupt`` block is encountered, the statements under ``try`` are executed. If at any point one of the ``interrupt`` conditions is met, the ``interrupt`` block is entered and run. Once the ``interrupt`` block is complete, control is returned to the statement that was being executed under the ``try`` block. If there are multiple ``interrupt`` clauses, successive clauses take precedence over those which precede them. ``except`` statements are also supported, and function identically to their Python counterparts.

Standard Statements
===================

The following statements can occur throughout a Scenic program unless otherwise stated.

.. _model *name*:

model *name*
------------
Select the world model. ``model X`` is equivalent to ``from X import *`` except that ``X`` can be replaced using the ``--model`` command-line option or the ``model`` keyword argument to the top-level APIs. 

.. _import *module*:

import *module*
----------------
Imports a Scenic or Python module. This statement behaves as in Python, but when importing a Scenic module M it also imports any objects created and requirements imposed in M. Scenic also supports the form :samp:`from {module} import {identifier}, {...}` , which as in Python imports the module plus one or more identifiers from its namespace.

.. note::

    Scenic modules can only be imported at the top level, or in a top level try except block that does not create any objects. Python modules however, can be imported dynamically.

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

.. _require[*number*] *boolean*:

require[*number*] *boolean*
---------------------------
Defines a soft requirement, requiring that the given condition hold at least the percentage of the time specified. For example, ``require[0.75] ego in parking_lot`` would require that the ego be in the parking lot at least 75% percent of the time.

.. note::

    The provided number must be a literal number, not something that evaluates to a number.

.. _require (always | eventually) *boolean*:

require (always | eventually) *boolean*
---------------------------------------
Require a condition hold at each timestep (``always``) or at some point during the simulation (``eventually``).

.. _terminate when *boolean*:

terminate when *boolean*
------------------------
Terminates the scenario when the provided conditional evaluates to true.

.. note::
    
    If you are using modular scenarios and the current scenario was invoked from another scenario, only the current scenario will end, not the entire simulation.

.. _mutate *identifier*, . . . [by *number* ]:

mutate *identifier*, . . . [by *scalar* ]
-----------------------------------------
Enables mutation of the given list of objects, adding Gaussian noise with the given standard deviation (default 1) to their position and heading properties. If no objects are specified, mutation applies to every Object already created.

.. _record *expression* [(initial | final)] as *name*:

record *expression* [(initial | final)] as *name*
-------------------------------------------------
Record the expression as the name provided. The value can be recorded at the start of the simulation (initial), at the end of the simulation (final), or at every timestep if neither initial or final is specified. The values are available in the records dictionary of SimulationResult, and for debugging can also be printed out using the ``--show-records`` command-line option.

Dynamic Statements
==================

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
Used in an interrupt body to terminate the current :ref:`tryInterruptStmt` statement.

.. _override *name* *specifier*:

override *name* *specifier*
---------------------------
Override the property of an object dynamically for the duration of the current scenario.
