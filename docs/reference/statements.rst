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

    class <name>[(<superclass>)]:
        (<property>: <value>)*

Defines a Scenic class.
If a superclass is not explicitly specified, `Object` is used (see :ref:`objects_and_classes`).
The body of the class defines a set of properties its objects have, together with default values for each property.
Properties are inherited from superclasses, and their default values may be overridden in a subclass.
Default values may also use the special syntax :samp:`self.{property}` to refer to one of the other properties of the same object, which is then a *dependency* of the default value.
The order in which to evaluate properties satisfying all dependencies is computed (and cyclic dependencies detected) during :ref:`specifier resolution`.

Scenic classes may also define attributes and methods in the same way as Python classes.

.. _behaviorDef:

Behavior Definition
--------------------

::

    behavior <name>(<arguments>):
        (precondition: <boolean>)*
        (invariant: <boolean>)*
        <statement>*

Defines a :term:`dynamic behavior`, which can be assigned to a Scenic object by setting its ``behavior`` property using the :samp:`with behavior {behavior}` specifier; this makes the object an :term:`agent`.
See our tutorial on :ref:`dynamics` for examples of how to write behaviors.

Behavior definitions have the same form as function definitions, with an argument list and a body consisting of one or more statements; the body may additionally begin with definitions of preconditions and invariants.
Preconditions are checked when a behavior is started, and invariants are checked at every time step of the simulation while the behavior is executing (including time step zero, like preconditions).
The body of a behavior executes in parallel with the simulation: in each time step, it must either :sampref:`take` specified action(s) or :sampref:`wait` and perform no actions.
After each :sampref:`take` or :sampref:`wait` statement, the behavior's execution is suspended, the simulation advances one step, and the behavior is then resumed.
It is thus an error for a behavior to enter an infinite loop which contains no :sampref:`take` or :sampref:`wait` statements (or :sampref:`do` statements invoking a sub-behavior; see below): the behavior will never yield control to the simulator and the simulation will stall.

Behaviors end naturally when their body finishes executing (or if they ``return``): if this happens, the agent performing the behavior will take no actions for the rest of the scenario.
Behaviors may also :sampref:`terminate` the current scenario, ending it immediately.

Behaviors may invoke sub-behaviors, optionally for a limited time or until a desired condition is met, using :sampref:`do` statements.
It is also possible to (temporarily) interrupt the execution of a sub-behavior under certain conditions and resume it later, using :ref:`try-interrupt <try>` statements.

.. _monitorDef:

Monitor Definition
------------------

::

    monitor <name>:
        <statement>*

Defines a Scenic :term:`monitor`, which runs in parallel with the simulation like a :term:`dynamic behavior`.
Monitors are not associated with an `Object` and cannot take actions, but can :sampref:`wait` to wait for the next time step (or :sampref:`terminate` the simulation).
The main purpose of monitors is to evaluate complex temporal properties that do not fit into the :sampref:`require always` and :sampref:`require eventually` statements: they can maintain state and use :sampref:`require` to enforce requirements depending on that state.
For examples of monitors, see our tutorial on :ref:`dynamics`.

.. _modularScenarioDef:
.. _scenario:
.. _setup:
.. _compose:

Modular Scenario Definition 
---------------------------

::

    scenario <name>(<arguments>):
        (precondition: <boolean>)*
        (invariant: <boolean>)*
        [setup:
            <statement>*]
        [compose:
            <statement>*]

::

    scenario <name>(<arguments>):
        <statement>*

Defines a Scenic :term:`modular scenario`.
Scenario definitions, like behavior definitions, may include preconditions and invariants.
The body of a scenario consists of two optional parts: a ``setup`` block and a ``compose`` block.
The ``setup`` block contains code that runs once when the scenario begins to execute, and is a list of statements like a top-level Scenic program (so it may create objects, define requirements, etc.).
The ``compose`` block orchestrates the execution of sub-scenarios during a dynamic scenario, and may use :sampref:`do` and any of the other statements allowed inside behaviors (except :sampref:`take`, which only makes sense for an individual :term:`agent`).
If a modular scenario does not use preconditions, invariants, or sub-scenarios (i.e., it only needs a ``setup`` block) it may be written in the second form above, where the entire body of the ``scenario`` comprises the ``setup`` block.

.. seealso:: Our tutorial on :ref:`composition` gives many examples of how to use modular scenarios.

.. _tryInterruptStmt:
.. _try-interrupt:

Try-Interrupt Statement
-----------------------

::

    try:
        <statement>*
    (interrupt when <boolean>:
        <statement>*)*
    (except <exception>:
        <statement>*)*

A ``try-interrupt`` statement can be placed inside a behavior (or :sampref:`compose` block of a :term:`modular scenario`) to run a series of statements, including invoking sub-behaviors with :sampref:`do`, while being able to interrupt at any point if given conditions are met.
When a ``try-interrupt`` statement is encountered, the statements in the ``try`` block are executed.
If at any time step one of the ``interrupt`` conditions is met, the corresponding ``interrupt`` block (its *handler*) is entered and run.
Once the interrupt handler is complete, control is returned to the statement that was being executed under the ``try`` block.

If there are multiple ``interrupt`` clauses, successive clauses take precedence over those which precede them; furthermore, during execution of an interrupt handler, successive ``interrupt`` clauses continue to be checked and can interrupt the handler.
Likewise, if ``try-interrupt`` statements are nested, the outermost statement takes precedence and can interrupt the inner statement at any time.
When one handler interrupts another and then completes, the original handler is resumed (and it may even be interrupted again before control finally returns to the ``try`` block).

The ``try-interrupt`` statement may conclude with any number of ``except`` blocks, which function identically to their :ref:`Python counterparts <except>`.

Simple Statements
=================

The following statements can occur throughout a Scenic program unless otherwise stated.

.. _model {name}:
.. _model:

model *name*
------------
Select a :term:`world model` to use for this scenario.
The statement :samp:`model {X}` is equivalent to :samp:`from {X} import *` except that :samp:`{X}` can be replaced using the :option:`--model` command-line option or the ``model`` keyword argument to the top-level APIs.
When writing simulator-agnostic scenarios, using the ``model`` statement is preferred to a simple ``import`` since a more specific world model for a particular simulator can then be selected at compile time.

.. _import {module}:
.. _import:

import *module*
----------------
Import a Scenic or Python module. This statement behaves :ref:`as in Python <import>`, but when importing a Scenic module it also imports any objects created and requirements imposed in that module.
Scenic also supports the form :samp:`from {module} import {identifier}, {...}` , which as in Python imports the module plus one or more identifiers from its namespace.

.. note::

    Scenic modules can only be imported at the top level, or in a top-level try-except block that does not create any objects (so that you can catch `ModuleNotFoundError` for example). Python modules can be imported dynamically inside functions as usual.

.. _param {identifier} = {value}, {...}:
.. _param:

param *identifier* = *value*, . . .
---------------------------------------
Defines one or more global parameters of the scenario.
These have no semantics in Scenic, simply having their values included as part of the generated `Scene`, but provide a general-purpose way to encode arbitrary global information.

If multiple ``param`` statements define parameters with the same name, the last statement takes precedence, except that Scenic world models imported using the :sampref:`model` statement do not override existing values for global parameters.
This allows models to define default values for parameters which can be overridden by particular scenarios.
Global parameters can also be overridden at the command line using the :option:`--param` option, or from the top-level API using the ``params`` argument to `scenic.scenarioFromFile`.

To access global parameters within the scenario itself, you can read the corresponding attribute of the ``globalParameters`` object.
For example, if you declare ``param weather = 'SUNNY'``, you could then access this parameter later in the program via ``globalParameters.weather``.
If the parameter was not overridden, this would evaluate to ``'SUNNY'``; if Scenic was run with the command-line option ``--param weather SNOW``, it would evaluate to ``'SNOW'`` instead.

.. _require {boolean}:
.. _require:

require *boolean*
------------------
Defines a hard requirement, requiring that the given condition hold in all instantiations of the scenario.
This is equivalent to an "observe" statement in other probabilistic programming languages.

.. _require[{number}] {boolean}:

require[*number*] *boolean*
---------------------------
Defines a soft requirement, requiring that the given condition hold with at least the given probability (which must be a literal number, not an expression).
For example, ``require[0.75] ego in parking_lot`` would require that the ego be in the parking lot at least 75% percent of the time.

.. _require (always | eventually) {boolean}:
.. _require always:
.. _require eventually:

require (always | eventually) *boolean*
---------------------------------------
Require a condition hold at each time step (``always``) or at some point during the simulation (``eventually``).

.. _terminate when {boolean}:
.. _terminate when:

terminate when *boolean*
------------------------
Terminates the scenario when the provided condition becomes true.
If this statement is used in a :term:`modular scenario` which was invoked from another scenario, only the current scenario will end, not the entire simulation.

.. _mutate {identifier}, {...} [by {number}]:
.. _mutate:

mutate *identifier*, . . . [by *scalar*]
-----------------------------------------
Enables mutation of the given list of objects, adding Gaussian noise with the given standard deviation (default 1) to their ``position`` and ``heading`` properties.
If no objects are specified, mutation applies to every `Object` already created.

.. note::

    User-defined classes may specify custom mutators to allow mutation to apply to properties other than ``position`` and ``heading``.
    This is done by providing a value for the ``mutator`` property, which should be an instance of `Mutator`.
    Mutators inherited from superclasses (such as the default ``position`` and ``heading`` mutators from `Point` and `OrientedPoint`) will still be applied unless the new mutator disables them; see `Mutator` for details.

.. _record [(initial | final)] {value} as {name}:
.. _record:

record [(initial | final)] *value* [as *name*]
----------------------------------------------
Record the value of an expression during each simulation.
The value can be recorded at the start of the simulation (``initial``), at the end of the simulation (``final``), or at every time step (if neither ``initial`` nor ``final`` is specified).
The recorded values are available in the ``records`` dictionary of `SimulationResult`: its keys are the given names of the records (or synthesized names if not provided), and the corresponding values are either the value of the recorded expression or a tuple giving its value at each time step as appropriate.
For debugging, the records can also be printed out using the :option:`--show-records` command-line option.

Dynamic Statements
==================

The following statements are valid only in :term:`dynamic behaviors`, :term:`monitors`, and :sampref:`compose` blocks.

.. _take {action}, {...}:
.. _take:

take *action*, ...
------------------
Takes the action(s) specified and pass control to the simulator until the next time step.
Unlike :sampref:`wait`, this statement may not be used in monitors or :term:`modular scenarios`, since these do not take actions.

.. _wait:

wait
----
Take no actions this time step.

.. _terminate:

terminate
---------
Immediately end the scenario.

.. _do {behavior/scenario}, {...}:
.. _do:

do *behavior/scenario*, ...
-------------------------------
Run one or more sub-behaviors or sub-scenarios in parallel.
This statement does not return until all invoked sub-behaviors/scenarios have completed.

.. _do {behavior/scenario}, {...} until {boolean}:
.. _do-until:

do *behavior/scenario*, ... until *boolean*
-------------------------------------------
As above, except the sub-behaviors/scenarios will terminate when the condition is met.

.. _do {behavior/scenario}, {...} for {scalar} (seconds | steps):

do *behavior/scenario* for *scalar* (seconds | steps)
-----------------------------------------------------
Run sub-behaviors/scenarios for a set number of simulation seconds/time steps.
This statement can return before that time if all the given sub-behaviors/scenarios complete.

.. _do choose {behavior/scenario}, {...}:
.. _do choose:

do choose *behavior/scenario*, ...
----------------------------------
Randomly pick one of the given behaviors/scenarios whose preconditions are satisfied, and run it.
If no choices are available, the simulation is rejected.

This statement also allows the more general form :samp:`do choose \\{ {behavior/scenario}: {weight}, {...} \}`, giving weights for each choice (which need not add up to 1).
Among all choices whose preconditions are satisfied, this picks a choice with probability proportional to its weight.

.. _do shuffle {behavior/scenario}, {...}:
.. _do shuffle:

do shuffle *behavior/scenario*, ...
-----------------------------------
Like :sampref:`do choose` above, except that when the chosen sub-behavior/scenario completes, a different one whose preconditions are satisfied is chosen to run next, and this repeats until all the sub-behaviors/scenarios have run once.
If at any point there is no available choice to run (i.e. we have a deadlock), the simulation is rejected.

This statement also allows the more general form :samp:`do shuffle \\{ {behavior/scenario}: {weight}, {...} \}`, giving weights for each choice (which need not add up to 1).
Each time a new sub-behavior/scenario needs to be selected, this statement finds all choices whose preconditions are satisfied and picks one with probability proportional to its weight.

.. _abort:

abort
-----
Used in an interrupt handler to terminate the current :sampref:`try-interrupt` statement.

.. _override {object} {specifier}, {...}:
.. _override:

override *object* *specifier*, ...
------------------------------------
Override one or more properties of an object, e.g. its ``behavior``, for the duration of the current scenario.
The properties will revert to their previous values when the current scenario terminates.
It is illegal to override :term:`dynamic properties`, since they are set by the simulator each time step and cannot be mutated manually.
