Tutorial on Dynamic Scenario
****************************
This tutorial covers how to model temporal aspect of scenarios.

Syntax
======

Object Properties
-----------------
There is a new property, behavior, which if not set to ``None`` designates an object as a dynamic agent. In this case, the property must be assigned a behavior (or distribution over behaviors)::

	Car on road, with behavior FollowLaneBehavior()

Defining Behaviors
------------------
Behaviors are defined using a slight modification of the Python function definition syntax::

	behavior Passing(<arguments>):
		precondition: <condition>
		invariant: <condition>
		<body>

There can be any number of ``precondition`` and ``invariant`` lines. The corresponding conditions are checked at runtime as explained above.

The body of a behavior has the same syntax as a Scenic (i.e. Python) function. Unlike ordinary Scenic code, control flow constructs such as ``if`` and ``while`` are allowed to depend on random variables inside a behavior. Any distributions defined inside a behavior are sampled at simulation time, not during scene sampling, and in particular can depend on the current state of the simulation. For example, if you write::

	anotherObject = Object at ...

	behavior Foo():
		while True:
			if (distance from self to anotherObject) > 5:
				...


then the distance is evaluated with respect to the positions of ``self`` (the agent using behavior ``Foo``) and ``anotherObject`` at the current time during the simulation, not their original positions.

Since behaviors are functions, they can use local variables and thus maintain state (which of course will be reset for every new simulation). It is also possible to change ``global`` state using the Python global statement, in order to communicate between behaviors.

Two existing Scenic constructs have different meanings inside a behavior:

	* ``self`` refers to the agent which is running the behavior.
	* The ``require`` statement, when violated, causes the current simulation to be discarded. Violated soft requirements cause the simulation to be discarded with the given probability.
	* There are also several special pieces of syntax which may only be used inside behaviors:

	* The ``take <action>`` statement picks the given ``Action`` to be the next primitive action for the agent. As explained above, execution of the behavior is then suspended until the next time step.
	* The ``wait`` statement is shorthand for ``take None``, causing no action to be taken this time step.
	* The ``terminate`` statement causes the current simulation to end.
	* The built-in function ``simulation()`` returns the ``Simulation`` object representing the current simulation. This can be used to access any simulator-specific data, as well as a few generic values: the ``objects`` property is a ``tuple`` of all ``Object``, ``agents`` is a ``tuple`` of all agents, and ``currentTime`` is the current time step (starting from 0).


Modelling Reactivity of Agents
------------------------------
Behaviors may use use a ``try-interrupt`` block of the form::

	try:
		<body>
	interrupt when <condition>:
		<handler>

Any number of ``interrupt when`` clauses is allowed, and ``except`` clauses as in Python are also allowed.

Such a block is executed as follows. Before the body is executed, and at every time step during its execution (i.e., whenever the agent's behavior would be resumed), the interrupt conditions are checked in order from bottom to top (i.e., each successive interrupt takes precedence over those preceding it). If a condition is met, the corresponding handler code block is executed. During execution of that block, higher-priority interrupt conditions (i.e. those lower down in the try-interrupt statement) continue to be checked and may cause further interrupt handlers to be executed. When a handler completes, control passes back to the code which was interrupted (either a lower-priority handler or the body of the whole try-interrupt statement). If the code being resumed is a behavior, its invariants are checked as usual, throwing exceptions if they are violated.

For example, suppose ``BehaviorA`` is a nominal behavior which is always safe to run, while we eventually want to run ``BehaviorB``, which can only happen under certain circumstances. There are some dangerous situations possible, some of which can be recovered from and some of which require us to give up entirely. We could write the following::

	while True:
		try:
			while <not ready for BehaviorB>:
				BehaviorA()
			BehaviorB()
		interrupt when <recoverable condition>:
			BehaviorC()
		catch InvariantFailure:
			pass	# do nothing
		interrupt when <fatal condition>:
			break

The nominal behavior is to continually run ``BehaviorA`` (which we assume terminates after a short time) until it is safe to run ``BehaviorB``. If a recoverable error happens, during ``BehaviorA``, we switch to ``BehaviorC`` to handle it and then resume. However, if the error happens during ``BehaviorB``, one of its invariants may be broken when we resume. Normally this would cause the simulation to abort, but here we explicitly catch the ``InvariantFailure``, doing nothing until we try again during the next iteration of the ``while`` loop. Finally, we have a last ``interrupt when`` clause to catch some other condition which requires us to abort the whole loop and try something different.

As illustrated above, when an interrupt handler completes, by default we resume execution of the interrupted code. If this is undesired, the ``abort`` statement causes the entire try-interrupt statement to exit.

When try-interrupt blocks are nested (for example, when a try-interrupt block calls a behavior which itself uses interrupts), the interrupts of the outer block take precedence over those of the inner block. For example, consider::

	try:
	    try:
	        A()
	    interrupt when <condition1>:
	        B()
	interrupt when <condition2>:
	    C()

Here, during execution of ``A`` at every time step ``condition2`` is checked before ``condition1``. During execution of ``B``, ``condition2`` is still checked. Whenever ``condition2`` holds, control passes to ``C``, returning to either ``A`` or ``B`` depending on when the interrupt occurred.


Defining When to Terminate Scenario
-----------------------------------
Termination criteria can be specified in two ways. The statement::

	terminate when <condition>

can be used at the top level of the Scenic program to add a termination criterion. Additionally, the ``terminate`` statement can be used inside behaviors: if it is ever executed, the simulation ends.


Defining Monitors
-----------------
To require that some condition always hold during simulations, you can write::

	require always <condition>

For more complex conditions, you can add a monitor function. These are in all ways identical to behaviors, except that they are not associated with any agent and any actions they pick are ignored (so you might as well only use the ``wait`` statement). Monitors are defined as follows::

	monitor mySpec:
		<body>

The body of a monitor, like that of a behavior, can use the ``require`` statement to enforce any desired conditions and ``terminate`` to end the simulation.


Semantics
=========
As in original Scenic, a program defines a distribution over scenes (consisting of a set of objects with their properties, plus global parameters). In addition, the program defines a policy used to control simulations involving dynamic agents: Scenic objects with a behavior assigned to their ``behavior`` property (more on this below). Each simulation begins from a scene sampled from the program, and proceeds in discrete simulation time steps. We call the assignment of values to all object properties, plus any state which the policy maintains, the state of the simulation. At each time step, the policy maps the current state to a (possibly random) choice of primitive actions for each dynamic agent. Specifically, the procedure for running a simulation is as follows:

1. A scene is sampled to be the initial condition for the simulation.
2. For each discrete time step (up to a specified time bound or forever):

	* All ``require always`` conditions are checked; if one fails, the simulation is discarded.
	* All monitors are executed until they execute ``wait`` (or select a primitive action, which is ignored). If a ``require`` statement is violated, the simulation is discarded; if a ``terminate`` statement is executed, a flag is set.
	* The behavior of each agent is executed, selecting a primitive action. The ``require`` and ``terminate`` statements behave as for monitors.
	* If the terminate flag is set, or any of the termination criteria from a ``terminate when`` statement are met, the simulation is ended.
	* The primitive actions are executed inside the simulator, in some order (by default the order of agent definitions, but customizable).
	* The simulation is advanced one time step.
	* The new values for ``position``, ``heading``, and potentially other properties (as specified by the simulator) are read back from the simulator into the Scenic objects.


Behaviors
---------
Behaviors are coroutines specifying the primitive actions to be taken for a single agent. At each time step, each behavior is executed until it decides on a primitive action; it is then suspended while the other agents' behaviors execute. As shown above, once all agents have chosen an action, the simulation itself runs for a time step, before the behaviors are resumed and the process repeats.

Behaviors may call other sub-behaviors. Control passes to the sub-behavior until it returns (which may never happen, if it continues to issue actions forever). Behaviors may return values, although Scenic itself does not assign any meaning to such values. If the top-level behavior for an agent returns, the agent is assumed to take no further actions for the rest of the simulation.

Behaviors may also have preconditions and invariants. When a behavior is first executed, its preconditions and invariants are checked, and any violations cause a ``PreconditionFailure`` exception to be raised (which may be caught using a standard Python ``try`` statement; uncaught exceptions will terminate Scenic as usual). Similarly, when a behavior is resumed after being suspended, its invariants are checked, and any violations cause an ``InvariantFailure`` exception to be raised.


Hierarchically Constructing Scenarios
=====================================
Scenic enables users to construct behaviors and in a bottom-up fashion as illustrated below, which makes scenario writing efficient. 

1. Primitive Actions
--------------------
Each simulator provides its own APIs to control agents in simulation with "primitive," or basic actions (e.g. setting a vehicle throttle or steering angle). For the simulators already interfaced with Scenic, we already provided a library of actions. For details, please refer to ``\src\scenic\domains\driving\actions.py``

2. Building Basic Behaviors from Primitive Actions
--------------------------------------------------
Suppose we want to construct a behavior for a vehicle to follow a lane. This can be done by::

	from scenic.domains.driving.actions import *

	behavior FollowLaneBehavior(lane):
		while True:
			throttle, steering = ... # compute steering at every simulation step
			take (SetThrottleAction(throttle), SetSteerAction(steering))

Note that the SetThrottleAction() and SetSteerAction() are imported from the library of actions, and how we are hierarchically building up the behavior from primitive actions. Also, note that execution of multiple primitive actions is possible in one simulation timestep. In such case, the actions to be taken simultaneously need to be provided in a tuple after ``take``.

3. Constructing Higher Level Behaviors from Basic Behaviors
-----------------------------------------------------------
Suppose we want to a vehicle to follow the lane but also brake if it comes within a certain distance from ego vehicle within a certain safety distance::

	behavior FollowTrafficBehavior(lane, safety_distance):
		try:
			FollowLaneBehavior(lane)
		interrupt when (distance from self to ego) < safety_distance:
			SlowDownBehavior()

Suppose ``SlowDownBehavior`` is also another basic behavior constructed from primitive action. Now, we created a behavior which normally follows the lane but will slow down if ego approaches nearby. 


Trying Some Examples
====================
An interface between a Scenic program and a simulator is instantiated with in a server/client communication over a websocket. 
Open a terminal and instantiate a simulator. On another terminal, instantiate the scenic program using the following command::

	python -m scenic --time 100 --count 10 -S examples/driving/badlyParkedCarPullingIn.sc -m scenic.simulators.carla.model

The ``m`` flag specifies the model. <-- need a way to easily differentiate and explain the first and the second -m flags

The ``time`` flag specifies the duration of each simulation run in simulation timesteps.
The ``count`` flag specifies the number of simulation runs to execute. And, ``S`` flag specifies to simulate. An absence of this 
``S`` flag will show Scenic's abstracted internal visualization of the world, which is often used for debugging purpose. 



