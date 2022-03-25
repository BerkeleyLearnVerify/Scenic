..  _dynamics:

Dynamic Scenarios
=================

.. py:currentmodule:: scenic.domains.driving.model

The :ref:`tutorial` described how Scenic can model scenarios like "a badly-parked car" by defining spatial relationships between objects.
Here, we'll cover how to model *temporal* aspects of scenarios: for a scenario like "a badly-parked car, which pulls into the road as the ego car approaches", we need to specify not only the initial position of the car but how it behaves over time.

Agents, Actions, and Behaviors
------------------------------

In Scenic, we call objects which take actions over time *dynamic agents*, or simply
*agents*. These are ordinary Scenic objects, so we can still use all of Scenic's syntax
for describing their initial positions, orientations, etc. In addition, we specify their
dynamic behavior using a built-in property called ``behavior``. Here's an example using
one of the built-in behaviors from the :ref:`driving_domain`:

.. code-block:: scenic

    model scenic.domains.driving.model
    Car with behavior FollowLaneBehavior

A behavior defines a sequence of *actions* for the agent to take, which need not be fixed
but can be probabilistic and depend on the state of the agent or other objects. In
Scenic, an :term:`action` is an instantaneous operation executed by an agent, like
setting the steering angle of a car or turning on its headlights. Most actions are
specific to particular application domains, and so different sets of actions are provided
by different simulator interfaces. For example, the :ref:`driving_domain` defines a
`SetThrottleAction` for cars.

To define a behavior, we write a function which runs over the course of the scenario,
periodically issuing actions. Scenic uses a discrete notion of time, so at each time
step the function specifies zero or more actions for the agent to take. For example, here
is a very simplified version of the `FollowLaneBehavior` above:

.. code-block:: scenic

    behavior FollowLaneBehavior():
        while True:
            throttle, steering = ...    # compute controls
            take SetThrottleAction(throttle), SetSteerAction(steering)

We intend this behavior to run for the entire scenario, so we use an infinite loop. In
each step of the loop, we compute appropriate throttle and steering controls, then use
the ``take`` statement to take the corresponding actions. When that statement is
executed, Scenic pauses the behavior until the next time step of the simulation, when the
function resumes and the loop repeats.

When there are multiple agents, all of their behaviors run in parallel; each time step,
Scenic sends their selected actions to the simulator to be executed and advances the
simulation by one step. It then reads back the state of the simulation, updating the
positions and other :term:`dynamic properties` of the objects.

.. figure:: /images/scenic-sim.png
  :width: 50%
  :figclass: align-center
  :alt: Diagram showing interaction between Scenic and a simulator.

Behaviors can access the current state of the world to decide what actions to take:

.. code-block:: scenic

    behavior WaitUntilClose(threshold=15):
        while (distance from self to ego) > threshold:
            wait
        do FollowLaneBehavior()

Here, we repeatedly query the distance from the agent running the behavior (``self``)
to the ego car; as long as it is above a threshold, we ``wait``, which means take no
actions. Once the threshold is met, we start driving by invoking the `FollowLaneBehavior`
we saw above using the ``do`` statement. Since `FollowLaneBehavior` runs forever, we will
never return to the ``WaitUntilClose`` behavior.

The example above also shows how behaviors may take arguments, like any Scenic function.
Here, ``threshold`` is an argument to the behavior which has default value 15 but can be
customized, so we could write for example:

.. code-block:: scenic

    ego = Car
    car2 = Car visible, with behavior WaitUntilClose
    car3 = Car visible, with behavior WaitUntilClose(20)

Both ``car2`` and ``car3`` will use the ``WaitUntilClose`` behavior, but independent
copies of it with thresholds of 15 and 20 respectively.

Unlike ordinary Scenic code, control flow constructs such as ``if`` and ``while`` are
allowed to depend on random variables inside a behavior. Any distributions defined inside
a behavior are sampled at simulation time, not during scene sampling. Consider the
following behavior:

.. code-block:: scenic
    :linenos:

    behavior Foo:
        threshold = Range(4, 7)
        while True:
            if self.distanceToClosest(Pedestrian) < threshold:
                strength = TruncatedNormal(0.8, 0.02, 0.5, 1)
                take SetBrakeAction(strength), SetThrottleAction(0)
            else:
                take SetThrottleAction(0.5), SetBrakeAction(0)

Here, the value of ``threshold`` is sampled only once, at the beginning of the scenario
when the behavior starts running. The value ``strength``, on the other hand, is sampled
every time control reaches line 5, so that every time step when the car is braking we use
a slightly different braking strength (0.8 on average, but with Gaussian noise added with
standard deviation 0.02, truncating the possible values to between 0.5 and 1).

Interrupts
----------

It is frequently useful to take an existing behavior and add a complication to it; for
example, suppose we want a car that follows a lane, stopping whenever it encounters an
obstacle. Scenic provides a concept of *interrupts* which allows us to reuse the basic
`FollowLaneBehavior` without having to modify it.

.. code-block:: scenic

    behavior FollowAvoidingObstacles():
        try:
            do FollowLaneBehavior()
        interrupt when self.distanceToClosest(Object) < 5:
            take SetBrakeAction(1)

This ``try-interrupt`` statement has similar syntax to the Python
:ref:`try statement <python:try>` (and in fact allows ``except`` clauses just as in
Python), and begins in the same way: at first, the code block after the ``try:`` (the
*body*) is executed. At the start of every time step during its execution, the condition
from each ``interrupt`` clause is checked; if any are true, execution of the body is
suspended and we instead begin to execute the corresponding *interrupt handler*. In the
example above, there is only one interrupt, which fires when we come within 5 meters of
any object. When that happens, `FollowLaneBehavior` is paused and we instead apply full
braking for one time step. In the next step, we will resume `FollowLaneBehavior` wherever
it left off, unless we are still within 5 meters of an object, in which case the
interrupt will fire again.

If there are multiple ``interrupt`` clauses, successive clauses take precedence over
those which precede them. Furthermore, such higher-priority interrupts can fire even
during the execution of an earlier interrupt handler. This makes it easy to model a
hierarchy of behaviors with different priorities; for example, we could implement a car
which drives along a lane, passing slow cars and avoiding collisions, along the
following lines:

.. code-block:: scenic

    behavior Drive():
        try:
            do FollowLaneBehavior()
        interrupt when self.distanceToNextObstacle() < 20:
            do PassingBehavior()
        interrupt when self.timeToCollision() < 5:
            do CollisionAvoidance()

Here, the car begins by lane following, switching to passing if there is a car or other
obstacle too close ahead. During *either* of those two sub-behaviors, if the time to
collision gets too low, we switch to collision avoidance. Once the ``CollisionAvoidance``
behavior completes, we will resume whichever behavior was interrupted earlier. If we were
in the middle of ``PassingBehavior``, it will run to completion (possibly being
interrupted again) before we finally resume ``FollowLaneBehavior``.

As this example illustrates, when an interrupt handler completes, by default we resume
execution of the interrupted code. If this is undesired, the ``abort`` statement can be
used to cause the entire try-interrupt statement to exit. For example, to run a behavior
until a condition is met without resuming it afterward, we can write:

.. code-block:: scenic

    behavior ApproachAndTurnLeft():
        try:
            do FollowLaneBehavior()
        interrupt when (distance from self to intersection) < 10:
            abort    # cancel lane following
        do WaitForTrafficLightBehavior()
        do TurnLeftBehavior()

This is a common enough use case of interrupts that Scenic provides a shorthand notation:

.. code-block:: scenic

    behavior ApproachAndTurnLeft():
        do FollowLaneBehavior() until (distance from self to intersection) < 10
        do WaitForTrafficLightBehavior()
        do TurnLeftBehavior()

Scenic also provides a shorthand for interrupting a behavior after a certain period of
time:

.. code-block:: scenic

    behavior DriveForAWhile():
        do FollowLaneBehavior() for 30 seconds

The alternative form :samp:`do {behavior} for {n} steps` uses time steps instead of real
simulation time.

Finally, note that when try-interrupt statements are nested, interrupts of the outer
statement take precedence. This makes it easy to build up complex behaviors in a modular
way. For example, the behavior ``Drive`` we wrote above is relatively complicated, using
interrupts to switch between several different sub-behaviors. We would like to be able to
put it in a library and reuse it in many different scenarios without modification.
Interrupts make this straightforward; for example, if for a particular scenario we want a
car that drives normally but suddenly brakes for 5 seconds when it reaches a certain
area, we can write:

.. code-block:: scenic

    behavior DriveWithSuddenBrake():
        haveBraked = False
        try:
            do Drive()
        interrupt when self in targetRegion and not haveBraked:
            do StopBehavior() for 5 seconds
            haveBraked = True

With this behavior, ``Drive`` operates as it did before, interrupts firing as appropriate
to switch between lane following, passing, and collision avoidance. But during any of
these sub-behaviors, if the car enters the ``targetRegion`` it will immediately brake for
5 seconds, then pick up where it left off.

Stateful Behaviors
------------------

As the last example shows, behaviors can use local variables to maintain state, which is
useful when implementing behaviors which depend on actions taken in the past. To
elaborate on that example, suppose we want a car which usually follows the ``Drive``
behavior, but every 15-30 seconds stops for 5 seconds. We can implement this behavior as
follows:

.. code-block:: scenic

    behavior DriveWithRandomStops():
        delay = Range(15, 30) seconds
        last_stop = 0
        try:
            do Drive()
        interrupt when simulation.currentTime - last_stop > delay:
            do StopBehavior() for 5 seconds
            delay = Range(15, 30) seconds
            last_stop = simulation.currentTime

Here ``delay`` is the randomly-chosen amount of time to run ``Drive`` for,
and ``last_stop`` keeps track of the time when we last started to run it. When the time
elapsed since ``last_stop`` exceeds ``delay``, we interrupt ``Drive`` and
stop for 5 seconds. Afterwards, we pick a new ``delay`` before the next stop, and save
the current time in ``last_stop``, effectively resetting our timer to zero.

.. note::

    It is possible to change global state from within a behavior by using the Python
    :ref:`global statement <python:global>`, for instance to communicate between
    behaviors. If using this ability, keep in mind that the order in which behaviors of
    different agents is executed within a single time step could affect your results. The
    default order is the order in which the agents were defined, but it can be adjusted
    by overriding the `Simulation.scheduleForAgents` method.

Requirements and Monitors
-------------------------

Just as you can declare spatial constraints on scenes using the ``require`` statement,
you can also impose constraints on dynamic scenarios. For example, if we don't want to
generate any simulations where ``car1`` and ``car2`` are simultaneously visible from the
ego car, we could write:

.. code-block:: scenic

    require always not ((ego can see car1) and (ego can see car2))

The :samp:`require always {condition}` statement enforces that the given condition must
hold at every time step of the scenario; if it is ever violated during a simulation, we
reject that simulation and sample a new one. Similarly, we can require that a condition
hold at *some* time during the scenario using the ``require eventually`` statement:

.. code-block:: scenic

    require eventually ego in intersection

You can also use the ordinary ``require`` statement inside a behavior to require that a
given condition hold at a certain point during the execution of the behavior. For
example, here is a simple elaboration of the ``WaitUntilClose`` behavior we saw above:

.. code-block:: scenic

    behavior WaitUntilClose(threshold=15):
        while (distance from self to ego) > threshold:
            require self.distanceToClosest(Pedestrian) > threshold
            wait
        do FollowLaneBehavior()

The requirement ensures that no pedestrian comes close to ``self`` until the ego does;
after that, we place no further restrictions.

To enforce more complex temporal properties like this one without modifying behaviors,
you can define a *monitor*. Like behaviors, monitors are functions which run in parallel
with the scenario, but they are not associated with any agent and any actions they take
are ignored (so you might as well only use the ``wait`` statement). Here is a monitor
for the property "``car1`` and ``car2`` enter the intersection before ``car3``":

.. code-block:: scenic
    :linenos:

    monitor Car3EntersLast:
        seen1, seen2 = False, False
        while not (seen1 and seen2):
            require car3 not in intersection
            if car1 in intersection:
                seen1 = True
            if car2 in intersection:
                seen2 = True
            wait

We use the variables ``seen1`` and ``seen2`` to remember whether we have seen ``car1``
and ``car2`` respectively enter the intersection. The loop will iterate as long as at
least one of the cars has not yet entered the intersection, so if ``car3`` enters before
either ``car1`` or ``car2``, the requirement on line 4 will fail and we will reject the
simulation. Note the necessity of the ``wait`` statement on line 9: if we omitted it, the
loop could run forever without any time actually passing in the simulation.

..  _guards:

Preconditions and Invariants
----------------------------

Even general behaviors designed to be used in multiple scenarios may not operate
correctly from all possible starting states: for example, `FollowLaneBehavior` assumes
that the agent is actually in a lane rather than, say, on a sidewalk. To model such
assumptions, Scenic provides a notion of *guards* for behaviors. Most simply, we can
specify one or more *preconditions*:

.. code-block:: scenic

    behavior MergeInto(newLane):
        precondition: self.lane is not newLane and self.road is newLane.road
        ...

Here, the precondition requires that whenever the ``MergeInto`` behavior is executed by
an agent, the agent must not already be in the destination lane but should be on the same
road. We can add any number of such preconditions; like ordinary requirements, violating
any precondition causes the simulation to be rejected.

Since behaviors can be interrupted, it is possible for a behavior to resume execution in
a state it doesn't expect: imagine a car which is lane following, but then swerves onto
the shoulder to avoid an accident; naïvely resuming lane following, we find we are no
longer in a lane. To catch such situations, Scenic allows us to define *invariants* which
are checked at every time step during the execution of a behavior, not just when it
begins running. These are written similarly to preconditions:

.. code-block:: scenic

    behavior FollowLaneBehavior():
        invariant: self in road
        ...

While the default behavior for guard violations is to reject the simulation, in some
cases it may be possible to recover from a violation by taking some additional actions.
To enable this kind of design, Scenic signals guard violations by raising a
`GuardViolation` exception which can be caught like any other exception; the simulation
is only rejected if the exception propagates out to the top level. So to model the
lane-following-with-collision-avoidance behavior suggested above, we could write code
like this:

.. code-block:: scenic

    behavior Drive():
        while True:
            try:
                do FollowLaneBehavior()
            interrupt when self.distanceToClosest(Object) < 5:
                do CollisionAvoidance()
            except InvariantViolation:   # FollowLaneBehavior has failed
                do GetBackOntoRoad()

When any object comes within 5 meters, we suspend lane following and switch to collision
avoidance. When the ``CollisionAvoidance`` behavior completes, ``FollowLaneBehavior``
will be resumed; if its invariant fails because we are no longer on the road, we catch
the resulting `InvariantViolation` exception and run a ``GetBackOntoRoad`` behavior to
restore the invariant. The whole ``try`` statement then completes, so the outermost loop
iterates and we begin lane following once again.

Terminating the Scenario
------------------------

By default, scenarios run forever, unless the :option:`--time` option is used to impose a
time limit. However, scenarios can also define termination criteria using the
``terminate when`` statement; for example, we could decide to end a scenario as soon as
the ego car travels at least a certain distance:

.. code-block:: scenic

    start = Point on road
    ego = Car at start
    terminate when (distance to start) >= 50

Additionally, the ``terminate`` statement can be used inside behaviors and monitors: if
it is ever executed, the scenario ends. For example, we can use a monitor to terminate
the scenario once the ego spends 30 time steps in an intersection:

.. code-block:: scenic

    monitor StopAfterTimeInIntersection:
        totalTime = 0
        while totalTime < 30:
            if ego in intersection:
                totalTime += 1
            wait
        terminate

.. note::

    In order to make sure that requirements are not violated, termination criteria are
    only checked *after* all requirements. So if in the same time step a monitor uses the
    ``terminate`` statement but another behavior uses ``require`` with a false condition,
    the simulation will be rejected rather than terminated.

..  _dynamics_running_examples:

Trying Some Examples
--------------------

You can see all of the above syntax in action by running some of our examples of dynamic
scenarios. We have examples written for the CARLA and LGSVL driving simulators, and those
in :file:`examples/driving` in particular are designed to use Scenic's abstract
:ref:`driving domain <driving_domain>` and so work in either of these simulators, as well
as Scenic's built-in Newtonian physics simulator. The Newtonian simulator is convenient
for testing and simple experiments; you can find details on how to install the more
realistic simulators in our :ref:`simulators` page (they should work on both Linux and
Windows, but not macOS, at the moment).

Let's try running
:file:`examples/driving/badlyParkedCarPullingIn.scenic`, which implements the "a
badly-parked car, which pulls into the road as the ego car approaches" scenario we
mentioned above. To start out, you can run it like any other Scenic scenario to get the
usual schematic diagram of the generated scenes:

.. code-block:: console

    $ scenic examples/driving/badlyParkedCarPullingIn.scenic

To run dynamic simulations, add the :option:`--simulate` option (:option:`-S` for short).
Since this scenario is not written for a particular simulator, you'll need to specify
which one you want by using the :option:`--model` option (:option:`-m` for short) to
select the corresponding Scenic world model: for example, to use the Newtonian simulator we could add
``--model scenic.simulators.newtonian.model``. It's also a good idea to put a time bound on
the simulations, which we can do using the :option:`--time` option.

.. code-block:: console

    $ scenic examples/driving/badlyParkedCarPullingIn.scenic \
        --simulate \
        --model scenic.simulators.newtonian.model \
        --time 200

Running the scenario in CARLA is exactly the same, except we use the
``--model scenic.simulators.carla.model`` option instead (make sure to start CARLA
running first). For LGSVL, the one difference is that this scenario
specifies a map which LGSVL doesn't have built in; fortunately, it's easy to switch to a
different map. For scenarios using the :ref:`driving domain <driving_domain>`, the map
file is specified by defining a global parameter ``map``, and for the LGSVL interface we
use another parameter ``lgsvl_map`` to specify the name of the map in LGSVL (the CARLA
interface likewise uses a parameter ``carla_map``). These parameters can be set at the
command line using the :option:`--param` option (:option:`-p` for short); for example,
let's pick the "BorregasAve" LGSVL map, an OpenDRIVE file for which is included in the
Scenic repository. We can then run a simulation by starting LGSVL in "API Only" mode and
invoking Scenic as follows:

.. code-block:: console

    $ scenic examples/driving/badlyParkedCarPullingIn.scenic \
        --simulate \
        --model scenic.simulators.lgsvl.model \
        --time 200 \
        --param map tests/formats/opendrive/maps/LGSVL/borregasave.xodr \
        --param lgsvl_map BorregasAve

Try playing around with different example scenarios and different choices of maps (making
sure that you keep the ``map`` and ``lgsvl_map``/``carla_map`` parameters consistent).
For both CARLA and LGSVL, you don't have to restart the simulator between scenarios: just
kill Scenic [#f1]_ and restart it with different arguments.

Further Reading
---------------

This tutorial illustrated most of Scenic's core syntax for dynamic scenarios. As with the
rest of Scenic's syntax, these constructs are summarized in our :ref:`syntax_guide`, with
links to detailed documentation in the :ref:`syntax_details`. You may also be interested
in some other sections of the documentation:

    :ref:`composition`
        Building more complex scenarios out of simpler ones in a modular way.

    :ref:`simulators`
        Details on which simulator interfaces support dynamic scenarios.

.. rubric:: Footnotes

.. [#f1] Or use the :option:`--count` option to have Scenic automatically terminate after
    a desired number of simulations.
