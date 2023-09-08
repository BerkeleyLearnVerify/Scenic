..  _composition:

Composing Scenarios
===================

.. py:currentmodule:: scenic.domains.driving.model

Scenic provides facilities for defining multiple scenarios in a single program and *composing* them in various ways.
This enables writing a library of scenarios which can be repeatedly used as building blocks to construct more complex scenarios.

Modular Scenarios
-----------------

The :keyword:`scenario <scenario-stmt>` statement defines a named, reusable scenario, optionally with tunable parameters: what we call a :term:`modular scenario`.
For example, here is a scenario which creates a parked car on the shoulder of the :scenic:`ego`'s current lane (assuming there is one), using some APIs from the :ref:`driving_domain`::

    scenario ParkedCar(gap=0.25):
        precondition: ego.laneGroup._shoulder != None
        setup:
            spot = new OrientedPoint on visible ego.laneGroup.curb
            parkedCar = new Car left of spot by gap

The ``setup`` block contains Scenic code which executes when the scenario is instantiated, and which can define classes, create objects, declare requirements, etc. as in any ordinary Scenic scenario.
Additionally, we can define preconditions and invariants, which operate in the same way as for :ref:`dynamic behaviors <guards>`.

Having now defined the ``ParkedCar`` scenario, we can use it in a more complex scenario, potentially multiple times::

    scenario Main():
        setup:
            ego = new Car
        compose:
            do ParkedCar(), ParkedCar(0.5)

Here our ``Main`` scenario itself only creates the ego car; then its ``compose`` block orchestrates how to run other modular scenarios.
In this case, we invoke two copies of the ``ParkedCar`` scenario in parallel, specifying in one case that the gap between the parked car and the curb should be 0.5 m instead of the default 0.25.
So the scenario will involve three cars in total, and as usual Scenic will automatically ensure that they are all on the road and do not intersect.

Parallel and Sequential Composition
-----------------------------------

The scenario above is an example of *parallel* composition, where we use the :keyword:`do` statement to run two scenarios at the same time.
We can also use *sequential* composition, where one scenario begins after another ends.
This is done the same way as in behaviors: in fact, the ``compose`` block of a scenario is executed in the same way as a monitor, and allows all the same control-flow constructs.
For example, we could write a ``compose`` block as follows:

.. code-block::
    :linenos:

    while True:
        do ParkedCar(gap=0.25) for 30 seconds
        do ParkedCar(gap=0.5) for 30 seconds

Here, a new parked car is created every 30 seconds, [#f1]_ with the distance to the curb alternating between 0.25 and 0.5 m.
Note that without the :scenic:`for 30 seconds` qualifier, we would never get past line 2, since the ``ParkedCar`` scenario does not define any termination conditions using :keyword:`terminate when` (or :keyword:`terminate` in a ``compose`` block) and so runs forever by default.
If instead we want to create a new car only when the :scenic:`ego` has passed the current one, we can use a :keyword:`do-until` statement::

    while True:
        subScenario = ParkedCar(gap=0.25)
        do subScenario until (distance past subScenario.parkedCar) > 10

Note how we can refer to the ``parkedCar`` variable created in the ``ParkedCar`` scenario as a property of the scenario.
Combined with the ability to pass objects as parameters of scenarios, this is convenient for reusing objects across scenarios.

Interrupts, Overriding, and Initial Scenarios
---------------------------------------------

The :keyword:`try-interrupt` statement used in behaviors can also be used in ``compose`` blocks to switch between scenarios.
For example, suppose we already have a scenario where the :scenic:`ego` is following a ``leadCar``, and want to elaborate it by adding a parked car which suddenly pulls in front of the lead car.
We could write a ``compose`` block as follows:

.. code-block::
    :linenos:

    following = FollowingScenario()
    try:
        do following
    interrupt when (distance to following.leadCar) < 10:
        do ParkedCarPullingAheadOf(following.leadCar)

If the ``ParkedCarPullingAheadOf`` scenario is defined to end shortly after the parked car finishes entering the lane, the interrupt handler will complete and Scenic will resume executing ``FollowingScenario`` on line 3 (unless the :scenic:`ego` is still within 10 m of the lead car).

Suppose that we want the lead car to behave differently while the parked car scenario is running; for example, perhaps the behavior for the lead car defined in ``FollowingScenario`` does not handle a parked car suddenly pulling in.
To enable changing the :prop:`behavior` or other properties of an object in a sub-scenario, Scenic provides the :keyword:`override` statement, which we can use as follows::

    scenario ParkedCarPullingAheadOf(target):
        setup:
            override target with behavior FollowLaneAvoidingCollisions
            parkedCar = new Car left of ...

Here we override the :prop:`behavior` property of ``target`` for the duration of the scenario, reverting it back to its original value (and thereby continuing to execute the old behavior) when the scenario terminates.
The :sampref:`override {object} {specifier}, {...}` statement takes a comma-separated list of specifiers like an :ref:`instance creation <objectCreate>`, and can specify any properties of the object except for :term:`dynamic properties` like :prop:`position` or :prop:`speed` which can only be indirectly controlled by taking actions.

In order to allow writing scenarios which can both stand on their own and be invoked during another scenario, Scenic provides a special conditional statement testing whether we are inside the *initial scenario*, i.e., the very first scenario to run.
For instance::

    scenario TwoLanePedestrianScenario():
        setup:
            if initial scenario:  # create ego on random 2-lane road
                roads = filter(lambda r: len(r.lanes) == 2, network.roads)
                road = Uniform(*roads)  # pick uniformly from list
                ego = new Car on road
            else:  # use existing ego car; require it is on a 2-lane road
                require len(ego.road.lanes) == 2
                road = ego.road
            new Pedestrian on visible road.sidewalkRegion, with behavior ...

Random Selection of Scenarios
-----------------------------

For very general scenarios, like "driving through a city, encountering typical human traffic", we may want a variety of different events and interactions to be possible.
We saw in the :ref:`dynamics` tutorial how we can write behaviors for individual agents which choose randomly between possible actions; Scenic allows us to do the same with entire scenarios.
Most simply, since scenarios are first-class objects, we can write functions which operate on them, perhaps choosing a scenario from a list of options based on some complex criterion::

    chosenScenario = pickNextScenario(ego.position, ...)
    do chosenScenario

However, some scenarios may only make sense in certain contexts; for example, a red light runner scenario can take place only at an intersection.
To facilitate modeling such situations, Scenic provides variants of the :keyword:`do` statement which randomly choose scenarios to run amongst only those whose preconditions are satisfied:

.. code-block::
    :linenos:

    do choose RedLightRunner, Jaywalker, ParkedCar(gap=0.5)
    do choose {RedLightRunner: 2, Jaywalker: 1, ParkedCar(gap=0.5): 1}
    do shuffle RedLightRunner, Jaywalker, ParkedCar

Here, line 1 checks the preconditions of the three given scenarios, then executes one (and only one) of the enabled scenarios. If for example the current road has no shoulder, then ``ParkedCar`` will be disabled and we will have a 50/50 chance of executing either ``RedLightRunner`` or ``Jaywalker`` (assuming their preconditions are satisfied).
If *none* of the three scenarios are enabled, Scenic will reject the simulation.
Line 2 shows a non-uniform variant, where ``RedLightRunner`` is twice as likely to be chosen as each of the other scenarios (so if only ``ParkedCar`` is disabled, we will pick ``RedLightRunner`` with probability 2/3; if none are disabled, 2/4).
Finally, line 3 is a shuffled variant, where *all three* scenarios will be executed, but in random order. [#f2]_


.. rubric:: Footnotes

.. [#f1] In a real implementation, we would probably want to require that the parked car is not initially visible from the :scenic:`ego`, to avoid the sudden appearance of cars out of nowhere.

.. [#f2] Respecting preconditions, so in particular the simulation will be rejected if at some point none of the remaining scenarios to execute are enabled.
