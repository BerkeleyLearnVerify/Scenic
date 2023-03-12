..  _api:

Using Scenic Programmatically
=============================

While Scenic is most easily invoked as a command-line tool, it also provides a Python API
for compiling Scenic programs, sampling scenes from them, and running dynamic
simulations.

The top-level interface to Scenic is provided by two functions in the ``scenic`` module
which compile a Scenic program:

.. autofunction:: scenic.scenarioFromFile

.. autofunction:: scenic.scenarioFromString

The resulting `Scenario` object represents the abstract scenario defined by the Scenic
program. To sample concrete scenes from this object, you can call the `Scenario.generate`
method, which returns a `Scene`. If you are only using static scenarios, you can extract
the sampled values for all the global parameters and objects in the scene from the
`Scene` object. For example:

.. testsetup::

	import os
	os.chdir('..')

.. testcode::

	import random, scenic
	random.seed(12345)
	scenario = scenic.scenarioFromString('ego = Object with foo Range(0, 5)')
	scene, numIterations = scenario.generate()
	print(f'ego has foo = {scene.egoObject.foo}')

.. testoutput::

	ego has foo = 2.083099362726706

To run dynamic scenarios, you must instantiate an instance of the `Simulator` class for
the particular simulator you want to use. Each simulator interface that supports dynamic
simulations defines a subclass of `Simulator`; for example, `NewtonianSimulator` for the
simple Newtonian simulator built into Scenic. These subclasses provide simulator-specific
functionality, and have different requirements for their use: see the specific
documentation of each interface under `scenic.simulators` for details.

Once you have an instance of `Simulator`, you can ask it to run a simulation from a
`Scene` by calling the `Simulator.simulate` method. If Scenic is able to run a simulation
that satisfies all the requirements in the Scenic program (potentially after multiple
attempts -- Scenic uses rejection sampling), this method will return a `Simulation`
object. Results of the simulation can then be obtained by inspecting its ``result``
attribute, which is an instance of `SimulationResult` (simulator-specific subclasses of
`Simulation` may also provide additional information). For example:

.. testcode::

	import scenic
	from scenic.simulators.newtonian import NewtonianSimulator
	scenario = scenic.scenarioFromFile('examples/driving/badlyParkedCarPullingIn.scenic',
	                                   model='scenic.simulators.newtonian.driving_model')
	scene, _ = scenario.generate()
	simulator = NewtonianSimulator()
	simulation = simulator.simulate(scene, maxSteps=10)
	if simulation:	# `simulate` can return None if simulation fails
		result = simulation.result
		for i, state in enumerate(result.trajectory):
			egoPos, parkedCarPos = state
			print(f'Time step {i}: ego at {egoPos}; parked car at {parkedCarPos}')

.. testoutput::
	:hide:
	:options: +ELLIPSIS

	Time step 0: ego at ...; parked car at ...
	Time step 1: ego at ...; parked car at ...
	Time step 2: ego at ...; parked car at ...
	Time step 3: ego at ...; parked car at ...
	Time step 4: ego at ...; parked car at ...
	Time step 5: ego at ...; parked car at ...
	Time step 6: ego at ...; parked car at ...
	Time step 7: ego at ...; parked car at ...
	Time step 8: ego at ...; parked car at ...
	Time step 9: ego at ...; parked car at ...
	Time step 10: ego at ...; parked car at ...

If you want to monitor data from simulations to see if the system you are testing
violates its specfications, you may want to use `VerifAI`_ instead of implementing your
own code along the lines above. VerifAI supports running tests from Scenic programs,
specifying system specifications using temporal logic or arbitrary Python monitor
functions, actively searching the space of parameters in a Scenic program to find
concrete scenarios where the system violates its specs [#f1]_, and more. See the VerifAI
documentation for details.

.. seealso:: If you get exceptions or unexpected behavior when using the API, Scenic provides various debugging features: see :ref:`debugging`.

.. [#f1] VerifAI's active samplers can be used directly from Scenic when VerifAI is
	installed. See `scenic.core.external_params`.

.. _VerifAI: https://verifai.readthedocs.io/
