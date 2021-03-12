..  _options:

Command-Line Options
====================

The ``scenic`` command supports a variety of options. Run ``scenic -h`` for a full list
with short descriptions; we elaborate on some of the most important options below.

General Scenario Control
------------------------

.. option:: -m <model>, --model <model>

	Specify the world model to use for the scenario, overriding any ``model`` statement
	in the scenario. The argument must be the fully-qualified name of a Scenic module
	found on your :envvar:`PYTHONPATH` (it does not necessarily need to be built into
	Scenic).

.. option:: -p <param> <value>, --param <param> <value>

	Specify the value of a global parameter. This assignment overrides any
	``param`` statements in the scenario. If the given value can be interpreted as an
	int or float, it is; otherwise it is kept as a string.

Dynamic Simulations
-------------------

.. option:: -S, --simulate

	Run dynamic simulations from scenes instead of plotting scene diagrams. This option
	will only work for scenarios which specify a simulator, which is done automatically
	by the world models for the simulator interfaces that support dynamic scenarios, e.g.
	`scenic.simulators.carla.model` and `scenic.simulators.lgsvl.model`. If your scenario
	is written for an abstract domain, like `scenic.domains.driving`, you will need to
	use the :option:`--model` option to specify the specific model for the simulator you
	want to use.

.. option:: --time <steps>

	Maximum number of time steps to run each simulation (the default is infinity).
	Simulations may end earlier if termination criteria defined in the scenario are met.

.. option:: --count <number>

	Number of successful simulations to run (i.e., not counting rejected simulations).
	The default is to run forever.

Debugging
---------

.. option:: -v <verbosity>, --verbosity <verbosity>

	Set the verbosity level, from 0 to 3 (default 1):

		0
			Nothing is printed except error messages and
			:doc:`warnings <python:library/warnings>` (to ``stderr``). Warnings can be
			suppressed using the :envvar:`PYTHONWARNINGS` environment variable.
		1
			The main steps of compilation and scene generation are indicated, with timing
			statistics.
		2
			Additionally, details on which modules are being compiled and the reasons for
			any scene/simulation rejections are printed.
		3
			Additionally, the actions taken by each agent at each time step of a dynamic
			simulation are printed.
