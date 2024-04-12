################
Basic Usage 
################

.. testsetup::

	import os
	os.chdir('..')

************************
Setting up the Falsifier
************************

Defining a Sample Space and Choosing a Sampler
===============================================
There are two ways of defining a feature space.

Method 1: Using :ref:`Feature APIs`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. testcode::

	from verifai.features import *
	from verifai.samplers import *

	control_params = Struct({
        'x_init': Box([-0.05, 0.05]),
        'cruising_speed': Box([10.0, 20.0]),
        'reaction_time': Box([0.7, 1.00])
	})

	env_params = Struct({
	        'broken_car_color': Box([0.5, 1], [0.25, 0.75], [0, 0.5]),
	        'broken_car_rotation': Box([5.70, 6.28])
	    })

	cones_config = Struct({
	        'traffic_cones_pos_noise': Box([-0.25, 0.25], [-0.25, 0.25], [-0.25, 0.25]),
	        'traffic_cones_down_0': Categorical(*np.arange(5)),
	        'traffic_cones_down_1': Categorical(*np.arange(5)),
	        'traffic_cones_down_2': Categorical(*np.arange(5))
	    })

	sample_space = FeatureSpace({
		'control_params': Feature(control_params),
		'env_params': Feature(env_params),
		'cones_config': Feature(cones_config)
	})

	# Examples of Instantiating Some of VerifAI's Supported Samplers
	random_sampler                = FeatureSampler.randomSamplerFor(sample_space)
	halton_sampler                = FeatureSampler.haltonSamplerFor(sample_space)
	cross_entropy_sampler         = FeatureSampler.crossEntropySamplerFor(sample_space)
	simulated_annealing_sampler   = FeatureSampler.simulatedAnnealingSamplerFor(sample_space)


Method 2: Using Scenic
^^^^^^^^^^^^^^^^^^^^^^

.. testcode::

	from verifai.samplers import ScenicSampler

	path = 'examples/webots/controllers/scenic_cones_supervisor/lane_cones.scenic'
	scenic_sampler = ScenicSampler.fromScenario(path)

Scenic's sampler, by default, does random sampling (see :obj:`~verifai.samplers.ScenicSampler` for the available configuration options).
However, it is possible to invoke VerifAI's other samplers from within the scenario using Scenic's :term:`external parameters`.


Constructing a Monitor 
====================================================================
Active samplers sample for the next point in the feature space by accounting for the history of
the performance of a system being tested in previous simulations. To use active samplers,
a user need to provide a monitor (i.e. objective function).
For passive samplers, monitor is optional, but it can be used to populate the error table with 
the how a system of interest performed in each simulation.

.. testcode::

	from verifai.monitor import specification_monitor

	# The specification must assume specification_monitor class
	class confidence_spec(specification_monitor):
	    def __init__(self):
	        def specification(traj):
	            return traj['yTrue'] == traj['yPred']
	        super().__init__(specification)


Writing a Formal Specification with Metric Temporal Logic
====================================================================
Instead of a customized monitor, users can provide a specification using `metric temporal logic <https://github.com/mvcisback/py-metric-temporal-logic>`_. In such case, users need to use mtl_falsifier instead of generic_falsifier.

.. testcode::
	
	from verifai.falsifier import mtl_falsifier

	specification = ["G(collisioncone0 & collisioncone1 & collisioncone2)"]


Defining Falsifier Parameters
====================================================================

.. testcode::
	
	from dotmap import DotMap
	falsifier_params = DotMap(
		n_iters=1000,   # Number of simulations to run (or None for no limit)
		max_time=None,	# Time limit in seconds, if any
		save_error_table=True,   # Record samples that violated the monitor/specification
		save_good_samples=False,  # Don't record samples that satisfied the monitor/specification
		fal_thres=0.5,    # Monitor return value below which a sample is considered a violation
		sampler_params=None   # optional DotMap of sampler-specific parameters
	)


Setting up Client/Server Communication
====================================================================

VerifAI uses a client/server model to communicate with an external simulator for running tests.
The default `Server` (suitable for use with user-provided clients for new simulators) uses network sockets and can be customized as follows:

.. testcode::

	PORT = 8888
	BUFSIZE = 4096
	MAXREQS = 5

	server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

When performing falsification with dynamic Scenic scenarios, VerifAI communicates with the simulator through Scenic, and a special `ScenicServer` is required: see below for an example.

Instantiating a Falsifier
====================================================================

Setting up a falsifier is a simple matter of combining the pieces above.
For a custom monitor, we can use `generic_falsifier`:

.. testcode::

	from verifai.falsifier import generic_falsifier
	falsifier = generic_falsifier(
		sampler=random_sampler,		# or scenic_sampler, etc. as above
		monitor=confidence_spec(),
		falsifier_params=falsifier_params,
		server_options=server_options
	)

.. testcode::
	:hide:

	falsifier.server.terminate()

For a specification in Metric Temporal Logic, we can use `mtl_falsifier`:

.. testcode::

	from verifai.falsifier import mtl_falsifier
	falsifier = mtl_falsifier(
		sampler=random_sampler,
		specification=specification,
		falsifier_params=falsifier_params,
		server_options=server_options
	)

.. testcode::
	:hide:

	falsifier.server.terminate()

After instantiating either kind of falsifier, it can be run as follows:

.. code:: python

	# Wait for a client to connect, run the simulations, then clean up
	falsifier.run_falsifier()

Dynamic Scenic scenarios can be used with any type of falsifier, but you must specify the `ScenicServer` class (see its documentation for available options).
Monitors will be passed the Scenic :obj:`~scenic.core.simulators.Simulation` object resulting from each simulation:

.. testcode::

	from verifai.scenic_server import ScenicServer

	scenic_sampler = ScenicSampler.fromScenicCode("""\
	model scenic.simulators.newtonian.model
	ego = Object with velocity (0, Range(5, 15))
	other = Object at (5, 0), with velocity (-10, 10)
	terminate after 2 seconds
	record final (distance to other) as dist
	""")

	class scenic_spec(specification_monitor):
		def __init__(self):
			def specification(simulation):
				return simulation.result.records['dist'] > 1
			super().__init__(specification)

	falsifier = generic_falsifier(
		sampler=scenic_sampler,
		monitor=scenic_spec(),
		falsifier_params=DotMap(n_iters=2),
		server_class=ScenicServer
	)
	falsifier.run_falsifier()
