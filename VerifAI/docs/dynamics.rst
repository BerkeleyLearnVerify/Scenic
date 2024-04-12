#######################################
Interfacing VerifAI with Dynamic Scenic 
#######################################

Setting up Client/Server Communication
====================================================================

The syntax for setting up a VerifAI falsifier with a dynamic Scenic script is very similar to the setup outlined in :doc:`Basic Usage <basic_usage>`.
The major difference is that the maximum number of timesteps to run each simulation must be provided as an argument to the falsifier:

.. code:: python

    falsifier_params = DotMap(
        n_iters=None,
        save_error_table=True,
        save_safe_table=True,
        max_time=60,
    )
    server_options = DotMap(maxSteps=300, verbosity=0) # maximum number of timesteps to run each simulation.

For an example of using dynamic Scenic with VerifAI, see the :file:`examples/multi_objective` folder.
