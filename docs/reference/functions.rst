..  _functions:

****************************
Built in Functions Reference
****************************

.. _gen_lifted_funcs:

General Lifted Functions
------------------------

The following functions work exactly the same as their Python counterparts except that they have been modified to accept and return distributions:

* `sin`, `cos`, `hypot`
* `max`, `min`
* `str`

.. _filter_func:

filter
------

The `filter` function also works like its Python counterparts except it is now able to operate over distributions. However, the `filter` function is of particular interest because it can be used to work around Scenic's lack of support for randomized control flow in certain cases. Consider the situation where you would like to sample from a random variable, and then sample from another random variable based on the first result. Should you attempt to do this by sampling the first and using an if statement to select the second result, Scenic will throw an error. However, we can instead use the filter operation to get similar behavior. For an example see``examples/driving/OAS_scenarios/oas_scenario_28.scenic``.

.. _resample_func:

resample
--------
The `resample` function takes a distribution and samples a new value from it. This is useful in cases where you have a complicated distribution that you want multiple values from.

.. _localPath_func:

localPath
---------
The `localPath` function takes a relative path and converts it to an absolute path, rooted at the directory containing the ``.scenic`` file where it is used.

.. _verbosePrint_func:

verbosePrint
------------
The `verbosePrint` function operates like `print` except that it you can specify at what verbosity level it should actually print.

.. _simulation_func:

simulation
----------
The `simulation` function returns the current simulation object.
