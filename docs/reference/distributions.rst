..  _distributions:

***********************
Distributions Reference
***********************

.. _Range({low}, {high}):

Range(*low*, *high*)
--------------------
Uniformly-distributed real number in the interval.

.. _DiscreteRange({low}, {high}):

DiscreteRange(*low*, *high*)
----------------------------
Uniformly-distributed integer in the (fixed) interval.

.. _Normal({mean}, {stdDev}):

Normal(*mean*, *stdDev*)
------------------------
Normal distribution with the given mean and standard deviation.

.. _Uniform({value}, {...}):

Uniform(*value*, ...)
---------------------
Uniform over a finite set of values. The Uniform distribution can also be used to uniformly select over a list of unknown length. This can be done using the unpacking operator (which supports distributions over lists) as follows: ``Uniform(*list)``.

.. _DiscreteDistr:

Discrete({*value*: *weight*, ... })
-----------------------------------
Discrete distribution over a finite set of values, with weights (which need not add up to 1).
Each value is sampled with probability proportional to its weight.

.. note::
    
    Distributions can be sampled multiple times using the `resample` function.
