..  _distributions:

***********************
Distributions Reference
***********************

Scenic provides functions for sampling from various types of probability distributions, and it is also possible to define custom types of distributions.

If you want to sample multiple times from the same distribution (for example if the distribution is passed as an argument to a helper function), you can use the `resample` function.


Built-in Distributions
======================

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

.. _TruncatedNormal({mean}, {stdDev}, {low}, {high}):

TruncatedNormal(*mean*, *stdDev*, *low*, *high*)
------------------------------------------------
Normal distribution as above, but truncated to the given window.

.. _Uniform({value}, {...}):

Uniform(*value*, ...)
---------------------
Uniform over a finite set of values. The Uniform distribution can also be used to uniformly select over a list of unknown length. This can be done using the unpacking operator (which supports distributions over lists) as follows: :scenic:`Uniform(*list)`.

.. _DiscreteDistr:

Discrete({*value*: *weight*, ... })
-----------------------------------
Discrete distribution over a finite set of values, with weights (which need not add up to 1).
Each value is sampled with probability proportional to its weight.

.. _uniform_in_region:

Uniform Distribution over a Region
----------------------------------
Scenic can also sample points uniformly at random from a `Region`, using the :sampref:`in {region}` and :sampref:`on {region}` specifiers.
Most subclasses of `Region` support random sampling.
A few regions, such as the `everywhere` region representing all space, cannot be sampled from since a uniform distribution over them does not exist.

Defining Custom Distributions
=============================

If necessary, custom distributions may be implemented by subclassing the `Distribution` class.
New subclasses must implement the :obj:`~scenic.core.distributions.Samplable.sampleGiven` method, which computes a random sample from the distribution given values for its dependencies (if any).
See :obj:`~scenic.core.distributions.Range` (the implementation of the uniform distribution over a range of real numbers) for a simple example of how to define a subclass.
Additional functionality can be enabled by implementing the optional `clone`, `bucket`, and :obj:`~scenic.core.distributions.Distribution.supportInterval` methods; see their documentation for details.
