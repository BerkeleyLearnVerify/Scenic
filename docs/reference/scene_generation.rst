
.. _scene generation:

****************
Scene Generation
****************

The "output" of a Scenic program has two parts: a *scene* describing a configuration of physical objects, and a *policy* defining how those objects behave over time.
The latter is relevant only for running dynamic simulations from a Scenic program, and is discussed in our page on `dynamic scenario semantics`.
In this page, we describe how scenes are generated from a Scenic program.

In Scenic, a scene consists of the following data:

	* a set of `objects <Object>` present in the scene (one of which *may* be designated the :scenic:`ego` object);
	* concrete values for all of the properties of these objects, such as :prop:`position`, :prop:`heading`, etc.;
	* concrete values for each :term:`global parameter`.

A Scenic program defines a probability distribution over such scenes in the usual way for imperative probabilistic programming languages with constraints (often called *observations*).
Running the program ignoring any :keyword:`require` statements and making random choices whenever a distribution is evaluated yields a distribution over possible executions of the program and therefore over generated scenes.
Then any executions which violate a :keyword:`require` condition are discarded, normalizing the probabilities of the remaining executions.

The Scenic tool samples from this distribution using rejection sampling: repeatedly sampling scenes until one is found which satisfies the requirements.
This approach has the advantage of allowing arbitrarily-complex requirements and sampling from the exact distribution we want.
However, if the requirements have a low probability of being satisfied, it may take many iterations to find a valid scene: in the worst case, if the requirements cannot be satisfied, rejection sampling will run forever (although the `Scenario.generate` function imposes a finite limit on the number of iterations by default).
To reduce the number of iterations required in some common cases, Scenic applies several "pruning" techniques to exclude parts of the scene space which violate the requirements ahead of time (this is done during compilation; see `our paper <publications>` for details).
The scene generation procedure then works as follows:

1. Decide which user-defined requirements will be enforced for this sample (`soft requirements <soft-requirements>` have only some probability of being required).
2. Invoke the external sampler to sample any :term:`external parameters`.
3. Sample values for all distributions defined in the scene (all expressions which have random values, represented internally as `Distribution` objects).
4. Check if the sampled values satisfy the built-in and user-defined requirements: if not, reject the sample and repeat from step (2).
