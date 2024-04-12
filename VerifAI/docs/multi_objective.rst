#############################
Multi-Objective Falsification
#############################

Specification of Objectives
===========================

VerifAI now provides the ability to run falsification on multiple metrics at the same time. For example, consider the following VerifAI monitor:

.. code:: python

    from verifai.monitor import multi_objective_monitor

    """
    Example of multi-objective specification. This monitor specifies that the ego vehicle
    must stay at least 5 meters away from each other vehicle in the scenario.
    """
    class distance_multi(multi_objective_monitor):
        def __init__(self, num_objectives=1):
            priority_graph = nx.DiGraph()
            self.num_objectives = num_objectives
            priority_graph.add_edge(0, 2)
            priority_graph.add_edge(1, 3)
            priority_graph.add_edge(2, 4)
            priority_graph.add_edge(3, 4)
            print(f'Initialized priority graph with {self.num_objectives} objectives')
            def specification(simulation):
                positions = np.array(simulation.result.trajectory)
                distances = positions[:, [0], :] - positions[:, 1:, :]
                distances = np.linalg.norm(distances, axis=2)
                rho = np.min(distances, axis=0) - 5
                return rho
            
            super().__init__(specification, priority_graph)

The monitor computes the distance between the ego vehicle and every other vehicle in the scenario and returns all of these distances. Note that the monitor class extends the ``multi_objective_monitor`` class, written specifically for vector-valued objectives.
Additionally, a *rulebook* is defined in the ``priority_graph`` variable, which is a partial ordering over the metrics providing some pairwise information about which metrics are considered most important. This rulebook is encoded as a directed acyclic graph (DAG) using the `NetworkX library <https://networkx.org/>`_.

Samplers Supporting Multiple Objectives
=======================================

To mitigate issues with sensitivity to results of initial samples, VerifAI implements the *multi-armed bandit sampler*, an active sampler which uses the Upper Confidence Bound (UCB) algorithm to tradeoff exploration of new regions of the feature space as well as exploitation of previously found counterexamples. To use the multi-armed bandit sampler, either use the ``MultiArmedBanditSampler`` class or, if using Scenic, add the line

.. code:: python

    param verifaiSamplerType = 'mab'

For an example of using multi-objective sampling, see the :file:`examples/multi_objective` folder.
