"""Domain for driving scenarios.

The :doc:`world model <scenic.domains.driving.model>` defines Scenic classes for cars,
pedestrians, etc., actions for dynamic agents which walk or drive, as well as simple
behaviors like lane-following. Scenarios for the driving domain should import the model
as follows:

.. code-block:: scenic

    model scenic.domains.driving.model

Scenarios written for the driving domain should work without changes [#f1]_ in any of the
following simulators:

    * CARLA, using the model :doc:`scenic.simulators.carla.model`
    * LGSVL, using the model :doc:`scenic.simulators.lgsvl.model`
    * the built-in Newtonian simulator, using the model
      :doc:`scenic.simulators.newtonian.model`

For example, the :file:`examples/driving/badlyParkedCarPullingIn.scenic` scenario is
written for the driving domain and can be run in multiple simulators:

    * no simulator, for viewing the initial scene:

        .. code-block:: console

            $ scenic examples/driving/badlyParkedCarPullingIn.scenic

    * the built-in Newtonian simulator, for quick debugging without having to install an
      external simulator:

        .. code-block:: console

            $ scenic -S --model scenic.simulators.newtonian.model \\
                examples/driving/badlyParkedCarPullingIn.scenic

    * CARLA, using the default map specified in the scenario:

        .. code-block:: console

            $ scenic -S --model scenic.simulators.carla.model \\
                examples/driving/badlyParkedCarPullingIn.scenic

    * LGSVL, specifying a map which it supports:

        .. code-block:: console

            $ scenic -S --model scenic.simulators.lgsvl.model \\
                --param map tests/formats/opendrive/maps/LGSVL/borregasave.xodr \\
                --param lgsvl_map BorregasAve \\
                examples/driving/badlyParkedCarPullingIn.scenic

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree:

   model
   behaviors
   actions
   roads
   controllers
   workspace

.. rubric:: Footnotes

.. [#f1] Assuming the simulator supports the selected map. If necessary, the map may be
    changed from the command line using the :option:`--param` option; see the
    :doc:`model documentation <scenic.domains.driving.model>` for details.
"""
