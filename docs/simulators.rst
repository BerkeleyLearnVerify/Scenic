..  _simulators:

Supported Simulators
====================

Scenic is designed to be easily interfaced to any simulator (see :ref:`new_simulator`).
On this page we list interfaces that we and others have developed; if you have a new interface, let us know and we'll list it here!

.. contents:: Supported Simulators:
   :local:

Built-in Newtonian Simulator
----------------------------

To enable debugging of dynamic scenarios without having to install an external simulator, Scenic includes a simple Newtonian physics simulator.
The simulator supports scenarios written using the cross-platform :ref:`driving_domain`, and can render top-down views showing the positions of objects relative to the road network.
See the documentation of the `scenic.simulators.newtonian` module for details.


CARLA
-----

Our interface to the `CARLA <https://carla.org/>`_ simulator enables using Scenic to describe autonomous driving scenarios.
The interface supports dynamic scenarios written using the CARLA world model (:obj:`scenic.simulators.carla.model`) as well as scenarios using the cross-platform :ref:`driving_domain`.
To use the interface, please follow these instructions:

1. Install the latest version of CARLA (we've tested versions 0.9.9, 0.9.10, and 0.9.11) from the `CARLA Release Page <https://github.com/carla-simulator/carla/releases>`_.
2. Install Scenic in your Python virtual environment as instructed in :ref:`quickstart`.
3. Within the same virtual environment, install CARLA's Python API by executing the following command:

.. code-block:: console

	$ easy_install /PATH_TO_CARLA_FOLDER/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg

The exact name of the ``.egg`` file may vary depending on the version of CARLA you installed; make sure to use the file for Python 3, not 2.
You may get an error message saying ``Could not find suitable distribution``, which you can ignore.
Instead, check that the ``carla`` package was correctly installed by running :command:`pip show carla`.

To start CARLA, run the command :command:`./CarlaUE4.sh` in your CARLA folder.
Once CARLA is running, you can run dynamic Scenic scenarios following the instructions in :ref:`the dynamics tutorial <dynamics_running_examples>`.

.. note::

	If you are using Scenic 1.x, there is an older CARLA interface which works with static Scenic scenarios and so requires agent behaviors to be written in plain Python. This interface is part of the VerifAI toolkit; documentation and examples can be found in the `VerifAI repository`_.


Grand Theft Auto V
------------------

The interface to `Grand Theft Auto V <https://www.rockstargames.com/V/>`_, used in `our PLDI paper`_, allows Scenic to position cars within the game as well as to control the time of day and weather conditions.
Many examples using the interface (including all scenarios from the paper) can be found in :file:`examples/gta`.
See the paper and `scenic.simulators.gta` for documentation.

Importing scenes into GTA V and capturing rendered images requires a GTA V plugin, which you can find `here <https://github.com/xyyue/scenic2gta>`__.


LGSVL
-----

We have developed an interface to the LGSVL simulator for autonomous driving, used in our `ITSC 2020 <ITSC2020>`__ paper.
The interface supports dynamic scenarios written using the LGSVL world model (:obj:`scenic.simulators.lgsvl.model`) as well as scenarios using the cross-platform :ref:`driving_domain`.

To use the interface, first install the simulator from the `LGSVL Simulator <https://www.lgsvlsimulator.com/>`_ website.
Then, within the Python virtual environment where you installed Scenic, install LGSVL's Python API package from `source <https://github.com/lgsvl/PythonAPI>`__.

An example of how to run a dynamic Scenic scenario in LGSVL is given in :ref:`dynamics`.

Webots
------

We have several interfaces to the `Webots robotics simulator <https://cyberbotics.com/>`_, for different use cases.
Our main interface provides a generic world model that can be used with any Webots world and supports dynamic scenarios.
See the :file:`examples/webots` folder for example Scenic scenarios and Webots worlds using this interface, and `scenic.simulators.webots` for documentation.

Scenic also includes several more specialized world models for use with Webots:

	* A model for the Mars rover example used in `our PLDI paper`_.
	  This model is extremely simple and might be a good baseline for developing your own model.
	  See the examples in :file:`examples/webots/mars` (including a dynamic version of the scenario from the paper) and the documentation of `scenic.simulators.webots.mars` for details.

	* A general model for traffic scenarios, used in `our VerifAI paper`_.
	  Examples using this model can be found in the `VerifAI repository`_; see also the documentation of `scenic.simulators.webots.road`.

	* A more specific model for traffic scenarios at intersections, using guideways from the `Intelligent Intersections Toolkit <https://github.com/ucbtrans/intelligent_intersection>`_.
	  See the examples in :file:`examples/webots/guideways` and the documentation of `scenic.simulators.webots.guideways` for details.

.. note::

	The last two models above, and the example ``.wbt`` files for them, were written for the R2018 version of Webots.
	Relatively minor changes would be required to make them work with the newer `open source versions of Webots <https://github.com/cyberbotics/webots>`_.
	We may get around to porting them eventually; we'd also gladly accept a pull request!


X-Plane
-------

Our interface to the `X-Plane flight simulator <https://www.x-plane.com>`_ enables using Scenic to describe aircraft taxiing scenarios.
This interface is part of the VerifAI toolkit; documentation and examples can be found in the `VerifAI repository`_.

.. _our PLDI paper: https://arxiv.org/abs/1809.09310

.. _our VerifAI paper: https://doi.org/10.1007/978-3-030-25540-4_25

.. _VerifAI repository: https://github.com/BerkeleyLearnVerify/VerifAI
