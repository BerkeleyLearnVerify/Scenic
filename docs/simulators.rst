..  _simulators:

Supported Simulators
====================

Scenic is designed to be easily interfaced to any simulator (see :ref:`new_simulator`).
On this page we list interfaces that we and others have developed; if you have a new interface, let us know and we'll list it here!

.. contents:: Supported Simulators:
   :local:


CARLA
-----

Our interface to the `CARLA <https://carla.org/>`_ simulator enables using Scenic to describe autonomous driving scenarios.
This interface is part of the VerifAI toolkit; documentation and examples can be found in the `VerifAI repository`_ (the Scenic repository also has several other example scenarios). Please follow the following instruction to install Carla and interface it with Scenic. 

1. Install the latest Carla (tested version 0.9.9 so far) from the `Release Page <https://github.com/carla-simulator/carla/releases>`_.
2. Install Scenic in your python virtual environment as instructed in :ref:`quickstart`.
3. Within the same python virtual environment, please install Carla's Python API by executing the following command in the terminal commandline::

	easy_install \PATH_TO_CARLA_FOLDER\PythonAPI\carla\dist\carla-0.9.9-py3.7-linux-x86_64.egg

The exact name of the ``.egg`` file may vary depending on minor version of the Carla you installed. If different, please use the name of the egg file for python version 3. The PythonAPI package should be installed as ``carla`` package. For validation, if you execute ``pip list`` on terminal ``carla`` should be listed.

To instantiate the Carla simulator, please run ``.\CarlaUE4.sh`` in your terminal. This bash file should be located at the root of your Carla folder. 
If not, please search for it within your Carla folder. Once the Carla simulator is instantiated, please open another terminal window and follow the :ref:`quickstart` instructions.


Grand Theft Auto V
------------------

The interface to `Grand Theft Auto V <https://www.rockstargames.com/V/>`_, used in `our PLDI paper`_, allows Scenic to position cars within the game as well as to control the time of day and weather conditions.
Many examples using the interface (including all scenarios from the paper) can be found in :file:`examples/gta`.
See the paper and `scenic.simulators.gta` for documentation.

Importing scenes into GTA V and capturing rendered images requires a GTA V plugin, which you can find `here <https://github.com/xyyue/scenic2gta>`__.


LGSVL
-----
We have developed an interface to the LGSVL simulator for autonomous driving, used in our `ITSC 2020 <ITSC2020>`__ paper. Please install the simulator from the `LGSVL Simulator <https://www.lgsvlsimulator.com/>`_ website. Then, within your python virutal environment where you installed Scenic, please also install LGSVL's python API package from `source <https://github.com/lgsvl/PythonAPI>`__. 


Webots
------

We have several interfaces to the `Webots robotics simulator <https://cyberbotics.com/>`_, for different use cases.

	* An interface for the Mars rover example used in `our PLDI paper`_.
	  This interface is extremely simple and might be a good baseline for developing your own interface.
	  See the examples in :file:`examples/webots/mars` and the documentation of `scenic.simulators.webots.mars` for details.

	* A general interface for traffic scenarios, used in `our VerifAI paper`_.
	  Examples using this interface can be found in the `VerifAI repository`_; see also the documentation of `scenic.simulators.webots.road`.

	* A more specific interface for traffic scenarios at intersections, using guideways from the `Intelligent Intersections Toolkit <https://github.com/ucbtrans/intelligent_intersection>`_.
	  See the examples in :file:`examples/webots/guideways` and the documentation of `scenic.simulators.webots.guideways` for details.

.. note::

	Our interfaces were written for the R2018 version of Webots, which is not free but has lower hardware requirements than R2019.
	Relatively minor changes would be required to make our interfaces work with the newer `open source versions of Webots <https://github.com/cyberbotics/webots>`_.
	We may get around to porting them eventually; we'd also gladly accept a pull request!


X-Plane
-------

Our interface to the `X-Plane flight simulator <https://www.x-plane.com>`_ enables using Scenic to describe aircraft taxiing scenarios.
This interface is part of the VerifAI toolkit; documentation and examples can be found in the `VerifAI repository`_.

.. _our PLDI paper: https://arxiv.org/abs/1809.09310

.. _our VerifAI paper: https://doi.org/10.1007/978-3-030-25540-4_25

.. _VerifAI repository: https://github.com/BerkeleyLearnVerify/VerifAI
