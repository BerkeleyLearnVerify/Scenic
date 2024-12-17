..  _simulators:

********************
Supported Simulators
********************

Scenic is designed to be easily interfaced to any simulator (see :ref:`new_simulator`).
On this page we list interfaces that we and others have developed; if you have a new interface, let us know and we'll list it here!

Note that not every interface supports all Scenic features: in particular, some interfaces do not support dynamic scenarios.
See the individual entries for details on each interface's capabilities and how to set it up.

.. contents:: List of Simulators
   :local:



Currently Supported
===================
AirSim
------

The interface to AirSim (https://github.com/microsoft/AirSim/tree/main) enables
the user to create dynamic scenarios using any AirSim binary.

**Installation**
~~~~~~~~~~~~~~~~

1. Begin by installing Scenic, which you can find detailed instructions
   for
   `here <https://scenic-lang.readthedocs.io/en/latest/quickstart.html>`__.

2. Additionally, you’ll need to install the following dependencies in
   your python environment:

   (Note: In the past, some users have encountered issues with AirSim
   indicating that numpy is not installed. If you run into this problem,
   consider utilizing a conda environment, which should resolve the
   issue.)

.. code:: bash

   pip install msgpack-rpc-python airsim promise numpy

If you’re using px4, you will also need:

.. code:: jsx

   pip install kconfiglib

Example Usage
~~~~~~~~~~~~~

Generate World Info
^^^^^^^^^^^^^^^^^^^

Before running Scenic, it’s essential to generate world information. To
achieve this, utilize the scripts located at
:file:`src/scenic/simulators/airsim/generators/`

Using Unreal Engine (recommended)
"""""""""""""""""""""""""""""""""

Enable these plugins in Unreal Engine:
- Python Editor Script Plugin
- Editor Scripting Utilities

First, run the :file:`generateUE4WorldInfo.py` or `generateUE5WorldInfo.py` script inside unreal engine by simply entering in the file path in the engine’s python console.
Then run `generateWorldInfoFromUnrealWorldInfo.py` script

First, run the :file:`generateUnrealWorldInfo.py` script inside Unreal Engine by
simply entering in the file path in the engine’s python console.

Then run the :file:`generateWorldInfoFromUnrealWorldInfo.py` script

Using Airsim
""""""""""""

Note: The newest version of airsim is required to to run :file:`generateWorldInfo.py`. Some precompiled binaires from https://github.com/microsoft/AirSim/releases may not be supported.

Run the :file:`generateWorldInfo.py` script

**Configure AirSim Settings**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before running Scenic, we also need to ensure that AirSim has been
launched with the appropriate settings. The choice of settings file
depends on your simulation requirements. To generate a basic settings
file tailored to your needs, utilize the
`generateAirsimSettings.py <http://generateairsimsettings.py/>`__ script
found at
Scenic/src/scenic/simulators/airsim/generators/generateAirsimSettings.py
. Use the -h flag for detailed usage instructions.

Linux Example:

.. code:: bash

   python Scenic/src/scenic/simulators/airsim/generators/generateAirsimSettings.py -o airsimsettings.json  --maxdrones 5

Once you’ve generated a settings file, you can run AirSim with it. For
Linux, it might look like this:

Linux Example:

.. code:: bash

   $ Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh -settings="Scenic/src/scenic/simulators/airsim/objs/cubes/airsimSettings.json"

Running Scenic
^^^^^^^^^^^^^^

Once AirSim is up and running with the appropriate settings, you can
proceed to run your Scenic code:

.. code:: bash

   scenic Scenic/examples/airsim/multi_drone.scenic --simulate

That’s all there is to it! This sequence of steps will set up and
execute your Airsim simulations using Scenic.

Using **MAVSDK-Python** with AirSim
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Obtaining PX4**
^^^^^^^^^^^^^^^^^

To utilize MAVSDK-Python with AirSim, clone the PX4-Autopilot repository
using the following link: https://github.com/PX4/PX4-Autopilot

**Concurrent Operation of AirSim and PX4**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Running AirSim with PX4 requires both to be running at the same time.
Follow the steps below to get both running simultaneously and compatible
with each other.

Running AirSim
^^^^^^^^^^^^^^

Before launching AirSim, it is important to ensure it is set up to
communicate with PX4.

You need to run AirSim with a PX4Multirotor as at least 1 of your
vehicles specified in the settings with specific configurations for the
drone’s port based on your desired running environment. For running
AirSim with PX4 on the same machine on the default port 4560, the
following AirSim settings should suffice.

.. code:: jsx

   {
       "SettingsVersion": 1.2,
       "SimMode": "Multirotor",
       "ClockType": "SteppableClock",
       "Vehicles": {
           "PX4": {
               "VehicleType": "PX4Multirotor",
               "UseSerial": false,
               "LockStep": true,
               "UseTcp": true,
               "TcpPort": 4560,
               "ControlIp": "local",
               "ControlPortLocal": 14540,
               "ControlPortRemote": 14580,
               "LocalHostIpLocal": "127.0.0.1",
               "LocalHostIp": "127.0.0.1",
               "QgcHostIp": "127.0.0.1",
               "QgcPort": 14550,
               "Sensors": {
                   "Barometer": {
                       "SensorType": 1,
                       "Enabled": true,
                       "PressureFactorSigma": 0.0001825
                   }
               },
               "Parameters": {
                   "LPE_LAT": 30.0368,
                   "LPE_LON": 51.2090
               }
           }
       }
   }

Running PX4
^^^^^^^^^^^

In the cloned PX4 directory, run the makefile with the correct settings
by running the following command in the terminal. This will start the
PX4 firmware in SITL mode.

.. code:: jsx

   make px4_sitl_default none_iris

After this step, you can run Scenic files normally as shown in the
“Running Scenic” section above.


Built-in Newtonian Simulator
----------------------------

To enable debugging of dynamic scenarios without having to install an external simulator, Scenic includes a simple 2D Newtonian physics simulator.
The simulator supports scenarios written using the cross-platform :ref:`driving_domain`, and can render top-down views showing the positions of objects relative to the road network.
See the documentation of the `scenic.simulators.newtonian` module for details.


CARLA
-----

Our interface to the `CARLA <https://carla.org/>`_ simulator enables using Scenic to describe autonomous driving scenarios.
The interface supports dynamic scenarios written using the CARLA world model (:obj:`scenic.simulators.carla.model`) as well as scenarios using the cross-platform :ref:`driving_domain`.
To use the interface, please follow these instructions:

1. Install the latest version of CARLA (we've tested versions 0.9.9 through 0.9.14) from the `CARLA Release Page <https://github.com/carla-simulator/carla/releases>`_.
   Note that CARLA currently only supports Linux and Windows.
2. Install Scenic in your Python virtual environment as instructed in :ref:`quickstart`.
3. Within the same virtual environment, install CARLA's Python API.
   How to do this depends on the CARLA version and whether you built it from source:

	.. tabs::

		.. tab:: 0.9.12+

			Run the following command, replacing ``X.Y.Z`` with the version of CARLA you installed:

			.. code-block:: text

				python -m pip install carla==X.Y.Z

		.. tab:: Older Versions

			For older versions of CARLA, you'll need to install its Python API from the provided ``.egg`` file.
			If your system has the :command:`easy_install` command, you can run:

			.. code-block:: text

				easy_install /PATH_TO_CARLA_FOLDER/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg

			The exact name of the ``.egg`` file may vary depending on the version of CARLA you installed; make sure to use the file for Python 3, not 2.
			You may get an error message saying ``Could not find suitable distribution``, which you can ignore.

			The :command:`easy_install` command is deprecated and may not exist if you have a newer version of Python.
			In that case, you can try setting your ``PYTHONPATH`` environment variable to include the egg with a command like:

			.. code-block:: text

				export PYTHONPATH=/PATH_TO_CARLA_FOLDER/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg

		.. tab:: Built from Source

			If you built CARLA from source, the process is more involved: see the detailed instructions `here <https://carla.readthedocs.io/en/latest/start_quickstart/#install-client-library>`__.

You can check that the ``carla`` package was correctly installed by running :command:`python -c 'import carla'`: if it prints ``No module named 'carla'``, the installation didn't work.
We suggest upgrading to a newer version of CARLA so that you can use :command:`pip` to install the Python API.

To start CARLA, run the command :command:`./CarlaUE4.sh` in your CARLA folder.
Once CARLA is running, you can run dynamic Scenic scenarios following the instructions in :ref:`the dynamics tutorial <dynamics_running_examples>`.


Grand Theft Auto V
------------------

The interface to `Grand Theft Auto V <https://www.rockstargames.com/V/>`_, used in `our PLDI paper`_, allows Scenic to position cars within the game as well as to control the time of day and weather conditions.
Many examples using the interface (including all scenarios from the paper) can be found in :file:`examples/gta`.
See the paper and `scenic.simulators.gta` for documentation.

Importing scenes into GTA V and capturing rendered images requires a GTA V plugin, which you can find `here <https://github.com/xyyue/scenic2gta>`__.


Webots
------

We have several interfaces to the `Webots robotics simulator <https://cyberbotics.com/>`_, for different use cases.
Our main interface provides a generic world model that can be used with any Webots world and supports dynamic scenarios.
See the :file:`examples/webots` folder for example Scenic scenarios and Webots worlds using this interface, and `scenic.simulators.webots` for documentation.

Scenic also includes more specialized world models for use with Webots:

	* A general model for traffic scenarios, used in `our VerifAI paper`_.
	  Examples using this model can be found in the `VerifAI repository`_; see also the documentation of `scenic.simulators.webots.road`.

.. note::

	The last model above, and the example ``.wbt`` files for it, was written for the R2018 version of Webots.
	Relatively minor changes would be required to make it work with the newer `open source versions of Webots <https://github.com/cyberbotics/webots>`_.
	We may get around to porting them eventually; we'd also gladly accept a pull request!

.. _xplane:

X-Plane
-------

Our interface to the `X-Plane flight simulator <https://www.x-plane.com>`_ enables using Scenic to describe aircraft taxiing scenarios.
This interface is part of the VerifAI toolkit; documentation and examples can be found in the `VerifAI repository`_.

.. _our PLDI paper: https://arxiv.org/abs/1809.09310

.. _our VerifAI paper: https://doi.org/10.1007/978-3-030-25540-4_25

.. _VerifAI repository: https://github.com/BerkeleyLearnVerify/VerifAI




Deprecated
==========

Scenic previously provided interfaces to these simulators, but no longer does.
See individual entries for the last version of Scenic providing the interface and the reason it is no longer supported.

LGSVL
-----

The LGSVL simulator (a.k.a. SVL Simulator) was deprecated in Scenic 3.0, with the last version of Scenic supporting this simulator being 2.1. The original simulator is no longer usable due to LG shutting down its cloud service, but we are open to a PR targeting one of its forks.
