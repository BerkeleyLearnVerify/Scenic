Supported Simulators
====================

Scenic is designed to be easily interfaced to any simulator (see :doc:`new_simulator`).
On this page we list interfaces that we and others have developed; if you have a new interface, let us know and we'll list it here!

.. contents:: Supported Simulators:
   :local:

Grand Theft Auto V
------------------

The interface to `Grand Theft Auto V <https://www.rockstargames.com/V/>`_, used in `our PLDI paper`_, allows Scenic to position cars within the game as well as to control the time of day and weather conditions.
Many examples using the interface (including all scenarios from the paper) can be found in :file:`examples/gta`.
See the paper and `scenic.simulators.gta` for documentation.

Importing scenes into GTA V and capturing rendered images requires a GTA V plugin, which you can find `here <https://github.com/xyyue/scenic2gta>`_.


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

**Note:** Our interfaces were written for the R2018 version of Webots, which is not free but has lower hardware requirements than R2019.
Relatively minor changes would be required to make our interfaces work with the newer `open source versions of Webots <https://github.com/cyberbotics/webots>`_.
We may get around to porting them eventually; we'd also gladly accept a pull request!

X-Plane
-------

bar

.. _our PLDI paper: https://arxiv.org/abs/1809.09310

.. _our VerifAI paper: https://doi.org/10.1007/978-3-030-25540-4_25

.. _VerifAI repository: https://github.com/BerkeleyLearnVerify/VerifAI
