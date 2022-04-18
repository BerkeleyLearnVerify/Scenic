# Webots Examples

This folder contains example Scenic scenarios for use with the Webots robotics simulator.

In the **generic** folder we provide several Webots worlds (``.wbt`` files) demonstrating these scenarios with Scenic's [generic Webots interface](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.simulators.webots.simulator.html). To run these, either install Scenic in the version of Python used by Webots or launch Webots from inside a virtual environment where Scenic is installed (the latter works as of Webots R2022a with Python 3.7; newer Python versions may crash due to a bug in Webots, unfortunately), then open one of the ``.wbt`` files. Starting the simulation will automatically start Scenic and repeatedly generate scenarios.

__Licensing Note:__ The ``mars.wbt`` file is a modified version of the [Sojourner Rover example](https://cyberbotics.com/doc/guide/sojourner#sojourner-wbt) included in Webots. The original was written by Nicolas Uebelhart and is copyrighted by Cyberbotics Ltd. under the [Webots asset license](https://cyberbotics.com/webots_assets_license). Under the terms of that license, the modified version remains property of Cyberbotics; however, all other files in this directory are covered by the Scenic license. In particular, please feel free to model your own supervisor implementation on ``generic/controllers/scenic_supervisor.py``.
