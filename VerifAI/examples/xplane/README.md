# VerifAI Interface to X-Plane

This interface allows VerifAI to run experiments in the X-Plane flight simulator.

## Installation


### Installing X-Plane

You can get X-Plane from its [website](https://www.x-plane.com).
It supports Linux, Windows, and Mac OS X.
Our interface requires X-Plane 9 or later.

### Installing XPlaneConnect

Next, build the X-Plane plugin in our [fork](https://github.com/johnathanchiu/XPlaneConnect) of XPlaneConnect (the few changes we made should eventually be merged into XPlaneConnect and the fork will no longer be necessary).
The plugin is in the _xpcPlugin_ folder.
The build process varies by platform: for OS X, you can use the Xcode project included there; for Linux and Windows, follow the instructions [here](https://developer.x-plane.com/article/building-and-installing-plugins/#Compiling_Plugins).

Building the plugin will result in an `.xpl` file (a dynamic library), which you should place in the _Resources/plugins_ folder of your X-Plane installation.

Finally, the Python client of XPlaneConnect does not have an installation script, so you will need to copy it here.
Go into _Python3/src_ in our fork, and copy the `xpc.py` file into this directory.

### Optional Plugins

While not necessary for using the interface, you may find the [XCamera plugin](https://www.stickandrudderstudios.com/x-camera/download-x-camera/) helpful if you want to change the position of the camera.

## Basic Usage

### API and Command-Line Functions

The interface defines classes `XPlaneFalsifier` and `XPlaneServer` which specialize the VerifAI classes `mtl_falsifier` and `Server` and can be used with any of VerifAI samplers.
For convenience, you can also run the interface as a script, which will cause it to perform falsification from a given Scenic scenario.
Here is the procedure in more detail:

1. Write a Scenic program defining the scenario of interest. See `example.scenic` for an example.

2. Specify the parameters of the runway the scenario should be implemented on in `runway.yaml` and `elevations.csv`. The example files we have included do this for KMWH (Grant County International Airport), Runway 4. To change the runway, see the next section.

3. Specify the experiment configuration (input and output files, the specification to falsify, etc.) in `config.yaml`.

4. Start a new flight in X-Plane; make sure to change the airport and runway as needed to match `runway.yaml` (e.g. for the default, search for KMWH in the airport list, select it, then press "Configure" and select Runway 04 in the runway list).

5. Run `python -m verifai.simulators.xplane.server`. The results of the tests will be written to files as specified in `config.yaml`; to print this information to the console, increase the verbosity by adding the option `-v 1`.

### Scenic Scenarios for X-Plane

The interface currently supports only scenarios with a single object, namely the `Plane`.
This can be positioned as usual in Scenic; by default, the coordinate system has the Scenic Y-axis aligned with the runway, so that `Plane at 5 @ 1000` would position the plane 5 meters to the right of the centerline and 1 km down the length of the runway.

Arbitrary [X-Plane datarefs](https://developer.x-plane.com/datarefs/) can be controlled by creating a Scenic global parameter with the same name in quotes.
For example, in `example.scenic` we write

```
param 'sim/weather/cloud_type[0]' = Uniform(0, 1, 2, 3, 4, 5)
```

give the `sim/weather/cloud_type[0]` dataref a uniform distribution over the 6 discrete cloud types supported by X-Plane (see the dataref's [documentation](https://developer.x-plane.com/datarefs/) for details).

Finally, the interface reads the global parameters `setup_time` and `simulation_length` to control the amounts of time respectively spent in waiting for X-Plane to stabilize (since when clouds are changed, for example, there is some delay before X-Plane responds) and for running the actual simulations.

## Advanced Usage

### Using a Custom Controller

By default, the interface uses a very simple controller for steering the plane which we provide as an example.
The controller can be found in `verifai.simulators.xplane.utils.controller`.
To use a custom controller (e.g. TaxiNet in our experiments), you can overwrite the function `control` in this module.
If you are creating the `XPlaneServer` (or `XPlaneFalsifier`) directly, the desired controller function can be passed in using the `controller` server option.

### Changing the Airport/Runway

Since the X-Plane API does not expose much information about the runway, changing the airport or runway is currently an onerous manual process.
There are two parts: first, you specify the endpoints of the runway and the desired Scenic coordinate system in `runway.yaml`.
Second, you must provide a sequence of XYZ positions (in X-Plane's OpenGL coordinates) along the length of the runway in `elevations.csv`: this is used to account for the runway not being exactly flat.