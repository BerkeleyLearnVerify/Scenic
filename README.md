# Scenic

A compiler and scene generator for the Scenic scenario description language, with (partial) interfaces to the Grand Theft Auto V and Webots simulators.
The _src/scenic_ directory contains the package proper, while the _examples_ directory has many examples of Scenic programs.

For a description of the language and some of its applications, see [this paper](https://math.berkeley.edu/~dfremont/papers/PLDI19.pdf).
Scenic was designed and implemented by Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia.

If you have any problems using Scenic, please contact Daniel at <dfremont@berkeley.edu> or submit an issue to [our GitHub repository](https://github.com/BerkeleyLearnVerify/Scenic).

### Contents

* [Installation](#installation)
* [Basic Usage](#basic-usage)
* [Interfacing to GTA V](#interfacing-to-gta-v)
* [Interfacing to New Simulators](#interfacing-to-new-simulators)
* [License](#license)

## Installation

You need Python 3.6 or newer.
The _setup.py_ script will handle installation of dependencies (run `python setup.py install`).
Some scenarios using OpenStreetMap data require the _pyproj_ package and will prompt you if you don't have it (we haven't listed it as a dependency since there seem to be issues building it for Windows at the moment).

## Basic Usage

The top-level interface to Scenic is through the function `scenic.scenarioFromFile`, which compiles the given file into a `Scenario` object (see _scenic/\_\_main\_\_.py_ for an example).
The `scenic` module can also be run as a standalone application, in which case it renders simple schematics of the generated scenes.
You can try this out as follows:

```
python3 -m scenic examples/gta/badlyParkedCar2.sc
```
A window should pop up with a diagram of a scene, in this case generated from the badly-parked car scenario from our GTA case study.
If you close the window, a new scene will be generated.
At least on OS X, you can kill the generator by right-clicking on the graphics window's icon in the Dock and selecting Quit.
Control-C may not work due to how Matplotlib works.
For demos, you can tell the generator to repeatedly generate new scenes after a given delay like so:

```
python3 -m scenic -d 5 examples/gta/bumperToBumper.sc
```

Scenarios for Webots or Guideways work the same way. For example:

```
python3 -m scenic examples/webots/mars/narrowGoal.sc
python3 -m scenic examples/webots/road/crossing.sc
python3 -m scenic examples/guideways/uberCrash.sc
```
There are many more examples of Scenic programs in the _examples_ folder.

## Interfacing to GTA V

Interfacing Scenic to GTA V requires a GTA V plugin, which you can find [here](https://github.com/xyyue/scenic2gta).

## Interfacing to New Simulators

If you want to interface Scenic to a simulator, there are two steps.
The easy part is using the APIs provided by `scenic` to load Scenic programs and generate scenes.
To get an idea of how this works, you can look at _scenic/\_\_main\_\_.py_ to see the source for the standalone application used above.

The more involved step is importing into Scenic information about the virtual world provided by the simulator, e.g. obstacles, regions, and types of objects.
This repository contains four world models you may find helpful as examples, under _scenic/simulators_.
Each consists of a master Scenic module which must be imported by every scenario using that simulator, possibly together with auxiliary Python and Scenic modules.
The models are:

- __webots/mars/mars_model.sc:__ The model for the Mars rover example in Webots. This model is extremely simple and might be a good place to start.

- __gta/gta_model.sc:__ The model for the GTAV world.

- __webots/road/road_model.sc:__ A general model for Webots worlds generated using the Webots OSM importer.

- __guideways/guideways_model.sc__: A prototype model for worlds derived from guideways data from the [Intelligent Intersections Toolkit](https://github.com/ucbtrans/intelligent_intersection).

## License

Scenic is distributed under the 3-Clause BSD License. However, it currently uses the [GPC library](http://www.cs.man.ac.uk/~toby/alan/software/) (through the [Polygon3](https://pypi.org/project/Polygon3/) package), which is free only for non-commercial use.
GPC is used only in the `triangulatePolygon` function in `scenic.core.geometry`, and you can alternatively plug in any algorithm of your choice for triangulation of polygons with holes.
We plan to replace GPC with a BSD-compatible library in the near future.
