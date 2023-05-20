# Webots Generic Examples

An example showing how to use Scenic to generate training data for an autonomous car. In this example the ego car is approaching an intersection where it has an obligation to yield, with another car crossing the intersection. At regular intervals the ego car's camera output will be saved, and tagged with whether or not the crossing car is visible or not visible.

First navigate to `controllers/create_avoid_obstacles` and run `make` (you may need to first set the Webots environment variable as shown [here](https://cyberbotics.com/doc/guide/compiling-controllers-in-a-terminal)). Then run the scenario using the `worlds/city_intersection.wbt` file in webots and play the simulation by pressing one of the buttons at the top (we recommend "Run the simulation as fast as possible" to maximize speed). After Webots closes (indicating the simulations has completed), look in the `images` directory for the tagged images.

These examples are intended to be run **without** the ``--2d`` flag.
