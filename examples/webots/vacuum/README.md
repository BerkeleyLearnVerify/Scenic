# Webots Vacuum Examples

An example showing how to use Scenic to evaluate the coverage of a robot vacuum. 

First navigate to `controllers/create_avoid_obstacles` and run `make` (you may need to first set the Webots environment variable as shown [here](https://cyberbotics.com/doc/guide/compiling-controllers-in-a-terminal)). Then run the scenario using the `worlds/create.wbt` file in webots and play the simulation by pressing one of the buttons at the top (we recommend "Run the simulation as fast as possible" to maximize speed). After Webots closes (indicating all simulations have run), run `python summary.py` to get a summary of the output.

These examples are intended to be run **without** the ``--2d`` flag.
