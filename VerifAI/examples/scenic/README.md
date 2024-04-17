# Dynamic Scenic Examples

This folder contains examples of how to use dynamic Scenic scenarios with VerifAI.

## Installation and Compatibility

No additional dependencies are required to use Scenic with VerifAI.
Some of the examples in this folder use the CARLA driving simulator, which only supports Windows and Linux.
It can be set up as follows:

* Install [CARLA](https://carla.readthedocs.io/en/latest/start_quickstart/), version greater than or equal to 0.9.8. Please note that at minimum, a NVIDIA GeForce 470 GTX or AMD Radeon 6870 HD series GPU is required.
* Install the [CARLA Python API](https://carla.readthedocs.io/en/latest/start_quickstart/#install-client-library): enter the virtual environment where VerifAI is installed (if any) and run `pip install carla`.

## Directory Structure

The `falsify_distance.py` example does falsification with a given driving scenario written in Scenic, trying to falsify a specification stating that the ego car always maintains a minimum distance of 5 meters to any other object.
We have included in the `newtonian` and `carla` subfolders examples of Scenic scenarios that work with this falsifier and run in Scenic's built-in Newtonian simulator and the CARLA driving simulator respectively.
These examples are [CARLA Challenge scenarios](https://web.archive.org/web/20221219223342/https://carlachallenge.org/challenge/nhtsa/) derived from the [National Highway Traffic Safety Administration (NHTSA) pre-crash typology](https://www.nhtsa.gov/sites/nhtsa.gov/files/pre-crash_scenario_typology-final_pdf_version_5-2-07.pdf).
Many more examples can be found in the [Scenic repo](https://github.com/BerkeleyLearnVerify/Scenic/tree/master/examples).

(Note: the CARLA examples use the CARLA interface provided by Scenic.
Older versions of VerifAI used a less-capable interface [not supporting dynamic Scenic scenarios] which can still be found in `src/verifai/simulators/carla` if needed.)

## Running the Examples

Instantiate the CARLA simulator server with `./CarlaUE4.sh`, then run `python falsify_distance.py [scenario]`, optionally passing a path to the Scenic scenario to use (if none is provided, the default is `newtonian/carlaChallenge2.scenic`, which does not require an external simulator to be installed).
The `falsify_distance.py` file defines the specification to monitor and the configuration parameters for running VerifAI.
For details on the VerifAI APIs it uses, please refer to [our documentation](https://verifai.readthedocs.io/en/latest/basic_usage.html).

If you use a custom Scenic scenario, note that CARLA scenarios require an OpenDRIVE `.xodr` file to specify the map (see the line in the `.scenic` file starting with `param map =`); our provided Scenic files use the CARLA map `Town01`, which is available elsewhere in the VerifAI repo.
