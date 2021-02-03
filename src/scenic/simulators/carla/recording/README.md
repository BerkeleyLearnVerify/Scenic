# Scenic Data Generation Platform

## Synthetic Dataset

Our synthetic dataset, containing hundreds of simulations of Scenic programs, can be found [at this link](https://drive.google.com/drive/folders/18SrqL2q7PyMfaS0oKAFqoc6hVasXS20I?usp=sharing).

If you wish to generate your own datasets, please follow the setup instructions below. If you're just looking to interact with our dataset above, feel free to skip to the API section.

## Setup

### Installing CARLA
* Download the latest release of CARLA. As of 10/6/20, this is located [here](https://github.com/carla-simulator/carla/releases/tag/0.9.10.1)
    * Other releases can be found [here](https://github.com/carla-simulator/carla/releases)
    * First, download “CARLA_0.9.10.1.tar.gz”. Unzip the contents of this folder into a directory of your choice. In this setup guide, we’ll unzip it into “~/carla”
    * Download “AdditionalMaps_0.9.10.1.tar.gz”. Do not unzip this file. Rather, navigate to “~/carla” (the directory you unzipped CARLA into in the previous step), and place “AdditionalMaps_0.9.10.1.tar.gz” in the “Import” subdirectory.
* In the command line, cd into “~/carla” and run `./ImportAssets.sh`
* Try running `./CarlaUE4.sh -fps=15` from the “~/carla” directory. You should see a window pop up containing a 3D scene.
* The CARLA release contains a Python package for the API. To use this, you need to add the package to your terminal’s PYTHONPATH variable as follows:
    * First, copy down the filepath of the Python package. The package should be located in “~/carla/PythonAPI/carla/dist”. Its name should be something like “carla-0.9.10-py3.7-linux-x86_64.egg”
    * Open your “~/.bashrc” file in an editor. Create a new line with the following export statement: “export PYTHONPATH=/path/to/egg/file”
    * Save and exit “~/.bashrc” and restart the terminal for the changes to take effect. To confirm that the package is on the PYTHONPATH, try the command “echo $PYTHONPATH”

### Installing Scenic
* In a new terminal window, clone the current repository.
* In the command line, enter the repository and switch to the branch “dynamics2-recording”
* Run `poetry install` followed by `poetry shell`
* You’re now ready to run dynamic Scenic scripts! Here’s an example: `python -m scenic -S --time 200 --count 3 -m scenic.simulators.carla.model /path/to/scenic/script`
    * This creates 3 simulations of the specified Scenic script, each of which runs for 200 time steps. Some example Scenic scripts are located in “examples/carla”

## Dataset Generation

To generate a synthetic dataset using Scenic, you need two things: a scenario configuration file and a sensor configuration file.

### Scenario Configuration

This file lets you configure which Scenic programs to simulate, how many times to simulate each program, how many steps to run each simulation for, and where to output the generated data.

A sample scenario configuration file, which must be saved in the JSON format, is shown below. Feel free to change the list of scripts to reference any Scenic programs on your machine.

```
{
   "output_dir": "/path/to/output/dir", // dataset output directory
   "simulations_per_scenario": 3, // number of simulations per Scenic program (script)
   "time_per_simulation": 300, // time steps per simulation
   "scripts": [
      "/path/to/scenario1",
      "/path/to/scenario2"
    ]
}
```

### Sensor Configuration

This file is another JSON file that lets you configure the number, placement, and type of sensors with which to record. Right now, RGB video cameras and lidar sensors are supported (with ground-truth annotations). An example configuration file is as follows:

```
{
   [
	{
		"name": "cam", // each sensor must have an associated unique name
		"type": "rgb",
		"transform": [0, 0, 2.4], // sensor xyz coordinates with respect to ego vehicle
		"settings": {
			"VIEW_WIDTH": 1280, // horizontal resolution in pixels
			"VIEW_HEIGHT": 720, // vertical resolution in pixels
			"VIEW_FOV": 90 // horizontal field of view
		}
	},
	{
		"name": "lidar",
		"type": "lidar",
		"transform": [0, 0, 2.4], // sensor xyz coordinates with respect to ego vehicle
		"settings": {
			"PPS": 400000, // number of points to record per second
			"UPPER_FOV": 15.0,
			"LOWER_FOV": -25.0, // combined 40 degree field of view
			"RANGE": 40, // range of sensor in meters
			"ROTATION_FREQUENCY": 18.0 // frequency of rotation per second (should be close to simulation fps)
		}
	}
]
```

In fact, this was the exact sensor configuration file that we used to generate our synthetic dataset.

Now, to actually generate data using the configurations above, simply run:

```
python -m scenic.simulators.carla.recording --scenarios /path/to/scenario/config --sensors /path/to/sensor/config
```

Remember to enter `poetry shell` before running this command so that Scenic is properly set up as a module. Your dataset is on its way!

## API

Once you've either downloaded our provided dataset or generated one of your own, you can browse the data using our API:

```
from scenic.simulators.carla.recording import *
```

Load the sensor configuration file into a `SensorConfig` object:

```
sensor_config = SensorConfig('/path/to/sensor/config/file')
```

Load the generated dataset. The dataset directory here is the same as the "output_dir" specified in the scenario configuration file. If you're using our provided dataset, the dataset path will be the full directory path of "Town03", "Town05", "Town10", or "dense".

```
data = DataAPI('/path/to/dataset/directory', sensor_config)
```

Now you may browse the data as you please. The following example demonstrates how to draw 3D bounding boxes onto a frame selected from a particular simulation:

```
from scenic.simulators.carla.recording import *

DATA_DIR = '/data/scenic_data_collection/Town03'
SENSOR_CONFIG_FILE = 'sensor_config.json'

sensor_config = SensorConfig(SENSOR_CONFIG_FILE)

data = DataAPI(DATA_DIR, sensor_config)

sims = data.get_simulations()
sims = list(sims.values())
sim = sims[0]

frame = sim.get_frame(10)
draw_bbox_3d(frame['bboxes'], sensor_config.get('cam'), frame['cam']['rgb'], 'frame.jpg')
```

### API Documentation

#### class DataAPI
* def get_simulations(self)
    * Returns simulation data as a dictionary, with keys as the simulation name and values as `SimulationData` objects.

#### class SimulationData
* def get_frame(self, frame_idx)
    * Returns all sensor data recorded at a particular frame index (time step). For example, if we recorded with an RGB camera named "cam" and a lidar sensor named "lidar", then this function would return:
```
{
	"bboxes": [list of 3D bounding boxes],
	"cam": {
		"rgb": image as numpy array,
		"depth": image as numpy array,
		"semantic": image as numpy array
	},
	"lidar": {
		"lidar": [list of lidar points]
	}
}
```

#### class SensorConfig
* def get(self, sensor_name)
    * Returns a dictionary object representing the sensor with name `sensor_name`. This sensor dictionary object should be used when working with the helper functions for drawing bounding boxes.

The API includes utility functions to draw 2D and 3D bounding boxes onto any camera of choice, as well as to output a labeled point cloud that can be opened with software such as CloudCompare:

* def draw_bbox_2d(bboxes, sensor, img, output_filepath)
    * `bboxes`: a list of 3D bounding boxes as returned by `SimulationData.get_frame()`
    * `sensor`: a sensor dictionary object as described above
    * `img`: numpy array of the image frame to draw on
    * `output_filepath`: where to output the final image
    * This function draws 2D bounding boxes onto an image captured by a particular sensor

The function for drawing 3D bounding boxes is similar. For outputting labeled point clouds, we have the following function:

* def save_point_cloud(lidar_points, output_filepath)
    * `lidar_points`: list of lidar points as returned by `SimulationData.get_frame()`
    * `output_filepath`: where to output the point cloud (should have extension `.asc`)

Ground-truth annotations contain semantic labels provided by CARLA, a complete description of which can be found [here](https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera) (scroll to the table of semantic tags).
