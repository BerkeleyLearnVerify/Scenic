# AirSim example

This directory contains a minimal Scenic example for controlling a drone in
[Microsoft AirSim](https://github.com/microsoft/AirSim).

The example has been tested with AirSim 1.8.1 on Ubuntu 22.04. AirSim requires
a graphical environment and is not supported on macOS.

## Install the Python dependencies

Create and activate a virtual environment, then install Scenic from the
repository:

```bash
python3 -m venv .venv-airsim
source .venv-airsim/bin/activate
python -m pip install --upgrade pip wheel
python -m pip install -e .
```

Install the AirSim client and its additional dependencies:

```bash
python -m pip install msgpack-rpc-python promise
python -m pip install --no-build-isolation airsim==1.8.1
```

The following NumPy and OpenCV versions were used during testing:

```bash
python -m pip install --force-reinstall \
  numpy==1.26.4 \
  opencv-contrib-python==4.10.0.84
```

Check the resulting environment:

```bash
python -m pip check
```

## Start AirSim

Download or build an AirSim environment. For example, launch the AirSim 1.8.1
Blocks environment with the settings included in this directory:

```bash
/path/to/Blocks.sh \
  -settings="/path/to/Scenic/examples/airsim/settings/simple_drone_settings.json" \
  -windowed \
  -ResX=640 \
  -ResY=480 \
  -NoVSync
```

These window settings are conservative defaults that work well on a remote
desktop. They can be adjusted for the available display and hardware.

Run AirSim from a graphical desktop session and leave it open while Scenic is
running.

## Generate world information

The Scenic AirSim model requires information about the objects and assets in
the selected AirSim environment.

Start AirSim first. Then, from the Scenic repository, run:

```bash
python src/scenic/simulators/airsim/generators/generateWorldInfo.py \
  --outputDirectory "/path/to/blocks-world-info"
```

The output directory must not already exist. Generating the meshes may take
several minutes.

The resulting directory contains:

```text
worldInfo.json
objectMeshes/
assets/
```

Keep this generated directory outside the Scenic repository. Pass its location
to Scenic using the `worldInfoPath` parameter.

## Run the patrol example

With AirSim running, open another terminal, activate the Python environment,
and run:

```bash
scenic examples/airsim/patrol.scenic \
  --simulate \
  --count 1 \
  --time 30 \
  --param worldInfoPath "/path/to/blocks-world-info"
```

The example creates one drone and sends it through a fixed sequence of patrol
points.
