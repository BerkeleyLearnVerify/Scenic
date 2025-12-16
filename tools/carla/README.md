# Dynamic CARLA blueprints

This folder has two scripts that **collect CARLA blueprint IDs/dimensions** and **generate** the `blueprints.py` module used by Scenic.

## Quick Start

    # From tools/carla:
    python snapshot_blueprints.py	# capture a snapshot from the running CARLA server
    python make_blueprints.py 		# build src/scenic/simulators/carla/blueprints.py

## What the scripts do

### `snapshot_blueprints.py`

Connects to CARLA (starts it headless if needed, requires `CARLA_ROOT`).
Make sure the CARLA server and Python API versions match.
Groups blueprints into categories, measures bounding-box dimensions, and writes:
`tools/carla/snapshots/blueprints_<SERVER_VERSION>.json`
Run this once for each CARLA version you want to support.

### `make_blueprints.py`

Reads all `tools/carla/snapshot/blueprints_*json`, sorts versions semantically (newest → oldest), normalizes category/ID ordering for stable diffs, and writes: `src/scenic/simulators/carla/blueprints.py`
