# Dynamic CARLA blueprints

This folder has two scripts that **collect CARLA blueprint IDs/dimensions** and **generate** the auto-generated data module used by 
`scenic.simulators.carla.blueprints`.

## Quick Start

    # From tools/carla:
    python snapshot_blueprints.py  # capture a snapshot from the running CARLA server
    python make_blueprints.py  # build src/scenic/simulators/carla/_blueprintData.py

## What the scripts do

### `snapshot_blueprints.py`

Connects to CARLA (starts it headless if needed, requires `CARLA_ROOT`).
Make sure the CARLA server and Python API versions match.
Groups blueprints into categories, measures bounding-box dimensions, and writes a compressed snapshot:
`tools/carla/snapshots/blueprints_<SERVER_VERSION>.json.gz`

(The `blueprints_*.json.gz` files are gzip-compressed JSON; you can decompress
one with `gunzip -k blueprints_0.9.15.json.gz` if you want to inspect it.)

Run this once for each CARLA version you want to support.

### `make_blueprints.py`

Reads all `tools/carla/snapshots/blueprints_*.json.gz`, sorts versions semantically (newest → oldest), normalizes category/ID ordering for stable diffs, and writes: 
`src/scenic/simulators/carla/_blueprintData.py`

