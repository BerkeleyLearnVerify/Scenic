# Driving Examples

This folder contains cross-platform examples using Scenic's abstract domain for driving scenarios.
The driving domain does not yet support 3D geometry, so these examples must be run with the `--2d` command-line option.
For example:

```
scenic --2d badlyParkedCarPullingIn.scenic
scenic --2d -S --model scenic.simulators.metadrive.model badlyParkedCarPullingIn.scenic
```
