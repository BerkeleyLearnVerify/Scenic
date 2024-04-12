# Multi-objective & Parallel Falsification Examples

This folder contains examples of performing falsification with multiple objectives and in parallel.
These examples all use dynamic Scenic, so if you aren't familiar with how to use Scenic with VerifAI, you might want to start with the examples in the `examples/scenic` folder.

The driver code for all of the examples is in the `multi_objective_falsification.py` file.
The file takes a variety of options to configure the experiment to be run; for example, here are several possibilities:

```
# Run falsification for 5 iterations
python multi_objective_falsification.py -n 5
# Likewise, with a different scenario
python multi_objective_falsification.py -n 5 -p intersection_01.scenic
# Run falsification for 15 iterations, with 4 parallel workers
python multi_objective_falsification.py -n 15 --parallel --num-workers 4
```

The script uses Scenic's built-in Newtonian simulator by default, so for each of the above commands you should see one or more ``pygame`` windows appear to visualize the simulations being run.
The example scenarios we have provided will also work in CARLA and other simulators supporting Scenic's abstract driving domain.
