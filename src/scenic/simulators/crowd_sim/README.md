# CrowdSim Interface

This simulator interface is used for training social navigation. The referces we used is at the bottom. Note that
we are using a modified version of the CrowdSim simulator used in the reference. We made the modification so that Scenic can be interfaced.

## Getting started
- Environment setup can be done by creating a Conda/Mamba environment using `corwd_env_linux.yml`.
- Note that this does not install Scenic/VerifAI, which should be installed according to their corresponding docs.
- After installing the dependencies, run `git submodule init` to install the simulator in this interface directory.
- In the submodule, `crowd_nav_fork`, checkout the branch `scenic-train-sim-mod-train-mod-gymnasium-cleanup`.
- Install the dependencies as described in the README of the submodule.
- For running example training, `cd crowd_nav_fork` and execute `python train_scenic.py`. This runs the training without
any rendering.
- Run `python test_scenic_train.py` in `crowd_nav_fork` to see what the simulation looks like during training with Scenic 

## References
- https://sites.google.com/view/intention-aware-crowdnav/home
