# Scenic Gym

This repository contains the code for ScenicGym, the Scenic functionality for training RL agents.

### Installation:
- Install Scenic as usual
- Install Gymnasium/Open AI Gym.

### In this directory:
- `envs` contains the envrionment classes for ScenicGym. See the examples for how to use them.
- There are three variants of the ScenicGym envrionments. One that inherits from `Gymnasium.Env`, made to be compatible for the current Gymnasium maintained by Farama. One that inherits from `gym.Env` by OpenAI. And another one that inherits from `gym.Env` for OpenAI Gym versions that uses the same API 0.15.
- The `examples/` folder in this directory contains a short notebook illustrating the API, `API_example.ipynb`.
- Note: the API is not yet finalized, and we are collecting user feedback on what API works best. Please contact us if you would like help in adapting ScenicGym to your project. 

### Other Notes on Using ScenicGym
- In this tentative API, there are a few functions you need to add to the simulator interface you are using. Specifically, they should be defined in the Simulation class in the interface's `simulator.py`
- `Simulation.get_obs()`; this should return the observation at that timestep
- `Simulation.get_reward()`; this should return the reward at that timestep
- `Simulation.get_info()`; this should return the additional info at that timestep
- Please let us know if you have suggestions for the final API

### Additional usage notes:
- For older Open AI gym uses, please use the `ScenicOAI15GymEnv` class in `envs/scenic_oai_15_gym.py` for your training
- For specific simulators dependencies, please refer to the simualtor interface docs. 
