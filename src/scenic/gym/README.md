# Scenic Gym

This repository contains the code for ScenicGym, the Scenic functionality for training RL agents.

Installation:
- Install Scenic as usual
- Install Gymnasium/Open AI Gym.

In this directory:
- `envs` contains the envrionment classes for ScenicGym. See the examples for how to use them.
- There are three variants of the ScenicGym envrionments. One that inherits from `Gymnasium.Env`, made to be compatible for the current Gymnasium maintained by Farama. One that inherits from `gym.Env` by OpenAI. And another one that inherits from `gym.Env` for OpenAI Gym versions that uses the same API 0.15.
- The `examples/` folder in this directory contains a short notebook illustrating the API, `API_example.ipynb`.
- Note: the API is not yet finalized, and we are collecting user feedback on what API works best. Please contact us if you would like help in adapting ScenicGym to your project. 
-
Additional usage notes:
- For older Open AI gym uses, please use the `ScenicOAI15GymEnv` class in `envs/scenic_oai_15_gym.py` for your training
- For specific simulators dependencies, please refer to the simualtor interface docs. 
