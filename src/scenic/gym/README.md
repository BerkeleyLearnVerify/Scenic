# Scenic Gym

This repository contains the code for ScenicGym, the Scenic functionality for training RL agents.

In this directory:
- `envs` contains the envrionment classes for ScenicGym. See the examples for how to use them.
- There are three variants of the ScenicGym envrionments. One that inherits from `Gymnasium.Env`, made to be compatible for the current Gymnasium maintained by Farama. One that inherits from `gym.Env` by OpenAI. And another one that inherits from `gym.Env` for OpenAI Gym versions that uses the same API 0.15.
- For the envrionments based on PettingZoo, please see the `zoo` module, `Scenic/src/scenic/zoo/`.
