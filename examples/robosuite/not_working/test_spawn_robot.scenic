# examples/robosuite/test_spawn_robot.scenic


# test_panda_empty.scenic
"""Test Panda in empty environment."""

model scenic.simulators.robosuite.model

# Override the default task to use an empty environment
class PandaEmpty(Panda):
    task: "Door"  # Try Door task which might be simpler

panda = new PandaEmpty

terminate after 10 steps