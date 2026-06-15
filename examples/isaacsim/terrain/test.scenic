model scenic.simulators.isaac.model

param isaacLab = True

param uniform_noise_range = (VerifaiRange(0.1, 0.4), VerifaiRange(0.4, 1.5))

param discrete_obstacles_max_height = VerifaiRange(0.1, 0.7)
param discrete_obstacles_min_size = VerifaiRange(0.1, 0.3)
param discrete_obstacles_max_size = VerifaiRange(1.0, 2.0)
param discrete_obstacles_num_rects = VerifaiDiscreteRange(100, 150)
param discrete_obstacles_platform_size = VerifaiRange(3, 5)

param pyramid_stairs_step_width = VerifaiRange(0.5, 1.5)
param pyramid_stairs_step_height = VerifaiRange(0.1, 0.4)

param slope = VerifaiRange(0.3, 0.8)

stairs_terrain = new PyramidStairsTerrain at (0, 0, 0),
                 with width 20,
                 with length 20,
                 with step_width 0.2,
                 with step_height 0.1
