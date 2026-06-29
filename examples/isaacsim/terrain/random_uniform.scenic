model scenic.simulators.isaac.model

param isaacLab = True

param verifaiSamplerType = "bo"

param uniform_noise_upper = VerifaiRange(0.4, 1.5)
param uniform_noise_range = (VerifaiRange(0.1, 0.4), globalParameters.uniform_noise_upper)

param discrete_obstacles_max_height = VerifaiRange(0.1, 0.7)
param discrete_obstacles_min_size = VerifaiRange(0.1, 0.3)
param discrete_obstacles_max_size = VerifaiRange(1.0, 2.0)
param discrete_obstacles_num_rects = VerifaiDiscreteRange(100, 150)
param discrete_obstacles_platform_size = VerifaiRange(3, 5)

param pyramid_stairs_step_width = VerifaiRange(0.5, 1.5)
param pyramid_stairs_step_height = VerifaiRange(0.1, 0.4)

param slope = VerifaiRange(0.3, 0.8)

terrain = new RandomUniformTerrain at (0, 0, 0), 
                with width 20, 
                with length 20, 
                with noise_range globalParameters.uniform_noise_range

pyramid_sloped_terrain = new PyramidSlopedTerrain right of terrain, 
                with width 20, with length 20, 
                with slope globalParameters.slope

discrete_obstacles_terrain = new DiscreteObstaclesTerrain ahead of pyramid_sloped_terrain, 
                                with width 20, 
                                with length 20, 
                                with max_height globalParameters.discrete_obstacles_max_height, 
                                with min_size globalParameters.discrete_obstacles_min_size, 
                                with max_size globalParameters.discrete_obstacles_max_size, 
                                with num_rects globalParameters.discrete_obstacles_num_rects, 
                                with platform_size globalParameters.discrete_obstacles_platform_size

pyramid_stairs_terrain = new PyramidStairsTerrain left of discrete_obstacles_terrain, 
                            with width 20, 
                            with length 20, 
                            with step_width globalParameters.pyramid_stairs_step_width, 
                            with step_height globalParameters.pyramid_stairs_step_height
