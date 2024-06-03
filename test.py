import numpy as np
import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network

# Define the range and step size for the grid search
longitudinal_kp_range = np.arange(0.01, 1.0, 0.05)
longitudinal_kd_range = np.arange(0.01, 1.0, 0.05)
longitudinal_ki_range = np.arange(0.01, 1.0, 0.05)
lateral_kp_range = np.arange(0.01, 1.0, 0.05)
lateral_kd_range = np.arange(0.01, 1.0, 0.05)
lateral_ki_range = np.arange(0.01, 1.0, 0.05)

network = Network.fromFile('./assets/maps/CARLA/Town01.xodr')
best_params = None
best_distance = float('inf')

for Longitudinal_K_P in longitudinal_kp_range:
    for Longitudinal_K_D in longitudinal_kd_range:
        for Longitudinal_K_I in longitudinal_ki_range:
            for Lateral_K_P in lateral_kp_range:
                for Lateral_K_D in lateral_kd_range:
                    for Lateral_K_I in lateral_ki_range:
                        simulator = NewtonianSimulator(
                            render=True, network=network,
                            Longitudinal_K_P=Longitudinal_K_P,
                            Longitudinal_K_D=Longitudinal_K_D,
                            Longitudinal_K_I=Longitudinal_K_I,
                            Lateral_K_P=Lateral_K_P,
                            Lateral_K_D=Lateral_K_D,
                            Lateral_K_I=Lateral_K_I
                        )
                        scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
                        scene, _ = scenario.generate(maxIterations=1000)
                        simulation = simulator.simulate(scene)
                    
                        if simulation == None:
                            print(f"Params: {Longitudinal_K_P}, {Longitudinal_K_D}, {Longitudinal_K_I}, {Lateral_K_P}, {Lateral_K_D}, {Lateral_K_I} -> Did not compute result object!")
                            continue
                        distances = simulation.result.records
                        
                        # Compute the average distance from the centerline
                        total_distance = 0
                        count = 0
                        for points in distances.values():
                            for point in points:
                                _, y = point
                                total_distance += y
                                count += 1
                        if count > 0:
                            average_distance_from_center_lane = total_distance / count
                        else:
                            average_distance_from_center_lane = float('inf')
                        
                        print(f"Params: {Longitudinal_K_P}, {Longitudinal_K_D}, {Longitudinal_K_I}, {Lateral_K_P}, {Lateral_K_D}, {Lateral_K_I} -> Avg. Distance: {average_distance_from_center_lane}")
                        
                        if average_distance_from_center_lane < best_distance:
                            best_distance = average_distance_from_center_lane
                            best_params = (Longitudinal_K_P, Longitudinal_K_D, Longitudinal_K_I, Lateral_K_P, Lateral_K_D, Lateral_K_I)

print("Best params:", best_params)
print("Best average distance from center lane:", best_distance)