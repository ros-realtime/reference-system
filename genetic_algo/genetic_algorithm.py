import json
import shutil
import tempfile

import pygad
import pandas as pd
import numpy as np
from reference_system_py.benchmark import get_benchmark_directories_below, generate_trace
from reference_system_py.std_latency import parseLogSummaryFromFiles

n_number_cruncher = 4096 # from:
# /home/kpsruser/colcon_reference-system/src/reference-system/autoware_reference_system/include/autoware_reference_system/system/timing/default.hpp
with open('/home/kpsruser/colcon_reference-system/src/reference-system/autoware_reference_system/src/ros2/data/streaming_policy_ref.json') as f:
    # 
    policy_template = json.load(f)

edges = [('FrontLidarDriver', 'PointsTransformerFront'),
('RearLidarDriver', 'PointsTransformerRear'),
('PointCloudFusion', 'VoxelGridDownsampler'),
('PointCloudMap', 'PointCloudMapLoader'),
('PointCloudFusion', 'RayGroundFilter'),
('EuclideanClusterDetector', 'ObjectCollisionEstimator'),
('BehaviorPlanner', 'MPCController'),
('Lanelet2MapLoader', 'ParkingPlanner'),
('Lanelet2MapLoader', 'LanePlanner'),
('PointsTransformerFront', 'PointCloudFusion'),
('VoxelGridDownsampler', 'NDTLocalizer'),
('MPCController', 'VehicleInterface'),
('Visualizer', 'Lanelet2GlobalPlanner'),
('Lanelet2Map', 'Lanelet2MapLoader'),
('ObjectCollisionEstimator', 'BehaviorPlanner'),
('RayGroundFilter', 'EuclideanClusterDetector'),
('VehicleInterface', 'VehicleDBWSystem'),
('EuclideanIntersection', 'IntersectionOutput'),
('PointsTransformerRear', 'PointCloudFusion'),
('PointCloudMapLoader', 'NDTLocalizer'),
('BehaviorPlanner', 'VehicleInterface'),
('NDTLocalizer', 'Lanelet2GlobalPlanner'),
('Lanelet2GlobalPlanner', 'Lanelet2MapLoader'),
('NDTLocalizer', 'BehaviorPlanner'),
('EuclideanClusterSettings', 'EuclideanClusterDetector'),
('Lanelet2GlobalPlanner', 'BehaviorPlanner'),
('ParkingPlanner', 'BehaviorPlanner'),
('LanePlanner', 'BehaviorPlanner'),
('Lanelet2MapLoader', 'BehaviorPlanner')]

hot_path_nodes = ["FrontLidarDriver",
                 "PointsTransformerFront",
                 "RearLidarDriver",
                 "PointsTransformerRear",
                 "PointCloudFusion",
                 "RayGroundFilter",
                 "EuclideanClusterDetector",
                 "ObjectCollisionEstimator"]

other_nodes = ["PointCloudMap",
    "Visualizer",
    "Lanelet2Map",
    "EuclideanClusterSettings",
    "PointCloudMapLoader",
    "MPCController",
    "VehicleInterface",
    "VehicleDBWSystem",
    "NDTLocalizer",
    "Lanelet2GlobalPlanner",
    "Lanelet2MapLoader",
    "ParkingPlanner",
    "LanePlanner",
    "IntersectionOutput",
    "VoxelGridDownsampler"]


def fitness_func(solution, idx_sol):
    print(idx_sol)
    print(solution)
    update_policy(solution)
    latency_hot_path = calculate_latency_policy()

    return -latency_hot_path


def update_policy(solution):
    base_policy = policy_template.copy()
    node_dicts = dict(zip(hot_path_nodes + other_nodes, solution))
    for composite_node in base_policy['layer_event_loop_map'].keys():
        node = composite_node.split('$')[0]
        if node in node_dicts:
            base_policy['layer_event_loop_map'][composite_node]['coreId'] = int(node_dicts[node])
    # Execute.
    with open('/home/kpsruser/colcon_reference-system/src/reference-system/autoware_reference_system/src/ros2/data/streaming_policy.json', 'w', encoding='utf-8') as f:
        json.dump(base_policy, f, ensure_ascii=False, indent=4)


def calculate_latency_policy():
    trace_type = 'std'
    exe = 'autoware_default_custom'
    rmw = 'rmw_cyclonedds_cpp'
    runtime = 30
    try:
        with tempfile.TemporaryDirectory() as temp_dir_name:
            common_args = {'pkg': 'autoware_reference_system',
                           'directory': temp_dir_name}

            generate_trace(trace_type, exe, rmw=rmw, runtime_sec=runtime, **common_args)

            trace_dirs = get_benchmark_directories_below(common_args['directory'], runtime_sec=runtime)

            data, hot_path_name = parseLogSummaryFromFiles(
                [directory + '/std_output.log' for directory in trace_dirs], runtime)
        latency_hot_path = data[(exe, rmw)]['hot_path']['latency'][-1]['mean']
        print(latency_hot_path)
    except:
        latency_hot_path = 100000
    return latency_hot_path


fitness_function = fitness_func
num_generations = 10
num_parents_mating = 4

gene_type = np.int32
sol_per_pop = 8
num_genes = len(hot_path_nodes) + len(other_nodes)

init_range_low = 0
init_range_high = 4 # not included

parent_selection_type = "sss"
keep_parents = 1

crossover_type = "single_point"

mutation_type = "random"
mutation_percent_genes = 15

ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating,
                       fitness_func=fitness_function,
                       sol_per_pop=sol_per_pop,
                       num_genes=num_genes,
                       gene_type=gene_type,
                       init_range_low=init_range_low,
                       init_range_high=init_range_high,
                       parent_selection_type=parent_selection_type,
                       keep_parents=keep_parents,
                       crossover_type=crossover_type,
                       mutation_type=mutation_type,
                       mutation_percent_genes=mutation_percent_genes)

#ga_instance.run()

filename = f'genetic_{n_number_cruncher}'
ga_instance.save(filename=filename)
#pygad.load(filename=filename)
solution, solution_fitness, solution_idx = ga_instance.best_solution()
update_policy(solution)

src = '/home/kpsruser/colcon_reference-system/src/reference-system/autoware_reference_system/src/ros2/data/streaming_policy.json'
dst =  f'/home/kpsruser/colcon_reference-system/src/reference-system/autoware_reference_system/src/ros2/data/policy_{n_number_cruncher}.json'

shutil.copyfile(src, dst)


print(solution_fitness)