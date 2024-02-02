import os
import numpy as np

def load_dataset(dataset_dir_path):
    world_points = []
    board_points = []

    for file in os.listdir(dataset_dir_path):
        path = os.path.join(dataset_dir_path, file)
        if file == 'world_points.txt':
            world_points.append(load_data(path))
        else:
            board_points.append(load_data(path))

    return world_points[0], board_points

def load_data(data_path):
    point_data = []
    with open(data_path, 'r') as f:
        for line in f:
            line = line.strip().split()
            point = []
            for coord in line:
                point.append(float(coord))
                point_data.append(point)

    point_data = np.array(point_data)
    return point_data