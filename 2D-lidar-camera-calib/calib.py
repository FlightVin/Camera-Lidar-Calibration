import cv2
import numpy as np
import math
import sys
from scipy.spatial.transform import Rotation

# defining configs
configs = { # Intrinsics from ROS
    "fx": 1345.61962890625,
    "fy": 1345.61962890625,
    "cx": 960.0,
    "cy": 540.0, 
    "k1": 0.0, 
    "k2": 0.0, 
    "p1": 0.0, 
    "p2": 0.0,
}

if __name__ == "__main__":
    try:
        data_file = sys.argv[1]
    except:
        raise ValueError("Expected data file")

    camera_intrinsics = np.array([
        [configs["fx"], 0, configs["cx"]],
        [0, configs["fy"], configs["cy"]],
        [0, 0, 1]
    ])

    distortions = np.array(
        [configs["k1"], configs["k2"], configs["p1"], configs["p2"]]
    ) # tangential and radial distortions

    print("Camera Intrinsics")
    print(camera_intrinsics)
    print()

    print("Distortion Paramaters")
    print(distortions)
    print()

    lidar_coords, image_coords = [], []
    with open(data_file) as fobj:
        data = fobj.readlines()
        for row in data:
            stripped_row = row.strip().split()
            
            lidar_coords.append([float(stripped_row[1]), float(stripped_row[0]), 1])
            image_coords.append([float(stripped_row[2]), float(stripped_row[3])])

    lidar_coords, image_coords = np.array(lidar_coords), np.array(image_coords)

    D_0 = np.array([0.0, 0.0, 0.0, 0.0])
    retval, rvec, tvec = cv2.solvePnP(lidar_coords, image_coords, camera_intrinsics, D_0, flags = cv2.SOLVEPNP_ITERATIVE)
    rotation_matrix, jac = cv2.Rodrigues(rvec)

    print("Extrinsics")
    print("T = ")
    print(tvec)
    print()


    # corrections for the way the data was collected
    # must change roll and yaw as the pointcloud collected had transformations in these
    roll_angle = 90 
    yaw_angle = 180

    # Create Rotation objects for the desired rotations
    r_roll = Rotation.from_euler('x', roll_angle, degrees=True)
    r_yaw = Rotation.from_euler('z', yaw_angle, degrees=True)
    rot_mat = Rotation.from_matrix(rotation_matrix)
    rot_mat = r_yaw * r_roll * rot_mat


    print("R = ")
    print(rot_mat.as_matrix())
    print()

    xyz_angles = rot_mat.as_euler('xyz', degrees=True)

    print("Roll:", xyz_angles[0], "degrees")
    print("Pitch:", xyz_angles[1], "degrees")
    print("Yaw:", xyz_angles[2], "degrees")

